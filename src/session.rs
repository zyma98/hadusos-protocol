//! Implementation for half-duplex session handling. The figure below outlines
//! the interaction between the sender and the receiver in time sequence.
//!
//! ```plain
//!        |                                          |
//!        |  send()                         listen() |
//!        |--------+                      +----------|
//!        |        |-----SEND REQUEST---->|          |
//!        |        |                      +--------->|
//!        |        |                                 |
//!        |        |                       receive() |
//!        |        |                      +----------|
//!        |        |<---SEND CLEARANCE----|          |
//! SENDER |        |                      |          | RECEIVER
//!        |        |---------DATA-------->|          |
//!        |        |<--------ACK----------|          |
//!        |        |         ...          |          |
//!        |        |---------DATA-------->|          |
//!        |        |<--------ACK----------|          |
//!        |        |                      |          |
//!        |        |---------RESET------->|          |
//!        |<-------+                      +--------->|
//!        |                                          |
//! ```

use crate::{
    link::Link,
    packet::{self, Acknowledge, Packet, PacketContent, PacketError, Scratchpad, Sequence},
    serial::Serial,
    timer::Timer,
};

/// Enumeration of possible session errors.
#[derive(Debug)]
pub enum SessionError<RE, WE> {
    /// Error occurred during serial read.
    SerialReadErr(RE),
    /// Error occurred during serial write.
    SerialWriteErr(WE),
    /// Cannot send because the other end was not ready to receive.
    NotClearToSend,
    /// Operation timed out.
    Timeout,
    /// Received clobbered packet. Retransmission did not fix the problem.
    Clobbered,
    /// The other end actively wanted to abort the session.
    Reset,
    /// A session was established but the sender did not send data for very
    /// long time.
    Disconnected,
    /// A session was established but the receiver did not acknowledge the
    /// data sent.
    NotAcknowledged,
    /// The other end is not following the procedure of session initialization
    /// handshake or the procedure of data acknowledgement. Maybe the other end
    /// has proceeded to a new session.
    OutOfSync,
    /// Cannot send the data because it is too large.
    Oversize,
    /// Buffer provided does not match the advertised incoming data length.
    BufferSizeMismatch,
}

#[doc(hidden)]
/// Implement conversion from [`PacketError`] to [`SessionError`].
impl<RE, WE> From<PacketError<RE, WE>> for SessionError<RE, WE> {
    fn from(pe: PacketError<RE, WE>) -> Self {
        match pe {
            PacketError::SerialReadErr(e) => SessionError::SerialReadErr(e),
            PacketError::SerialWriteErr(e) => SessionError::SerialWriteErr(e),
            PacketError::Timeout => SessionError::Timeout,
            PacketError::Clobbered => SessionError::Clobbered,
            // We do not provide a buffer for receiving when we do not expect
            // a data packet containing client data. If the received packet
            // happens to be such a data packet, treat it as cloberred.
            PacketError::NoBuffer => SessionError::Clobbered,
        }
    }
}

/// In some cases a particular session error is not fatal. For example,
/// receiving a cloberred data packet can be recovered by a retransmission.
/// To distinguish between fatal errors versus non-fatal ones, we convert the
/// two-variant result, i.e. either `Ok` or `Err`, to a three-variant type:
/// - `Ok(Some(_))`: No error
/// - `Ok(None)`: Non-fatal error
/// - `Err(_)`: Fatal error
trait IntoTolerable<T, RE, WE> {
    fn into_tolerable(self) -> Result<Option<T>, SessionError<RE, WE>>;
}

impl<T, RE, WE> IntoTolerable<T, RE, WE> for Result<T, SessionError<RE, WE>> {
    /// Convert `Ok(v)` to `Ok(Some(v))`, keep `Err(e)` as `Err(e)`. See
    /// [`IntoTolerable`] for explanation.
    fn into_tolerable(self) -> Result<Option<T>, SessionError<RE, WE>> {
        self.map(|v| Some(v))
    }
}

/// Make the given session error variant non-fatal. See [`IntoTolerable`] for
/// explanation.
macro_rules! tolerate_error {
    ($name:ident, $error_variant:pat) => {
        fn $name(self) -> Self {
            match self {
                Ok(v) => Ok(v),
                Err(e) => match e {
                    $error_variant => Ok(None),
                    e => Err(e),
                },
            }
        }
    };
}

/// Trait providing methods to tolerate non-fatal errors for [`Result`] with
/// [`SessionError`]. See [`IntoTolerable`] for explanation.
trait TolerableError<T, RE, WE> {
    /// Turn [`Err(SessionError::Timeout)`](SessionError) to `Some(None)`.
    fn tolerate_timeout(self) -> Self;
    /// Turn [`Err(SessionError::Clobbered)`](SessionError) to `Some(None)`.
    fn tolerate_clobber(self) -> Self;
    /// Turn [`Err(SessionError::OutOfSync)`](SessionError) to `Some(None)`.
    fn tolerate_out_of_sync(self) -> Self;
    /// Turn [`Err(SessionError::Reset)`](SessionError) to `Some(None)`.
    fn tolerate_reset(self) -> Self;
}

impl<T, RE, WE> TolerableError<T, RE, WE> for Result<Option<T>, SessionError<RE, WE>> {
    tolerate_error!(tolerate_timeout, SessionError::Timeout);
    tolerate_error!(tolerate_clobber, SessionError::Clobbered);
    tolerate_error!(tolerate_out_of_sync, SessionError::OutOfSync);
    tolerate_error!(tolerate_reset, SessionError::Reset);
}

/// The struct representing the session layer.
///
/// # Generic Parameters
/// - `RTO`: Retransmission timeout in milliseconds.
/// - `RRC`: Retransmission retry count.
pub struct Session<S, T, const RTO: u32 = 50, const RRC: usize = 3>
where
    S: Serial,
    T: Timer,
{
    /// The underlying link layer instance.
    link: Link<S, T>,
    /// The number identifying the session as a sender.
    tx_session_num: u16,
    /// The number identifying the session as a receiver.
    rx_session_num: u16,
    /// The length of data going to be received from the sender.
    rx_data_len: u16,
    /// An opaque type intance that should be passed to the functions when
    /// receiving a packet.
    scratchpad: Scratchpad,
}

/// Public functions and methods.
impl<S, T, const RTO: u32, const RRC: usize> Session<S, T, RTO, RRC>
where
    S: Serial,
    T: Timer,
{
    /// Create a new session layer instance.
    ///
    /// # Parameters
    /// - `serial`: A serial instance implementing the [`Serial`] trait.
    /// - `timer`: A timer instance implementing the [`Timer`] trait.
    pub const fn new(serial: S, timer: T) -> Self {
        Self {
            link: Link::new(serial, timer),
            tx_session_num: 0,
            rx_session_num: 0,
            rx_data_len: 0,
            scratchpad: Scratchpad::new(),
        }
    }

    /// Send the given data with a timeout.
    ///
    /// # Parameters
    /// - `data`: The data to be sent.
    /// - `timeout_ms`: The timeout in milliseconds.
    ///
    /// # Returns
    /// - `Ok(())`: The data was successfully sent and acknowledged by the
    ///   receiving end.
    /// - [`Err(SessionError)`](SessionError): An error occurred, which could
    ///   be a timeout.
    pub fn send(
        &mut self,
        data: &[u8],
        timeout_ms: u32,
    ) -> Result<(), SessionError<S::ReadError, S::WriteError>> {
        let update_timeout = self.get_timeout_update_func(timeout_ms);

        // Send a request to the receiver for sending clearance. Wait until the
        // clearance is received. Use the given timeout as this is the first
        // blocking operation.
        self.initiate_session(data.len(), timeout_ms)?;

        // Send the data to the receiver. Use an updated timeout as some time
        // has elapsed.
        let remaining_timeout = update_timeout(&mut self.link.get_timer())?;
        self.send_data(data, remaining_timeout)?;

        // Send a reset packet to the receiver to end the session.
        self.finalize_session()
    }

    /// Wait for a send request with a timeout.
    ///
    /// # Parameters
    /// - `timeout_ms`: The timeout in milliseconds.
    ///
    /// # Returns
    /// - `Ok(u16)`: The length of the data the other end intends to send.
    /// - [`Err(SessionError)`](SessionError): An error occurred, which could
    ///   be a timeout.
    pub fn listen(
        &mut self,
        timeout_ms: u32,
    ) -> Result<u16, SessionError<S::ReadError, S::WriteError>> {
        let update_timeout = self.get_timeout_update_func(timeout_ms);

        // Keep receiving packets until successfully getting a send request
        // packet or timeout.
        loop {
            // Will return timeout error here if the timeout set by client has
            // expired.
            let remaining_timeout = update_timeout(self.link.get_timer())?;

            // Receive a packet.
            let packet = match Packet::receive(
                &mut self.link,
                None,
                remaining_timeout,
                &mut self.scratchpad,
            )
            // Convert `PacketError` to `SessionError`.
            .map_err(Into::into)
            // Make packet cloberring a recoverable error.
            .into_tolerable()
            .tolerate_clobber()?
            {
                Some(packet) => packet,
                // If the packet is cloberred, send a reset packet so that the
                // other end can take further actions as quickly as possible
                // rather than wait for a timeout.
                None => {
                    Packet::build_reset().send(&mut self.link)?;
                    continue;
                }
            };

            // If the received packet is a send reqeust, save the advertised
            // data length and session number and return.
            if let PacketContent::SendRequest {
                data_len,
                session_num,
            } = packet.get_content()
            {
                self.rx_data_len = data_len;
                self.rx_session_num = session_num;
                return Ok(data_len);
            // Otherwise the packet is not expected. Send a reset packet so
            // that the other end can take further actions as quickly as
            // possible rather than wait for a timeout. Loop to tay again.
            } else {
                Packet::build_reset().send(&mut self.link)?;
            }
        }
    }

    /// Approve the previously received send request and receive data with a
    /// timeout.
    ///
    /// # Parameters
    /// - `data`: The buffer to hold incoming data. The buffer size must
    ///   match exactly the size previously received in the send request.
    /// - `timeout_ms`: The timeout in milliseconds.
    ///
    /// # Returns
    /// - `Ok(())`: The data was successfully received and stored to the given
    ///   buffer.
    /// - [`Err(SessionError)`](SessionError): An error occurred, which could
    ///   be a timeout.
    pub fn receive(
        &mut self,
        data: &mut [u8],
        timeout_ms: u32,
    ) -> Result<(), SessionError<S::ReadError, S::WriteError>> {
        // Check provided buffer size.
        if data.len() != self.rx_data_len as usize {
            return Err(SessionError::BufferSizeMismatch);
        }

        let update_timeout = self.get_timeout_update_func(timeout_ms);

        // Send a clearance packet to the sender and wait for the first data
        // packet. Also acknowledge the first data packet to the sender. Use
        // the given timeout as this is the first blocking operation.
        self.accept_new_session(data, timeout_ms)?;

        // Wait and receive the remaining data packets and acknowledge them.
        // Use an updated timeout as some time has elapsed.
        let remaining_timeout = update_timeout(&mut self.link.get_timer())?;
        let final_seq = self.accept_remaining_data(data, remaining_timeout)?;

        // Wait for the reset packet. Use an updated timeout as some time has
        // elapsed.
        let remaining_timeout = update_timeout(&mut self.link.get_timer())?;
        self.accept_session_termination(final_seq, remaining_timeout)
    }

    /// Reject the previously received send request.
    ///
    /// # Returns
    /// - `Ok(())`: Rejection successfully sent.
    /// - [`Err(SessionError)`](SessionError): An error occurred.
    pub fn reject(&mut self) -> Result<(), SessionError<S::ReadError, S::WriteError>> {
        Packet::build_reset()
            .send(&mut self.link)
            .map_err(Into::into)
    }
}

/// Private functions and methods.
impl<S, T, const RTO: u32, const RRC: usize> Session<S, T, RTO, RRC>
where
    S: Serial,
    T: Timer,
{
    /// Return a closure to get updated timeout based on elapsed time. When
    /// the returned closure is called, it returns the remaining time until
    /// timeout. The timeout counts from the creation time of the closure.
    ///
    /// # Returned Closure Parameters
    /// - `timer`: A mutable reference to a [`Timer`] instance.
    ///
    /// # Returned Closure Returns
    /// - `Ok(u32)`: Remaining timeout in milliseconds.
    /// - [`Err(SessionError::Timeout)`](SessionError): The given timeout has
    ///   expired.
    fn get_timeout_update_func(
        &mut self,
        timeout_ms: u32,
    ) -> impl Fn(&mut T) -> Result<u32, SessionError<S::ReadError, S::WriteError>> {
        // Record the closure creation time.
        let create_time = self.link.get_timer().get_timestamp_ms();

        move |timer| {
            // Get the time of closure invocation.
            let cur_time = timer.get_timestamp_ms();
            // Calculate time elapsed after closure creation.
            let elapsed_time = cur_time - create_time;
            // Calculate remaining time.
            let remaining_time = timeout_ms.saturating_sub(elapsed_time);

            if remaining_time == 0 {
                Err(SessionError::Timeout)
            } else {
                Ok(remaining_time)
            }
        }
    }

    /// Start a new session as the sender. Advertise to the receiver the length
    /// of the data to be sent.
    ///
    /// # Parameters
    /// - `data_len`: The length of the data to be sent.
    /// - `timeout_ms`: The timeout in milliseconds.
    ///
    /// # Returns
    /// - `Ok(())`: Session successfully started. Can proceed to send data.
    /// - [`Err(SessionError)`](SessionError): An error occurred, which could
    ///   be a timeout.
    fn initiate_session(
        &mut self,
        data_len: usize,
        timeout_ms: u32,
    ) -> Result<(), SessionError<S::ReadError, S::WriteError>> {
        // The data length must be representable with `u16`.
        if data_len > u16::MAX as usize {
            return Err(SessionError::Oversize);
        }
        let data_len = data_len as u16;

        // Increase the session number whenever we start a new session.
        self.tx_session_num = self.tx_session_num.overflowing_add(1).0;

        let update_timeout = self.get_timeout_update_func(timeout_ms);

        // Send a request to the receiver and wait for sending clearance. Try
        // RRC times before failing.
        for _ in 0..RRC {
            // Send request packet.
            Packet::build_send_request(data_len, self.tx_session_num).send(&mut self.link)?;

            // Will return timeout error here if the timeout set by client has
            // expired.
            let remaining_timeout = update_timeout(self.link.get_timer())?;
            // Wait for the clearance packet. Should timeout and try again
            // after the duration of RTO but not the potentially very long
            // timeout configured by the client.
            match self
                .wait_for_clearance(remaining_timeout.min(RTO))
                .into_tolerable()
                .tolerate_timeout()
                .tolerate_clobber()
                .tolerate_out_of_sync()
                .tolerate_reset()?
            {
                // Got a clearance packet. We are cleared to send if the echoed
                // session number matches what we just sent. Otherwise just try
                // again.
                Some(session_num) => {
                    if session_num == self.tx_session_num {
                        return Ok(());
                    }
                }
                // Did not get a clearance packet but the error was one of the
                // tolarable errors, just try again.
                None => continue,
            }
        }

        // Not cleared to send after trying RRC times.
        Err(SessionError::NotClearToSend)
    }

    /// Wait for the clearance packet and return any encountered errors.
    ///
    /// # Parameters
    /// - `timeout_ms`: The timeout in milliseconds.
    ///
    /// # Returns
    /// - `Ok(u16)`: The session number echoed by the send clearance packet.
    /// - [`Err(SessionError)`](SessionError): An error occurred, which could
    ///   be a timeout.
    fn wait_for_clearance(
        &mut self,
        timeout_ms: u32,
    ) -> Result<u16, SessionError<S::ReadError, S::WriteError>> {
        let packet = Packet::receive(&mut self.link, None, timeout_ms, &mut self.scratchpad)?;

        // Examine the received packet content.
        match packet.get_content() {
            // Got the clearance we are expecting.
            PacketContent::SendClearance { session_num } => Ok(session_num),
            // We are trying to start a session as the sender. The
            // synchronization is broken if the other end also wants to start a
            // session as the sender or is actively sending data.
            PacketContent::SendRequest { .. } | PacketContent::Data { .. } => {
                // Send a reset packet so that the other end can notice the
                // problem as quickly as possible.
                Packet::build_reset().send(&mut self.link)?;
                Err(SessionError::OutOfSync)
            }
            // Got a reset packet. Just return the error.
            PacketContent::Reset => Err(SessionError::Reset),
        }
    }

    /// Send the given data to the receiver. Ensure all data is acknowledged by
    /// the receiver.
    ///
    /// # Parameters
    /// - `data`: The data to be sent.
    /// - `timeout_ms`: The timeout in milliseconds.
    ///
    /// # Returns
    /// - `Ok(())`: Data successfully sent and acknowledged.
    /// - [`Err(SessionError)`](SessionError): An error occurred, which could
    ///   be a timeout.
    fn send_data(
        &mut self,
        data: &[u8],
        timeout_ms: u32,
    ) -> Result<(), SessionError<S::ReadError, S::WriteError>> {
        let update_timeout = self.get_timeout_update_func(timeout_ms);

        // The sequence number starts from even and alternates between even
        // and odd.
        let mut sequence = Sequence::Even;

        // The data is sent in chunks. All but the last chunk are of maximum
        // payload size.
        for bytes in data.chunks(packet::MAX_DATA_PACKET_PAYLOAD_SIZE) {
            let mut acknowledged = false;

            // Send the data chunk and wait for acknowledge from the receiver.
            // Retry up to RRC times.
            for _ in 0..RRC {
                // Send the chunk.
                Packet::build_data(sequence, bytes).send(&mut self.link)?;

                // Wait for acknowledge. Should timeout and retransmit the
                // chunk after the duration of RTO but not the potentially
                // very long timeout configured by the client.
                let remaining_timeout = update_timeout(self.link.get_timer())?;
                match self
                    .wait_for_acknowledge(sequence, remaining_timeout.min(RTO))
                    .into_tolerable()
                    .tolerate_timeout()
                    .tolerate_clobber()?
                {
                    // Got acknowledged. Should proceed to the next chunk.
                    Some(Acknowledge::Ack) => {
                        acknowledged = true;
                        break;
                    }
                    // Got actively non-acknowledged. Retransmit the chunk.
                    Some(Acknowledge::Nack) => continue,
                    // Timed out or got cloberred acknowledge packet.
                    // Retransmit the chunk.
                    None => continue,
                }
            }

            // If we have tried RRC times to send the chunk but still not get
            // acknowledged, we abort.
            if !acknowledged {
                return Err(SessionError::NotAcknowledged);
            }

            // Alternate the sequence number before sending the next chunk.
            sequence = sequence.toggled();
        }

        Ok(())
    }

    /// Wait for the acknowledge packet and return any encountered errors.
    ///
    /// # Parameters
    /// - `sequence`: The expected sequence number in the acknowledge packet.
    /// - `timeout_ms`: The timeout in milliseconds.
    ///
    /// # Returns
    /// - [`Ok(Acknowledge)`](Acknowledge): Either `Ack` or `Nack` conveyed by
    ///   the acknowledge packet.
    /// - [`Err(SessionError)`](SessionError): An error occurred, which could
    ///   be a timeout.
    fn wait_for_acknowledge(
        &mut self,
        sequence: Sequence,
        timeout_ms: u32,
    ) -> Result<Acknowledge, SessionError<S::ReadError, S::WriteError>> {
        // Receive a packet.
        Packet::receive(&mut self.link, None, timeout_ms, &mut self.scratchpad)
            // Convert `PacketError` to `SessionError`.
            .map_err(Into::into)
            .and_then(|packet| {
                // If the packet contains expected sequence number, return the
                // acknowledge field.
                if packet.get_sequence() == sequence {
                    Ok(packet.get_acknowledge())
                // Otherwise, the sequence number in the acknowledge packet
                // corresponds to the previous data chunk, not the current data
                // chunk being sent. However, the sender must have already
                // received positive acknowledgement for the previous data
                // chunk before sending the current chunk. Somehow the receiver
                // did not keep up and is still trying to acknowledge the
                // previous chunk. Return an out-of-sync error.
                } else {
                    Err(SessionError::OutOfSync)
                }
            })
    }

    /// Terminate the session after all data has been sent. This is done by
    /// sending a reset packet. No acknowledgement is expected.
    ///
    /// # Returns
    /// - `Ok(())`: Reset packet successfully sent.
    /// - [`Err(SessionError)`](SessionError): An error occurred.
    fn finalize_session(&mut self) -> Result<(), SessionError<S::ReadError, S::WriteError>> {
        Packet::build_reset()
            .send(&mut self.link)
            .map_err(Into::into)
    }

    /// Send the clearance to the receiver. Then, wait for and acknowledge the
    /// first data packet which marks the beginning of the data transmission.
    ///
    /// # Parameters
    /// - `data`: The buffer to hold incoming data.
    /// - `timeout_ms`: The timeout in milliseconds.
    ///
    /// # Returns
    /// - `Ok(())`: Session started. First data packet received and
    ///   acknowledged.
    /// - [`Err(SessionError)`](SessionError): An error occurred, which could
    ///   be a timeout.
    fn accept_new_session(
        &mut self,
        data: &mut [u8],
        timeout_ms: u32,
    ) -> Result<(), SessionError<S::ReadError, S::WriteError>> {
        let update_timeout = self.get_timeout_update_func(timeout_ms);

        // Send the clearance packet to the sender.
        Packet::build_send_clearance(self.rx_session_num).send(&mut self.link)?;

        // Get the buffer for the first data packet.
        let buffer = data
            .chunks_mut(packet::MAX_DATA_PACKET_PAYLOAD_SIZE)
            .nth(0)
            .unwrap_or(&mut []);

        // Receive the first data packet. Retry up to RRC times.
        for _ in 0..RRC {
            let remaining_timeout = update_timeout(&mut self.link.get_timer())?;

            // Receive a packet with the buffer.
            let packet = match Packet::receive(
                &mut self.link,
                Some(buffer),
                remaining_timeout.min((RRC as u32) * RTO),
                &mut self.scratchpad,
            )
            // Convert `PacketError` to `SessionError`.
            .map_err(Into::into)
            // If the sender did not send anyting for the duration of RRC * RTO,
            // then the sender should already have timed out and will never
            // send data for the current session. Return a disconnection error
            // in this case. But just in case check if the timeout was
            // triggered by the client timeout setting, and if so just keep the
            // timeout error instead of turning it into a disconnection error.
            .map_err(|e| match e {
                SessionError::Timeout => {
                    if update_timeout(self.link.get_timer()).is_err() {
                        SessionError::Timeout
                    } else {
                        SessionError::Disconnected
                    }
                }
                e => e,
            })
            // Receiving a cloberred packet is not a fatal error.
            .into_tolerable()
            .tolerate_clobber()?
            {
                Some(packet) => packet,
                // The cloberred packet might be either the first data packet
                // or another send request packet if our clearance packet got
                // lost. Thus we do not know whether we should send a nack
                // packet or a reset packet. So just do nothing and let the
                // sender timeout and try again.
                None => continue,
            };

            // Examine the received packet content.
            match packet.get_content() {
                // Got a data packet as expected.
                PacketContent::Data { buffer } => match buffer {
                    // If the data packet contains data, reply an ack packet to
                    // the sender. Since this is the first data packet, the
                    // sequence number must be even.
                    Some(_) => {
                        Packet::build_ack(Sequence::Even).send(&mut self.link)?;
                        return Ok(());
                    }
                    // If the data packet contains no data, then it is an ack
                    // or nack packet. It is not expected to receive such a
                    // packet as the receiver. The other end must have
                    // something going wrong.
                    None => return Err(SessionError::OutOfSync),
                },
                // Got a send request packet again. It is likely because our
                // send clearance packet got lost or cloberred.
                PacketContent::SendRequest {
                    data_len,
                    session_num,
                } => {
                    // If the new request packet contains identical information
                    // as in the previous one, it is a retransmission. In this
                    // case we send the clearance again.
                    if session_num == self.rx_session_num && data_len == self.rx_data_len {
                        Packet::build_send_clearance(self.rx_session_num).send(&mut self.link)?;
                        continue;
                    // Otherwise the sender is trying to start a new session.
                    // Return that we are not in synchronization anymore.
                    } else {
                        Packet::build_reset().send(&mut self.link)?;
                        return Err(SessionError::OutOfSync);
                    }
                }
                // Got an unexpected send clearance packet. We are the receiver
                // and we should be the one giving out clearance. The other
                // side must have something going wrong. Return the error.
                PacketContent::SendClearance { .. } => return Err(SessionError::OutOfSync),
                // The sender wants to abort the session. Return the error.
                PacketContent::Reset => return Err(SessionError::Reset),
            }
        }

        // Did not receive the well-formed first data packet after trying RRC
        // times.
        Err(SessionError::Clobbered)
    }

    /// Wait to receive all the remaining data chunks. Acknowledge each
    /// received data packet.
    ///
    /// # Parameters
    /// - `data`: The buffer to hold incoming data.
    /// - `timeout_ms`: The timeout in milliseconds.
    ///
    /// # Returns
    /// - [`Ok(Sequence)`](Sequence): The sequence number of the last received
    ///   data packet. All data chunks have been received and acknowledged.
    /// - [`Err(SessionError)`](SessionError): An error occurred, which could
    ///   be a timeout.
    fn accept_remaining_data(
        &mut self,
        data: &mut [u8],
        timeout_ms: u32,
    ) -> Result<Sequence, SessionError<S::ReadError, S::WriteError>> {
        let update_timeout = self.get_timeout_update_func(timeout_ms);

        // Since we have got the first data chunk, the sequence number then
        // starts with odd.
        let mut expect_seq = Sequence::Odd;

        // The iteration for the buffer chunks should also skip the first one.
        let mut buffer_iter = data
            .chunks_mut(packet::MAX_DATA_PACKET_PAYLOAD_SIZE)
            .skip(1);

        // Tracking the current buffer chunk being written to.
        let mut cur_buffer = buffer_iter.next();

        // Continue until we fill up all the buffer chunks.
        while let Some(buffer) = cur_buffer.as_mut() {
            let mut acknowledged = false;

            // Retry up to RRC times for each buffer chunk.
            for _ in 0..RRC {
                let remaining_timeout = update_timeout(self.link.get_timer())?;

                // Receive a packet with the current buffer chunk.
                let packet = match Packet::receive(
                    &mut self.link,
                    Some(*buffer),
                    remaining_timeout.min((RRC as u32) * RTO),
                    &mut self.scratchpad,
                )
                // Convert `PacketError` to `SessionError`.
                .map_err(Into::into)
                // If the sender did not send anyting for the duration of
                // RRC * RTO, then the sender should already have timed out
                // and will never send data for the current session. Return
                // a disconnection error in this case. But just in case check
                // if the timeout was triggered by the client timeout setting,
                // and if so just keep the timeout error instead of turning it
                // into a disconnection error.
                .map_err(|e| match e {
                    SessionError::Timeout => {
                        if update_timeout(self.link.get_timer()).is_err() {
                            SessionError::Timeout
                        } else {
                            SessionError::Disconnected
                        }
                    }
                    e => e,
                })
                // Receiving a cloberred packet is not a fatal error.
                .into_tolerable()
                .tolerate_clobber()?
                {
                    Some(packet) => packet,
                    // Reply a nack packet if the received packet is cloberred
                    // and retry.
                    None => {
                        Packet::build_nack(expect_seq).send(&mut self.link)?;
                        continue;
                    }
                };

                // Examine the packet content.
                match packet.get_content() {
                    // Got a data packet as expected.
                    PacketContent::Data { buffer } => match buffer {
                        // The data packet contains data as expected. The data
                        // has been written to the buffer.
                        Some(_) => {
                            // If the sequence number is also correct, then the
                            // received packet is the new data chunk. Reply an
                            // ack packet and then proceed to receiving the
                            // next chunk.
                            if packet.get_sequence() == expect_seq {
                                Packet::build_ack(expect_seq).send(&mut self.link)?;
                                acknowledged = true;
                                break;
                            // Otherwise, the packet is a retransmitted one of
                            // an already received chunk, probably because the
                            // previous ack packet got lost. We send again the
                            // ack for the *previous* data chunk.
                            } else {
                                Packet::build_ack(expect_seq.toggled()).send(&mut self.link)?;
                                continue;
                            }
                        }
                        // If the data packet contains no data, then it is an
                        // ack or nack packet. It is not expected to receive
                        // such a packet as the receiver. The other end must
                        // have something going wrong.
                        None => {
                            return Err(SessionError::OutOfSync);
                        }
                    },
                    // Got an unexpected send request or send clearance packet.
                    // We are in the middle of receiving data and should only
                    // receive data packets. The other side must have something
                    // going wrong. Reply a reset packet so that the other end
                    // can discover the problem as quickly as possible. Also
                    // return the error.
                    PacketContent::SendRequest { .. } | PacketContent::SendClearance { .. } => {
                        Packet::build_reset().send(&mut self.link)?;
                        return Err(SessionError::OutOfSync);
                    }
                    // The sender wants to abort the session. Return the error.
                    PacketContent::Reset => return Err(SessionError::Reset),
                }
            }

            // Did not receive a well-formed data packet after trying RRC times.
            if !acknowledged {
                return Err(SessionError::Clobbered);
            }

            // Alternate the sequence number for the next chunk.
            expect_seq = expect_seq.toggled();

            // Advance to the next buffer chunk.
            cur_buffer = buffer_iter.next();
        }

        // All data has been received. Return the final received sequence
        // number.
        Ok(expect_seq.toggled())
    }

    /// Wait for the final reset packet to terminate the session.
    ///
    /// # Parameters
    /// - `final_seq`: The sequence number of the last data packet received.
    /// - `timeout_ms`: The timeout in milliseconds.
    ///
    /// # Returns
    /// - `Ok(())`: Session successfully terminated.
    /// - [`Err(SessionError)`](SessionError): An error occurred.
    fn accept_session_termination(
        &mut self,
        final_seq: Sequence,
        timeout_ms: u32,
    ) -> Result<(), SessionError<S::ReadError, S::WriteError>> {
        let update_timeout = self.get_timeout_update_func(timeout_ms);

        // Retry up to RRC time to get the final reset packet.
        for _ in 0..RRC {
            let remaining_timeout = update_timeout(self.link.get_timer())?;

            // Receive a packet.
            let packet: Packet = match Packet::receive(
                &mut self.link,
                None,
                remaining_timeout.min((RRC as u32) * RTO),
                &mut self.scratchpad,
            ) {
                Ok(packet) => packet,
                // The cloberred packet might be the expected reset packet or
                // the retransmission of the last data chunk if our ack packet
                // got lost. We do not know which one is the actual case, so
                // we simply try again without replying anything and resort to
                // the timeout mechanism.
                Err(PacketError::Clobbered) => continue,
                // We received a data packet containing client data. This is
                // likely to be the retransmission of the last data chunk
                // probably because our ack packet got lost. Send the ack again
                // and retry.
                Err(PacketError::NoBuffer) => {
                    Packet::build_ack(final_seq).send(&mut self.link)?;
                    continue;
                }
                // We timed out either because the client configured timeout
                // has expired or the sender did not send anything for the
                // duration of RRC * RTO. Since we already got all of the data,
                // just return `Ok(())`.
                Err(PacketError::Timeout) => {
                    return Ok(());
                }
                // Propagate other errors.
                Err(e) => return Err(e.into()),
            };

            match packet.get_content() {
                // Got the expected reset packet.
                PacketContent::Reset => return Ok(()),
                // Got an unexpected send request. The reset packet might get
                // lost and the other side now wants to start a new session as
                // the sender. We simply return `Ok(())` and let the other side
                // timeout and try again.
                PacketContent::SendRequest { .. } => return Ok(()),
                // Got a send clearance packet but we did not send any request.
                // The other side must have something going wrong. Send a reset
                // packet so that the other side can discover the problem as
                // quickly as possible. But since we successfully received all
                // data, we still return `Ok(())`.
                PacketContent::SendClearance { .. } => {
                    Packet::build_reset().send(&mut self.link)?;
                    return Ok(());
                }
                // Got a data packet. Since we did not provide buffer to
                // receive the packet, the packet must be an ack or nack
                // packet, otherwise we would have got a `NoBuffer` error.
                // However, we did not send any data packet and are not
                // expecting an ack or nack. The other side must have something
                // going wrong. Send a reset packet so that the other side can
                // discover the problem as quickly as possible. But since we
                // successfully received all data, we still return `Ok(())`.
                PacketContent::Data { .. } => {
                    Packet::build_reset().send(&mut self.link)?;
                    return Ok(());
                }
            }
        }

        // Did not receive a well-formed reset packet after trying RRC times.
        // However, since we successfully received all data, we still return
        // `Ok(())`.
        Ok(())
    }
}

/// Unit tests for the [`session`](crate::session) module.
#[cfg(test)]
mod test {
    use super::*;
    use crate::link::tests::{DummyExpiringTimer, LoopbackSerial};
    use crate::tests::{MockSerial, MockTimer};
    use crossbeam::channel;
    use std::sync::{Arc, Mutex};
    use std::thread;

    /// Send "Hello, World" and receive "Hello, World".
    #[test]
    fn hello_world() {
        let (send0, recv1) = channel::unbounded();
        let (send1, recv0) = channel::unbounded();

        let serial0 = MockSerial::new(send0, recv0);
        let serial1 = MockSerial::new(send1, recv1);

        let timer0 = MockTimer::new();
        let timer1 = MockTimer::new();

        let trans0 = Arc::new(Mutex::new(Session::new(serial0, timer0)));
        let trans1 = Arc::new(Mutex::new(Session::new(serial1, timer1)));

        let _trans0_cloned = Arc::clone(&trans0);
        let _trans1_cloned = Arc::clone(&trans1);

        fn client(trans: &mut Session<MockSerial, MockTimer>) {
            let msg = b"Hello, World!";
            trans.send(msg, 1000).unwrap();
        }

        fn server(trans: &mut Session<MockSerial, MockTimer>) {
            let len = trans.listen(1000).unwrap();
            let mut buf = vec![0u8; len as usize].into_boxed_slice();
            trans.receive(&mut buf, 10000).unwrap();

            assert_eq!(buf.as_ref(), b"Hello, World!");
        }

        let t0 = thread::spawn(move || client(&mut *trans0.lock().unwrap()));
        let t1 = thread::spawn(move || server(&mut *trans1.lock().unwrap()));

        t0.join().unwrap();
        t1.join().unwrap();
    }

    /// Trigger [`SessionError::Timeout`].
    #[test]
    fn timeout() {
        let (send0, recv1) = channel::unbounded();
        let (send1, recv0) = channel::unbounded();

        let serial0 = MockSerial::new(send0, recv0);
        let serial1 = MockSerial::new(send1, recv1);

        let timer0 = DummyExpiringTimer::new();
        let timer1 = DummyExpiringTimer::new();

        let trans0 = Arc::new(Mutex::new(Session::new(serial0, timer0)));
        let trans1 = Arc::new(Mutex::new(Session::new(serial1, timer1)));

        let _trans0_cloned = Arc::clone(&trans0);
        let _trans1_cloned = Arc::clone(&trans1);

        fn client(trans: &mut Session<MockSerial, DummyExpiringTimer>) {
            let msg = b"Hello, World!";
            let res = trans.send(msg, 1000);
            match res {
                Err(SessionError::Timeout) => (),
                _ => panic!("Expected Timeout error, Found {:?}", res),
            }
        }

        fn server(trans: &mut Session<MockSerial, DummyExpiringTimer>) {
            let res = trans.listen(1000);
            match res {
                Err(SessionError::Timeout) => (),
                _ => panic!("Expected Timeout error, Found {:?}", res),
            }
        }

        let t0 = thread::spawn(move || client(&mut *trans0.lock().unwrap()));
        let t1 = thread::spawn(move || server(&mut *trans1.lock().unwrap()));

        t0.join().unwrap();
        t1.join().unwrap();
    }

    /// Trigger [`SessionError::NotClearToSend`].
    #[test]
    fn not_clear_to_send() {
        let serial = LoopbackSerial::new();
        let timer = MockTimer::new();
        let mut trans: Session<LoopbackSerial, MockTimer, 50, 3> = Session::new(serial, timer);

        let msg = b"NotClearToSend";
        let res = trans.send(msg, 1000);
        match res {
            Err(SessionError::NotClearToSend) => (),
            _ => panic!("Expected Timeout error, Found {:?}", res),
        }
    }

    /// Trigger [`SessionError::Oversize`].
    #[test]
    fn oversize() {
        let serial = LoopbackSerial::new();
        let timer = MockTimer::new();
        let mut trans: Session<LoopbackSerial, MockTimer, 50, 3> = Session::new(serial, timer);

        let msg = vec![0u8; (u16::MAX as usize) + 1];
        let res = trans.send(&msg, 1000);
        match res {
            Err(SessionError::Oversize) => (),
            _ => panic!("Expected Oversize error, Found {:?}", res),
        }
    }

    /// Trigger [`SessionError::BufferSizeMismatch`].
    #[test]
    fn buffer_size_mismatch() {
        let (send0, recv1) = channel::unbounded();
        let (send1, recv0) = channel::unbounded();

        let serial0 = MockSerial::new(send0, recv0);
        let serial1 = MockSerial::new(send1, recv1);

        let timer0 = MockTimer::new();
        let timer1 = MockTimer::new();

        let trans0 = Arc::new(Mutex::new(Session::new(serial0, timer0)));
        let trans1 = Arc::new(Mutex::new(Session::new(serial1, timer1)));

        let _trans0_cloned = Arc::clone(&trans0);
        let _trans1_cloned = Arc::clone(&trans1);

        fn client(trans: &mut Session<MockSerial, MockTimer>) {
            let msg = b"Hello, World!";
            trans.send(msg, 1000).unwrap_or_default();
        }

        fn server(trans: &mut Session<MockSerial, MockTimer>) {
            let len = trans.listen(1000).unwrap();
            let mut buf = vec![0u8; (len as usize) + 1].into_boxed_slice();
            let res = trans.receive(&mut buf, 1000);
            match res {
                Err(SessionError::BufferSizeMismatch) => (),
                _ => panic!("Expected BufferSizeMismatch error, Found {:?}", res),
            }
        }

        let t0 = thread::spawn(move || client(&mut *trans0.lock().unwrap()));
        let t1 = thread::spawn(move || server(&mut *trans1.lock().unwrap()));

        t0.join().unwrap();
        t1.join().unwrap();
    }

    /// Trigger [`SessionError::NotAcknowledged`].
    #[test]
    fn not_acknowledged() {
        let (send0, recv1) = channel::unbounded();
        let (send1, recv0) = channel::unbounded();

        let serial0 = MockSerial::new(send0, recv0);
        let serial1 = MockSerial::new(send1, recv1);

        let timer0 = MockTimer::new();
        let timer1 = MockTimer::new();

        let trans0 = Arc::new(Mutex::new(Session::new(serial0, timer0)));
        let trans1 = Arc::new(Mutex::new(Session::new(serial1, timer1)));

        let _trans0_cloned = Arc::clone(&trans0);
        let _trans1_cloned = Arc::clone(&trans1);

        fn client(trans: &mut Session<MockSerial, MockTimer>) {
            let msg = b"Hello, World!";
            let res = trans.send(msg, 1000);
            match res {
                Err(SessionError::NotAcknowledged) => (),
                _ => panic!("Expected NotAcknowledged error, Found {:?}", res),
            }
        }

        fn server(trans: &mut Session<MockSerial, MockTimer>) {
            trans.listen(1000).unwrap();
            Packet::build_send_clearance(trans.rx_session_num)
                .send(&mut trans.link)
                .unwrap();
            Packet::build_nack(Sequence::Even)
                .send(&mut trans.link)
                .unwrap();
            Packet::build_nack(Sequence::Even)
                .send(&mut trans.link)
                .unwrap();
            Packet::build_nack(Sequence::Even)
                .send(&mut trans.link)
                .unwrap();
        }

        let t0 = thread::spawn(move || client(&mut *trans0.lock().unwrap()));
        let t1 = thread::spawn(move || server(&mut *trans1.lock().unwrap()));

        t0.join().unwrap();
        t1.join().unwrap();
    }

    /// Trigger [`SessionError::OutOfSync`].
    #[test]
    fn out_of_sync() {
        let (send0, recv1) = channel::unbounded();
        let (send1, recv0) = channel::unbounded();

        let serial0 = MockSerial::new(send0, recv0);
        let serial1 = MockSerial::new(send1, recv1);

        let timer0 = MockTimer::new();
        let timer1 = MockTimer::new();

        let trans0 = Arc::new(Mutex::new(Session::new(serial0, timer0)));
        let trans1 = Arc::new(Mutex::new(Session::new(serial1, timer1)));

        let _trans0_cloned = Arc::clone(&trans0);
        let _trans1_cloned = Arc::clone(&trans1);

        fn client(trans: &mut Session<MockSerial, MockTimer>) {
            Packet::build_send_request(10, trans.rx_session_num)
                .send(&mut trans.link)
                .unwrap();
            Packet::build_send_request(10, trans.rx_session_num + 1)
                .send(&mut trans.link)
                .unwrap();
        }

        fn server(trans: &mut Session<MockSerial, MockTimer>) {
            let len = trans.listen(1000).unwrap();
            let mut buf = vec![0u8; len as usize].into_boxed_slice();
            let res = trans.receive(&mut buf, 1000);
            match res {
                Err(SessionError::OutOfSync) => (),
                _ => panic!("Expected OutOfSync error, Found {:?}", res),
            }
        }

        let t0 = thread::spawn(move || client(&mut *trans0.lock().unwrap()));
        let t1 = thread::spawn(move || server(&mut *trans1.lock().unwrap()));

        t0.join().unwrap();
        t1.join().unwrap();
    }

    /// Trigger [`SessionError::Reset`].
    #[test]
    fn reset() {
        let (send0, recv1) = channel::unbounded();
        let (send1, recv0) = channel::unbounded();

        let serial0 = MockSerial::new(send0, recv0);
        let serial1 = MockSerial::new(send1, recv1);

        let timer0 = MockTimer::new();
        let timer1 = MockTimer::new();

        let trans0 = Arc::new(Mutex::new(Session::new(serial0, timer0)));
        let trans1 = Arc::new(Mutex::new(Session::new(serial1, timer1)));

        let _trans0_cloned = Arc::clone(&trans0);
        let _trans1_cloned = Arc::clone(&trans1);

        fn client(trans: &mut Session<MockSerial, MockTimer>) {
            Packet::build_send_request(10, trans.rx_session_num)
                .send(&mut trans.link)
                .unwrap();
            Packet::build_reset().send(&mut trans.link).unwrap();
        }

        fn server(trans: &mut Session<MockSerial, MockTimer>) {
            let len = trans.listen(1000).unwrap();
            let mut buf = vec![0u8; len as usize].into_boxed_slice();
            let res = trans.receive(&mut buf, 1000);
            match res {
                Err(SessionError::Reset) => (),
                _ => panic!("Expected Reset error, Found {:?}", res),
            }
        }

        let t0 = thread::spawn(move || client(&mut *trans0.lock().unwrap()));
        let t1 = thread::spawn(move || server(&mut *trans1.lock().unwrap()));

        t0.join().unwrap();
        t1.join().unwrap();
    }

    /// Send data with length from 1 to 256, with a known data pattern. The
    /// receiver should correctly receive the data with any length from 1 to 256.
    #[test]
    fn data_length_1_to_256() {
        let (send0, recv1) = channel::unbounded();
        let (send1, recv0) = channel::unbounded();

        let serial0 = MockSerial::new(send0, recv0);
        let serial1 = MockSerial::new(send1, recv1);

        let timer0 = MockTimer::new();
        let timer1 = MockTimer::new();

        let trans0 = Arc::new(Mutex::new(Session::new(serial0, timer0)));
        let trans1 = Arc::new(Mutex::new(Session::new(serial1, timer1)));

        fn client(trans: &mut Session<MockSerial, MockTimer>, msg: &[u8]) {
            trans.send(msg, 1000).unwrap();
        }

        fn server(trans: &mut Session<MockSerial, MockTimer>, expected_buf: &[u8]) {
            let len = trans.listen(1000).unwrap();
            let mut buf = vec![0u8; len as usize].into_boxed_slice();
            trans.receive(&mut buf, 1000).unwrap();
            assert_eq!(buf.as_ref(), expected_buf);
        }

        for len in 0..=255 {
            let trans0_cloned = Arc::clone(&trans0);
            let trans1_cloned = Arc::clone(&trans1);

            let msg0: Vec<u8> = (0..=len).collect();
            let msg1 = msg0.clone();

            let t0 =
                thread::spawn(move || client(&mut *trans0_cloned.lock().unwrap(), msg0.as_ref()));
            let t1 =
                thread::spawn(move || server(&mut *trans1_cloned.lock().unwrap(), msg1.as_ref()));

            t0.join().unwrap();
            t1.join().unwrap();
        }
    }

    /// Letting the sender and receiver change their role.
    #[test]
    fn role_reversal() {
        let (send0, recv1) = channel::unbounded();
        let (send1, recv0) = channel::unbounded();

        let serial0 = MockSerial::new(send0, recv0);
        let serial1 = MockSerial::new(send1, recv1);

        let timer0 = MockTimer::new();
        let timer1 = MockTimer::new();

        let trans0 = Arc::new(Mutex::new(Session::new(serial0, timer0)));
        let trans1 = Arc::new(Mutex::new(Session::new(serial1, timer1)));

        let _trans0_cloned = Arc::clone(&trans0);
        let _trans1_cloned = Arc::clone(&trans1);

        fn client(trans: &mut Session<MockSerial, MockTimer>) {
            let msg = b"Hello, Receiver!";
            trans.send(msg, 1000).unwrap();

            let len = trans.listen(1000).unwrap();
            let mut buf = vec![0u8; len as usize].into_boxed_slice();
            trans.receive(&mut buf, 10000).unwrap();

            assert_eq!(buf.as_ref(), b"Hello, Sender!");
        }

        fn server(trans: &mut Session<MockSerial, MockTimer>) {
            let len = trans.listen(1000).unwrap();
            let mut buf = vec![0u8; len as usize].into_boxed_slice();
            trans.receive(&mut buf, 10000).unwrap();

            assert_eq!(buf.as_ref(), b"Hello, Receiver!");

            let msg = b"Hello, Sender!";
            trans.send(msg, 1000).unwrap();
        }

        let t0 = thread::spawn(move || client(&mut *trans0.lock().unwrap()));
        let t1 = thread::spawn(move || server(&mut *trans1.lock().unwrap()));

        t0.join().unwrap();
        t1.join().unwrap();
    }

    /// Letting the sender send multiple times.
    #[test]
    fn send_multiple_times() {
        let (send0, recv1) = channel::unbounded();
        let (send1, recv0) = channel::unbounded();

        let serial0 = MockSerial::new(send0, recv0);
        let serial1 = MockSerial::new(send1, recv1);

        let timer0 = MockTimer::new();
        let timer1 = MockTimer::new();

        let trans0 = Arc::new(Mutex::new(Session::new(serial0, timer0)));
        let trans1 = Arc::new(Mutex::new(Session::new(serial1, timer1)));

        let _trans0_cloned = Arc::clone(&trans0);
        let _trans1_cloned = Arc::clone(&trans1);

        fn client(trans: &mut Session<MockSerial, MockTimer>) {
            let msg = b"Hello, First Message!";
            trans.send(msg, 1000).unwrap();

            let msg = b"Hello, Second Message!";
            trans.send(msg, 1000).unwrap();
        }

        fn server(trans: &mut Session<MockSerial, MockTimer>) {
            let len = trans.listen(1000).unwrap();
            let mut buf = vec![0u8; len as usize].into_boxed_slice();
            trans.receive(&mut buf, 10000).unwrap();

            assert_eq!(buf.as_ref(), b"Hello, First Message!");

            let len = trans.listen(1000).unwrap();
            let mut buf = vec![0u8; len as usize].into_boxed_slice();
            trans.receive(&mut buf, 10000).unwrap();

            assert_eq!(buf.as_ref(), b"Hello, Second Message!");
        }

        let t0 = thread::spawn(move || client(&mut *trans0.lock().unwrap()));
        let t1 = thread::spawn(move || server(&mut *trans1.lock().unwrap()));

        t0.join().unwrap();
        t1.join().unwrap();
    }
}
