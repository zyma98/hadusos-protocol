//! A half-duplex session over serial protocol.
//!
//! The client should provide a serial device instance and a timer instance,
//! which respectively implements the [`Serial`] and [`Timer`] trait. Then, a
//! session layer instance can be constructed.
//!
//! The [`Session`] supports half-duplex data transmission, i.e., a client
//! operates as either the sender or the receiver at a time. When operating as
//! the sender, the client should call [`send`](Session::send) to send data.
//! When operating as the receiver, the client should call
//! [`listen`](Session::listen) to learn the incoming data length to allocate
//! a buffer, and subsequently call [`receive`](Session::receive) to receive
//! the data. The receiver may also call [`reject`](Session::reject) if it does
//! not want to proceed to receiving the data after [`listen`](Session::listen).
//!
//! Below shows an example.
//!
//! ```rust
//! use crossbeam::channel::{self, Receiver, RecvError, SendError, Sender};
//! use hadusos::{Serial, SerialError, Session, Timer};
//! use std::{
//!     sync::Arc,
//!     thread,
//!     time::{Duration, SystemTime, UNIX_EPOCH},
//! };
//!
//! fn main() {
//!     // Create a pair of connected serial devices.
//!     let (serial0, serial1) = MockSerial::new_pair();
//!
//!     // Create two timers.
//!     let (timer0, timer1) = (MockTimer, MockTimer);
//!
//!     const TIMEOUT: u32 = 1000;
//!
//!     // Start a thread what will echo back whatever it receives.
//!     let thread_echo = thread::spawn(|| {
//!         // Create a session layer instance using the serial device and timer.
//!         let mut sess = Session::<_, _, 50, 3>::new(serial0, timer0);
//!
//!         // Listen for incoming data.
//!         let data_len = sess.listen(TIMEOUT).unwrap();
//!
//!         // Allocate a buffer and receive incoming data.
//!         let mut buffer = vec![0u8; data_len as usize].into_boxed_slice();
//!         sess.receive(&mut buffer, TIMEOUT).unwrap();
//!
//!         // Echo the data back.
//!         sess.send(&buffer, TIMEOUT).unwrap();
//!     });
//!
//!     // Start a thread that will send "hello world!" over the serial and verify
//!     // the echoed data.
//!     let thread_check = thread::spawn(|| {
//!         // Create a session layer instance using the serial device and timer.
//!         let mut sess = Session::<_, _, 50, 3>::new(serial1, timer1);
//!
//!         // Send "hello world!".
//!         sess.send("hello world!".as_bytes(), TIMEOUT).unwrap();
//!
//!         // Listen for echoed data.
//!         let data_len = sess.listen(TIMEOUT).unwrap();
//!
//!         // Allocate a buffer and receive echoed data.
//!         let mut buffer = vec![0u8; data_len as usize].into_boxed_slice();
//!         sess.receive(&mut buffer, TIMEOUT).unwrap();
//!
//!         // Verify echoed data.
//!         assert!(buffer.as_ref() == "hello world!".as_bytes());
//!     });
//!
//!     thread_echo.join().unwrap();
//!     thread_check.join().unwrap();
//! }
//!
//! /// Simulated timer.
//! struct MockTimer;
//!
//! impl Timer for MockTimer {
//!     /// Get the timestamp from the std library.
//!     fn get_timestamp_ms(&mut self) -> u32 {
//!         SystemTime::now()
//!             .duration_since(UNIX_EPOCH)
//!             .unwrap()
//!             .as_millis() as u32
//!     }
//! }
//!
//! /// Simulated serial device. It is simulated with `crossbeam`'s MPMC queues.
//! struct MockSerial {
//!     send: Arc<Sender<u8>>,
//!     recv: Arc<Receiver<u8>>,
//! }
//!
//! impl MockSerial {
//!     /// Get a pair of connected serial devices.
//!     fn new_pair() -> (Self, Self) {
//!         // Create channels to send in both directions.
//!         let (send0, recv0) = channel::unbounded();
//!         let (send1, recv1) = channel::unbounded();
//!
//!         // Wrap the endpoints in `Arc`s.
//!         let send0 = Arc::new(send0);
//!         let send1 = Arc::new(send1);
//!         let recv0 = Arc::new(recv0);
//!         let recv1 = Arc::new(recv1);
//!
//!         // Deliberately keep the reference counts to be always positive, so
//!         // that the channels will always be kept alive, otherwise we will get
//!         // channel disconnection errors.
//!         std::mem::forget(Arc::clone(&send0));
//!         std::mem::forget(Arc::clone(&send1));
//!         std::mem::forget(Arc::clone(&recv0));
//!         std::mem::forget(Arc::clone(&recv1));
//!
//!         (
//!             Self {
//!                 send: send0,
//!                 recv: recv1,
//!             },
//!             Self {
//!                 send: send1,
//!                 recv: recv0,
//!             },
//!         )
//!     }
//! }
//!
//! impl Serial for MockSerial {
//!     type ReadError = RecvError;
//!     type WriteError = SendError<u8>;
//!
//!     /// Read a byte from the channel with a timeout.
//!     fn read_byte_with_timeout(
//!         &mut self,
//!         timeout_ms: u32,
//!     ) -> Result<u8, SerialError<Self::ReadError, Self::WriteError>> {
//!         // Read byte from the channel. MUST map the timeout error to the
//!         // specific `SerialError` variant because the protocol stack treats
//!         // timeout error in a special way as it is sometimes recoverable.
//!         self.recv
//!             .recv_timeout(Duration::from_millis(timeout_ms as u64))
//!             .map_err(|e| match e {
//!                 channel::RecvTimeoutError::Timeout => SerialError::Timeout,
//!                 channel::RecvTimeoutError::Disconnected => SerialError::ReadError(RecvError),
//!             })
//!     }
//!
//!     /// Write a byte to the channel.
//!     fn write_byte(
//!         &mut self,
//!         byte: u8,
//!     ) -> Result<(), SerialError<Self::ReadError, Self::WriteError>> {
//!         self.send.send(byte).map_err(|e| SerialError::WriteError(e))
//!     }
//! }
//! ```

#![cfg_attr(not(test), no_std)]

mod link;
mod packet;
mod serial;
mod session;
mod timer;

#[cfg(test)]
mod tests;

pub use serial::{Serial, SerialError};
pub use session::{Session, SessionError};
pub use timer::Timer;
