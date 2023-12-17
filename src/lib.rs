//! A small crate made to help parse SBUS packages from any source.
//!
//! SBUS is a common standard used with radio transmitters and receivers in the RC hobby.
//! SBUS is transmitted via UART at a baud rate of 100000 with 8 bits of data, an even parity bit,
//! and 2 stop bits. Each UART frame contains 1 byte of SBUS
//! data. 25 frames comprise a packet, which has the following bytes:
//! - 1 header byte with a value of `0x0F`
//! - 22 bytes that store 16 channels of 11 bit information
//! - 1 special flag byte that contains the values of the digital channels, whether the frame has been lost, and whether the failsafe has been engaged
//! - 1 footer byte with a value of 0x00
//!
//! The special flag byte has the following arrangement:
//! [0 0 0 0 failsafe frame_lost channel17 channel18]
//!
//! All of this data is transmitted with the least significant bit (LSB) first, hence the need to
//! write this library.
use std::fmt;

/// A struct containing all the data for one SBUS packet
///
/// It contains a vector of 16 11-bit analog data channels and 2 digital data channels.
#[derive(Debug)]
pub struct SbusPacket {
    pub analog_channels: Vec<u16>,
    pub channel_17: bool,
    pub channel_18: bool,
}

impl SbusPacket {
    /// Tries to create a new `SbusPacket` from an array of 25 bytes.
    ///
    /// It will return an `Err` value if:
    /// - The array is not an SBUS packet
    /// - The frame has been lost (so as to not use invalid data)
    /// - The failsafe is engaged (meaning that no valid frames have been received for some time)
    ///
    /// # Usage
    /// ```
    /// use parse_sbus::{SbusPacket, ErrorKind};
    ///
    /// let data = [
    ///    0x0F, 128, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    /// ];
    ///
    /// let mut throttle = 50;
    ///
    /// let packet = SbusPacket::try_from_bytes(&data);
    ///
    /// if packet.is_err() {
    ///     match packet.unwrap_err().kind() {
    ///         ErrorKind::NotSbus => println!("Invalid packet, ignoring"),
    ///         ErrorKind::FrameLost => println!("Frame lost, keeping last value"),
    ///         ErrorKind::FailsafeEngaged => {
    ///             println!("Failsafe engaged, cutting throttle");
    ///             throttle = 0;
    ///         }
    ///     }
    /// } else {
    ///     throttle = packet.unwrap().analog_channels[0];
    /// }
    ///
    /// assert_eq!(throttle, 1025u16)
    /// ```
    pub fn try_from_bytes(bytes: &[u8; 25]) -> Result<SbusPacket, ParsingError> {
        // Checks to see if the first byte is the SBUS start packet byte
        if bytes[0] != 0x0F {
            return Err(ParsingError::new(ErrorKind::NotSbus));
        }
        // The 24th byte contains the following bits: [0 0 0 0 failsafe frame_lost ch18 ch17]
        // Received in LSB first order. This code checks to see if the failsafe is engaged or if
        // the frame has been lost.
        if (bytes[23] & 16) > 0 {
            return Err(ParsingError::new(ErrorKind::FailsafeEngaged));
        }
        if (bytes[23] & 32) > 0 {
            return Err(ParsingError::new(ErrorKind::FrameLost));
        }
        // blazingly fast but exceedingly ugly
        let mut bit_counter = 7;
        let mut output = Vec::<u16>::new();
        let mut current = 0u16;
        for i in 0..176 {
            // i/8+1 gives the correct index of the bytes parameter to use.
            // Since i and 8 are both integers, this simply finds the quotient minus remainder, which is an index.
            //The remainder is the actual bit we are looking at
            // i mod 11 gives the number of bit for each 11 bit value, resetting each time a full
            // value is completed.
            // Bit counter is the index of the bit in the actual byte being examined
            current |=
                (((bytes[(i / 8) + 1] & 2u8.pow(bit_counter)) >> bit_counter) as u16) << (i % 11);
            if i % 11 == 10 {
                output.push(current);
                current = 0u16;
            }
            if bit_counter == 0 {
                bit_counter = 7;
            } else {
                bit_counter -= 1;
            }
        }

        let channel_17 = (bytes[23] & 128) > 0;
        let channel_18 = (bytes[23] & 64) > 0;

        Ok(SbusPacket {
            analog_channels: output,
            channel_17,
            channel_18,
        })
    }
}

/// The error type returned by `try_from_bytes`
pub struct ParsingError {
    kind: ErrorKind,
}

impl ParsingError {
    /// Returns the `ErrorKind` value of this particular error
    pub fn kind(&self) -> ErrorKind {
        self.kind.to_owned()
    }
}

impl fmt::Debug for ParsingError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let error = match self.kind {
            ErrorKind::NotSbus => "Received non-SBUS packet",
            ErrorKind::FailsafeEngaged => "The failsafe has been engaged, this packet is unusable",
            ErrorKind::FrameLost => "A frame has been lost, this packet is unusable",
        };
        write!(f, "{}", error)
    }
}

impl ParsingError {
    fn new(kind: ErrorKind) -> ParsingError {
        ParsingError { kind }
    }
}

/// The different kinds of errors that result from `try_from_bytes`
#[derive(Clone)]
pub enum ErrorKind {
    NotSbus,
    FailsafeEngaged,
    FrameLost,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn analog() {
        let data = [
            0x0F, 128, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        ];
        let parsed = SbusPacket::try_from_bytes(&data).unwrap();
        let correct_analog: [u16; 16] = [1025, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1024];
        let mut works = true;
        for i in 0..15 {
            if parsed.analog_channels[i] != correct_analog[i] {
                works = false;
                break;
            }
        }
        assert!(works);
    }
    #[test]
    fn rejects_non_sbus() {
        let data = [
            0x0A, 128, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        ];
        let packet_attempt = SbusPacket::try_from_bytes(&data);
        let error_kind = packet_attempt.unwrap_err().kind();
        let rejected_correctly = match error_kind {
            ErrorKind::NotSbus => true,
            _ => false,
        };
        assert!(rejected_correctly);
    }
}
