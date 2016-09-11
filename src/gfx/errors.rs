use std::error::Error as StdError;
use std::fmt::{Display, Formatter};
use std::fmt::Result as FmtResult;

error_chain!{}

#[derive(Debug)]
pub struct SdlError(pub String);

impl StdError for SdlError {
    fn description(&self) -> &str {
        &self.0
    }
}

impl Display for SdlError {
    fn fmt(&self, fmt: &mut Formatter) -> FmtResult {
        self.0.fmt(fmt)
    }
}
