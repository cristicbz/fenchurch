use std::fs::File;
use std::io::Read;
use std::path::Path;
use super::errors::{Result, ChainErr};
use std::cmp::Ordering;

pub fn read_utf8_file<P: AsRef<Path>>(path: P) -> Result<String> {
    let path = path.as_ref();
    let mut output = String::new();
    let mut file = try!(File::open(path).chain_err(|| format!("Error opening {:?}", path)));
    try!(file.read_to_string(&mut output).chain_err(|| format!("Error reading {:?}", path)));
    Ok(output)
}

pub fn pmin<T: PartialOrd>(x: T, y: T) -> T {
    if x < y { x } else { y }
}

pub fn pmax<T: PartialOrd>(x: T, y: T) -> T {
    if x > y { x } else { y }
}

pub fn pmin_pmax<T: PartialOrd>(x: T, y: T) -> (T, T) {
    if x < y { (x, y) } else { (y, x) }
}

#[derive(PartialOrd, PartialEq, Copy, Clone)]
pub struct AssertOrd<T: PartialOrd + PartialEq>(pub T);

impl<T: PartialEq + PartialOrd> Eq for AssertOrd<T> {}

impl<T: PartialEq + PartialOrd> Ord for AssertOrd<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        if self.0 < other.0 {
            Ordering::Less
        } else if self.0 > other.0 {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }
}
