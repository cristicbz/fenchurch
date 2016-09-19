use std::fs::File;
use std::io::Read;
use std::path::Path;

error_chain!{}

pub fn read_utf8_file<P: AsRef<Path>>(path: P) -> Result<String> {
    let path = path.as_ref();
    let mut output = String::new();
    let mut file = try!(File::open(path).chain_err(|| format!("Error opening {:?}", path)));
    try!(file.read_to_string(&mut output).chain_err(|| format!("Error reading {:?}", path)));
    Ok(output)
}
