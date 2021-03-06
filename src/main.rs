#![recursion_limit = "1024"]

extern crate num;
extern crate clap;
extern crate sdl2;
extern crate glium_sdl2;
extern crate sdl2_image;
extern crate time;
extern crate rand;
extern crate rayon;
extern crate parking_lot;
extern crate crossbeam;

#[macro_use]
extern crate idcontain;

#[macro_use]
extern crate glium;

#[macro_use]
extern crate error_chain;

#[macro_use]
extern crate log;

#[macro_use]
extern crate env_logger;

pub mod math;
pub mod gfx;
mod sim;
mod utils;

use std::process;
use clap::{App, AppSettings};

pub fn report_error(e: &sim::Error) {
    error!("fatal error: {}", e);

    for e in e.iter().skip(1) {
        error!("  caused by: {}", e);
    }

    if show_backtrace() {
        if let Some(backtrace) = e.backtrace() {
            error!("{:?}", backtrace);
        }
    } else {
    }

    fn show_backtrace() -> bool {
        use std::env;
        use std::ops::Deref;

        if env::var("RUST_BACKTRACE").as_ref().map(Deref::deref) == Ok("1") {
            return true;
        }

        for arg in env::args() {
            if arg == "-v" || arg == "--verbose" {
                return true;
            }
        }

        return false;
    }
}

fn main() {
    env_logger::init().expect("Failed to initialise logging");

    let _matches = App::new("Fesim::App")
        .version("0.1")
        .author("Cristi Cobzarenco <cristi.cobzarenco@gmail.com>")
        .about("Pods for everyone!")
        .settings(&[AppSettings::ColoredHelp])
        .get_matches();

    if let Err(error) = sim::App::new().and_then(sim::App::run) {
        report_error(&error);
        process::exit(1);
    }
}
