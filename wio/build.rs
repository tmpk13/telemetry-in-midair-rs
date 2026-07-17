use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Copy this crate's memory layout into OUT_DIR as memory.x and put OUT_DIR
    // on the linker search path.  The file is named memory-app.x in the crate
    // so that no bare `memory.x` sits in the workspace root: otherwise the
    // linker (whose CWD is the workspace root) would resolve every member's
    // `INCLUDE memory.x` to it, and the bootloader would link at 0x08004000.
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory-app.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory-app.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
