use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Copy this crate's own memory.x into OUT_DIR and put OUT_DIR on the
    // linker search path.  Without this, cortex-m-rt's `INCLUDE memory.x`
    // resolves to the workspace-root memory.x (ORIGIN = 0x08004000) and the
    // bootloader gets linked into the app's region instead of 0x08000000.
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
}
