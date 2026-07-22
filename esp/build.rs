fn main() {
    emit_ble_address();
    linker_be_nice();
    // make sure linkall.x is the last linker script (otherwise might cause problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}

/// Validate an optional `BLE_ADDRESS` override and hand it to the firmware.
///
/// The address is a static-random one written most-significant octet first,
/// e.g. "FF:C6:A1:53:50:47". Absent, nothing is emitted and the firmware
/// derives a per-chip address from the eFuse MAC (`option_env!` sees `None`).
/// Validating here means a bad override fails the build rather than flashing
/// a radio that will not advertise. The normalized (uppercase) string is
/// passed through `cargo:rustc-env`, where `option_env!` reads it.
fn emit_ble_address() {
    println!("cargo:rerun-if-env-changed=BLE_ADDRESS");
    let raw = match std::env::var("BLE_ADDRESS") {
        Ok(v) => v,
        Err(_) => return,
    };

    let octets: Vec<u8> = raw
        .split(':')
        .map(|o| {
            u8::from_str_radix(o, 16)
                .unwrap_or_else(|_| panic!("BLE_ADDRESS octet {o:?} is not two hex digits"))
        })
        .collect();
    if octets.len() != 6 {
        panic!("BLE_ADDRESS must be 6 colon-separated octets, got {}", octets.len());
    }
    // Static-random requires the two most-significant bits of the MSB set.
    if octets[0] & 0xC0 != 0xC0 {
        panic!(
            "BLE_ADDRESS {raw:?} is not a static-random address: the top two bits of the first \
             octet must be set (first octet 0xC0-0xFF)"
        );
    }

    let normalized = octets
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(":");
    println!("cargo:rustc-env=BLE_ADDRESS={normalized}");
}

fn linker_be_nice() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 {
        let kind = &args[1];
        let what = &args[2];

        match kind.as_str() {
            "undefined-symbol" => match what.as_str() {
                what if what.starts_with("_defmt_") => {
                    eprintln!();
                    eprintln!(
                        "💡 `defmt` not found - make sure `defmt.x` is added as a linker script and you have included `use defmt_rtt as _;`"
                    );
                    eprintln!();
                }
                "_stack_start" => {
                    eprintln!();
                    eprintln!("💡 Is the linker script `linkall.x` missing?");
                    eprintln!();
                }
                what if what.starts_with("esp_rtos_") => {
                    eprintln!();
                    eprintln!(
                        "💡 `esp-radio` has no scheduler enabled. Make sure you have initialized `esp-rtos` or provided an external scheduler."
                    );
                    eprintln!();
                }
                "embedded_test_linker_file_not_added_to_rustflags" => {
                    eprintln!();
                    eprintln!(
                        "💡 `embedded-test` not found - make sure `embedded-test.x` is added as a linker script for tests"
                    );
                    eprintln!();
                }
                "free"
                | "malloc"
                | "calloc"
                | "get_free_internal_heap_size"
                | "malloc_internal"
                | "realloc_internal"
                | "calloc_internal"
                | "free_internal" => {
                    eprintln!();
                    eprintln!(
                        "💡 Did you forget the `esp-alloc` dependency or didn't enable the `compat` feature on it?"
                    );
                    eprintln!();
                }
                _ => (),
            },
            // we don't have anything helpful for "missing-lib" yet
            _ => {
                std::process::exit(1);
            }
        }

        std::process::exit(0);
    }

    println!(
        "cargo:rustc-link-arg=--error-handling-script={}",
        std::env::current_exe().unwrap().display()
    );
}
