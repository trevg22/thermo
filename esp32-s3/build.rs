fn main() {
    println!("cargo:rustc-link-arg=-Tlinkall.x");

    // The esp32s3 GDMA can read PSRAM directly. esp-hal gates the
    // `Preparation::accesses_psram` field and PSRAM cache handling behind this
    // cfg, so enable it here too (declared first so `--check-cfg` stays quiet).
    println!("cargo:rustc-check-cfg=cfg(dma_can_access_psram)");
    println!("cargo:rustc-cfg=dma_can_access_psram");

    let config = slint_build::CompilerConfiguration::new()
        // Pre-rasterise glyphs so no font parsing or decoding happens on the MCU.
        .embed_resources(slint_build::EmbedResourcesKind::EmbedForSoftwareRenderer);

    slint_build::compile_with_config("ui/main.slint", config)
        .expect("failed to compile ui/main.slint");
}
