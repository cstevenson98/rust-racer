{ pkgs ? import <nixpkgs> { } }:

pkgs.mkShell rec {
  name = "rust-game-dev";

  nativeBuildInputs = with pkgs; [
    pkg-config
    vulkan-tools
  ];

  buildInputs = with pkgs; [
    udev
    alsa-lib
    vulkan-loader
    vulkan-validation-layers
    libx11
    libxcursor
    libxi
    libxrandr
    libxkbcommon
    wayland
  ];

  # winit dlopens these at runtime. /run/opengl-driver is the host Vulkan ICD.
  LD_LIBRARY_PATH =
    pkgs.lib.makeLibraryPath buildInputs + ":/run/opengl-driver/lib";

  VK_LAYER_PATH = "${pkgs.vulkan-validation-layers}/share/vulkan/explicit_layer.d";

  # Allow cargo to link proc-macro .so files under $HOME.
  NIX_ENFORCE_PURITY = 0;
}
