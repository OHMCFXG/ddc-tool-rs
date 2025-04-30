# DDC Tool RS

A command-line tool written in Rust to control monitor features via DDC/CI protocol.

## Features

- Get feature values from monitors (brightness, contrast, etc.)
- Set feature values:
  - Absolute values (e.g., brightness to 50)
  - Relative percentages (e.g., increase brightness by 2%)
- Listen mode for integration with status bars like waybar
- Support for both monitor names (e.g., DP-1) and I2C device paths (e.g., /dev/i2c-3)

## Installation

### From Source

```bash
git clone https://github.com/OHMCFXG/ddc-tool-rs.git
cd ddc-tool-rs
cargo build --release
```

The compiled binary will be at `target/release/ddc-tool-rs`.

## Usage

### Get a Feature Value

```bash
# Using monitor name
ddc-tool-rs -d DP-1 get 0x10

# Or using I2C device path
ddc-tool-rs -d /dev/i2c-3 get 0x10
```

### Set a Feature Value

```bash
# Set absolute value (brightness to 50)
ddc-tool-rs -d DP-1 set 0x10 50

# Increase by 2% of maximum brightness
ddc-tool-rs -d DP-1 set 0x10 2%+

# Decrease by 2% of maximum brightness
ddc-tool-rs -d DP-1 set 0x10 2%-
```

### Listen Mode

```bash
ddc-tool-rs -d DP-1 listen 0x10
```

This listens for changes to the monitor's brightness value without polling. When another program modifies the brightness using the `set` command, the listen mode immediately captures the change and prints the new value. Useful for integration with tools like waybar.

## Common Feature Codes

- `0x10`: Brightness
- `0x12`: Contrast
- `0x60`: Input source
- `0xD6`: Power state

## Notes

- Requires root permissions or adding the user to the i2c group
- May require loading the i2c-dev kernel module
- NVIDIA GPUs may require additional configuration

## License

GPL v3 