use std::fs;
use std::fs::read_dir;
use std::path::{Path, PathBuf};
use std::sync::mpsc;

use anyhow::{anyhow, Context, Result};
use clap::{Parser, Subcommand};
use ddc::Ddc;
use notify::{Event, EventKind, RecursiveMode, Watcher};

const DEFAULT_CACHE_DIR: &str = "/tmp/ddc-tool-rs";

#[derive(Parser)]
#[command(
    author, 
    version, 
    about,
    name = env!("CARGO_PKG_NAME"),
    long_about = None
)]
struct Cli {
    /// Monitor name (e.g. DP-1) or device path (e.g. /dev/i2c-3)
    #[arg(short, long)]
    device: String,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Get feature value from display
    Get {
        /// Feature code in decimal or hex (with 0x prefix)
        #[arg(value_parser = parse_feature_code)]
        feature_code: u8,
    },
    /// Set feature value to display
    Set {
        /// Feature code in decimal or hex (with 0x prefix)
        #[arg(value_parser = parse_feature_code)]
        feature_code: u8,

        /// Value to set: absolute value (e.g. 50) or relative percentage (e.g. 5%+ or 5%-)
        value: String,
    },
    /// Listen for changes (for use with status bars like waybar)
    Listen {
        /// Feature code in decimal or hex (with 0x prefix)
        #[arg(value_parser = parse_feature_code)]
        feature_code: u8,
    },
}

fn parse_feature_code(s: &str) -> Result<u8, String> {
    if let Some(hex) = s.strip_prefix("0x") {
        u8::from_str_radix(hex, 16).map_err(|e| format!("Invalid hex feature code: {}", e))
    } else {
        s.parse::<u8>()
            .map_err(|e| format!("Invalid feature code: {}", e))
    }
}

/// User input value type
enum ValueInput {
    /// Absolute value
    Absolute(u16),
    /// Relative percentage (percentage value, is increase)
    Relative(f32, bool),
}

fn parse_value_input(s: &str) -> Result<ValueInput, String> {
    // Handle percentage format: 5%+ or 5%-
    if s.ends_with("%+") || s.ends_with("%-") {
        let is_increase = s.ends_with("%+");
        let percentage = s.trim_end_matches("%+").trim_end_matches("%-");
        let value = percentage.parse::<f32>()
            .map_err(|e| format!("Invalid percentage value: {}", e))?;
        
        if value < 0.0 {
            return Err("Percentage value cannot be negative".to_string());
        }
        
        return Ok(ValueInput::Relative(value, is_increase));
    }
    
    // Handle absolute value
    if let Some(hex) = s.strip_prefix("0x") {
        u16::from_str_radix(hex, 16)
            .map(ValueInput::Absolute)
            .map_err(|e| format!("Invalid hex value: {}", e))
    } else {
        s.parse::<u16>()
            .map(ValueInput::Absolute)
            .map_err(|e| format!("Invalid value: {}", e))
    }
}

fn init_cache_dir() -> Result<()> {
    let path = Path::new(DEFAULT_CACHE_DIR);
    if !path.exists() {
        fs::create_dir_all(path).context("Failed to create cache directory")?;
    }
    Ok(())
}

/// Find I2C device path for the given display output name
fn get_i2c_dev(output: &str) -> Result<String> {
    // Return as-is if it's already a device path
    if output.starts_with("/dev/") {
        return Ok(output.to_string());
    }

    // Find the output directory in /sys/class/drm
    let output_dir = find_output_dir(output)?;
    
    // Find I2C device in the output directory
    find_i2c_device_in_dir(&output_dir, output)
}

/// Find the display output directory in /sys/class/drm
fn find_output_dir(output: &str) -> Result<PathBuf> {
    for entry in read_dir("/sys/class/drm")
        .context("Failed to read /sys/class/drm directory")? 
    {
        let path = entry.context("Failed to read directory entry")?.path();
        
        // Get file name as string
        let file_name = path.file_name();
        let name = match file_name.and_then(|n| n.to_str()) {
            Some(name) => name,
            None => continue,
        };
        
        // Check if name matches format (card*-output_name)
        if !name.starts_with("card") || !name.ends_with(output) {
            continue;
        }
        
        let before_name = name.len() - output.len() - 1;
        if before_name < name.len() && &name[before_name..before_name+1] == "-" {
            return Ok(path);
        }
    }
    
    Err(anyhow!("Output '{}' not found in /sys/class/drm", output))
}

/// Find I2C device in the display output directory
fn find_i2c_device_in_dir(dir: &Path, output: &str) -> Result<String> {
    for entry in read_dir(dir).context("Failed to read output directory")? {
        let entry = entry.context("Failed to read directory entry")?;
        
        // Get file name as string
        let file_name = entry.file_name();
        let name = match file_name.to_str() {
            Some(name) => name,
            None => continue,
        };
        
        // Match i2c device directly
        if name.starts_with("i2c-") {
            return Ok(format!("/dev/{}", name));
        } 
        // Handle ddc symlink
        else if name == "ddc" {
            if let Ok(link) = entry.path().read_link() {
                if let Some(filename) = link.file_name() {
                    let dev_name = filename.to_string_lossy();
                    return Ok(format!("/dev/{}", dev_name));
                }
            }
        }
    }
    
    Err(anyhow!("I2C device for output '{}' not found", output))
}

fn get_cache_path(device: &str, feature_code: u8) -> String {
    // Extract device name, remove /dev/ prefix if present
    let device_name = if device.starts_with("/dev/") {
        device.trim_start_matches("/dev/")
    } else {
        device
    };
    
    format!(
        "{}/{}_{:#04x}",
        DEFAULT_CACHE_DIR, device_name, feature_code
    )
}

fn save_value_to_cache(device: &str, feature_code: u8, value: u16) -> Result<()> {
    init_cache_dir()?;
    let cache_path = get_cache_path(device, feature_code);
    fs::write(&cache_path, value.to_string()).context("Failed to write cache file")?;
    Ok(())
}

fn read_value_from_cache(device: &str, feature_code: u8) -> Result<u16> {
    let cache_path = get_cache_path(device, feature_code);
    let content = fs::read_to_string(&cache_path).context("Failed to read cache file")?;
    let value = content
        .trim()
        .parse::<u16>()
        .context("Failed to parse cached value")?;
    Ok(value)
}

fn get_feature_value(device_input: &str, feature_code: u8) -> Result<u16> {
    let device = get_i2c_dev(device_input)?;
    
    let mut ddc = ddc_i2c::from_i2c_device(&device).context("Failed to open I2C device")?;

    let vcp = ddc
        .get_vcp_feature(feature_code)
        .context("Failed to get VCP feature")?;

    Ok(vcp.value())
}

/// Get the maximum value for a feature
fn get_feature_max_value(device_input: &str, feature_code: u8) -> Result<u16> {
    let device = get_i2c_dev(device_input)?;
    
    let mut ddc = ddc_i2c::from_i2c_device(&device).context("Failed to open I2C device")?;

    let vcp = ddc
        .get_vcp_feature(feature_code)
        .context("Failed to get VCP feature")?;

    Ok(vcp.maximum())
}

/// Calculate the new value after applying relative percentage change
fn calculate_relative_value(
    current: u16,
    max: u16,
    percentage: f32,
    is_increase: bool
) -> Result<u16> {
    // Calculate change amount (percentage of max value)
    let change = (max as f32 * percentage / 100.0).round() as i32;
    
    // Calculate new value based on direction
    let new_value = if is_increase {
        current as i32 + change
    } else {
        current as i32 - change
    };
    
    // Ensure value is within valid range (0 to max)
    let new_value = new_value.max(0).min(max as i32);
    
    Ok(new_value as u16)
}

/// Set display feature value, supporting absolute values and relative percentages
fn set_feature_value_with_input(device_input: &str, feature_code: u8, value_input: ValueInput) -> Result<u16> {
    // Get device path and max value
    let device = get_i2c_dev(device_input)?;
    let max_value = get_feature_max_value(device_input, feature_code)?;
    
    // Calculate final value and determine if warning is needed
    let (final_value, warning_msg) = compute_final_value(device_input, feature_code, value_input, max_value)?;
    
    // Show warning if needed
    if let Some(msg) = warning_msg {
        eprintln!("Warning: {}", msg);
    }
    
    // Set the new value
    let mut ddc = ddc_i2c::from_i2c_device(&device).context("Failed to set VCP feature")?;
    ddc.set_vcp_feature(feature_code, final_value)
        .context("Failed to set VCP feature")?;

    // Save to cache
    save_value_to_cache(device_input, feature_code, final_value)?;

    Ok(final_value)
}

/// Compute final value to set, handling range limits
fn compute_final_value(device_input: &str, feature_code: u8, value_input: ValueInput, max_value: u16) -> Result<(u16, Option<String>)> {
    match value_input {
        ValueInput::Absolute(value) => {
            // Apply range limits to absolute value
            if value > max_value {
                Ok((max_value, Some(format!("Value exceeds maximum {}", max_value))))
            } else {
                Ok((value, None))
            }
        },
        ValueInput::Relative(percentage, is_increase) => {
            // Get current value
            let current_value = get_feature_value(device_input, feature_code)?;
            
            // Calculate relative value (includes range check)
            Ok((calculate_relative_value(current_value, max_value, percentage, is_increase)?, None))
        }
    }
}

fn listen_feature(device_input: String, feature_code: u8) -> Result<()> {
    // Validate device path
    get_i2c_dev(&device_input)?;
    
    // Try to read initial value and print
    let initial_value = match get_feature_value(&device_input, feature_code) {
        Ok(value) => {
            save_value_to_cache(&device_input, feature_code, value)?;
            value
        },
        Err(e) => {
            // Try to read from cache, return original error if fails
            read_value_from_cache(&device_input, feature_code).map_err(|_| e)?
        }
    };
    
    // Print initial value
    println!("{}", initial_value);
    
    // Track last printed value for deduplication
    let mut last_value = initial_value;
    
    // Prepare file monitoring
    let cache_path = get_cache_path(&device_input, feature_code);
    let cache_file = PathBuf::from(&cache_path);
    
    // Create filesystem watcher
    let (tx, rx) = mpsc::channel();
    let mut watcher = notify::recommended_watcher(move |res| {
        let _ = tx.send(res);
    })?;
    
    // Start watching for file changes
    watcher.watch(Path::new(DEFAULT_CACHE_DIR), RecursiveMode::NonRecursive)?;
    
    // Main loop: listen for file change events
    while let Ok(event) = rx.recv() {
        let event = match event {
            Ok(e) => e,
            Err(e) => {
                eprintln!("Watch error: {:?}", e);
                continue;
            }
        };
        
        // Skip events not related to our file
        if !should_handle_event(&event, &cache_file) {
            continue;
        }
        
        // Read and print new value, try to get from device if cache fails
        match read_value_from_cache(&device_input, feature_code) {
            Ok(new_value) => {
                // Only print if value has changed
                if new_value != last_value {
                    println!("{}", new_value);
                    last_value = new_value;
                }
            },
            Err(_) => {
                if let Ok(value) = get_feature_value(&device_input, feature_code) {
                    save_value_to_cache(&device_input, feature_code, value)?;
                    // Only print if value has changed
                    if value != last_value {
                        println!("{}", value);
                        last_value = value;
                    }
                }
            }
        }
    }
    
    // Will never reach here as rx.recv() will block forever
    Ok(())
}

// Check if the event should be handled
fn should_handle_event(event: &Event, cache_file: &Path) -> bool {
    match event.kind {
        EventKind::Create(_) | EventKind::Modify(_) | EventKind::Remove(_) => {
            event.paths.iter().any(|path| path == cache_file)
        },
        _ => false,
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match &cli.command {
        Commands::Get { feature_code } => {
            handle_get_command(&cli.device, *feature_code)?;
        }
        Commands::Set { feature_code, value } => {
            handle_set_command(&cli.device, *feature_code, value)?;
        }
        Commands::Listen { feature_code } => {
            listen_feature(cli.device.clone(), *feature_code)?;
        }
    }

    Ok(())
}

/// Handle the Get command
fn handle_get_command(device: &str, feature_code: u8) -> Result<()> {
    let value = get_feature_value(device, feature_code)?;
    println!("Feature {:#04x}: {}", feature_code, value);

    // Update cache
    save_value_to_cache(device, feature_code, value)?;
    Ok(())
}

/// Handle the Set command
fn handle_set_command(device: &str, feature_code: u8, value_str: &str) -> Result<()> {
    // Parse value input
    let value_input = parse_value_input(value_str)
        .map_err(|e| anyhow!("Failed to parse value: {}", e))?;
    
    // Set value
    let new_value = set_feature_value_with_input(device, feature_code, value_input)?;
    
    println!("Set feature {:#04x} to {}", feature_code, new_value);
    Ok(())
}
