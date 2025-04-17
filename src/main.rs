use once_cell::sync::Lazy;
use serde::Deserialize;
use serialport::SerialPort;
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Write};
use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

// --- Configuration ---

#[derive(Debug, Deserialize, Clone)]
struct Config {
    #[serde(rename = "IMAGE_PATH")]
    image_path: String,
    #[serde(rename = "COM_PORT")]
    com_port: String,
    #[serde(rename = "COM_BAUDRATE")]
    com_baudrate: u32,
    #[serde(rename = "MAX_SLOT")]
    max_slot: usize,
    #[serde(rename = "AREA_SCOPE")]
    area_scope: f64,
    #[serde(rename = "AREA_POINT_NUM")]
    area_point_num: usize,
    #[serde(rename = "ANDROID_ABS_MONITOR_SIZE")]
    android_abs_monitor_size: [u32; 2],
    #[serde(rename = "ANDROID_ABS_INPUT_SIZE")]
    android_abs_input_size: [u32; 2],
    #[serde(rename = "ANDROID_REVERSE_MONITOR")]
    android_reverse_monitor: bool,
    #[serde(rename = "TOUCH_THREAD_SLEEP_MODE")]
    touch_thread_sleep_mode: bool, // Note: This name might be misleading in Rust context
    #[serde(rename = "TOUCH_THREAD_SLEEP_DELAY")]
    touch_thread_sleep_delay: u64, // Microseconds
    #[serde(rename = "SPECIFIED_DEVICES")]
    specified_devices: Option<String>,
    exp_image_dict: HashMap<String, String>,
}

static CONFIG: Lazy<Config> = Lazy::new(|| load_config("config.yaml").expect("Failed to load config"));
static IMAGE: Lazy<image::RgbImage> = Lazy::new(|| {
    image::open(&CONFIG.image_path)
        .expect("Failed to load image")
        .to_rgb8()
});
static ANDROID_REVERSE_MONITOR: Lazy<Arc<AtomicBool>> =
    Lazy::new(|| Arc::new(AtomicBool::new(CONFIG.android_reverse_monitor)));

// --- Constants & Global State (Derived) ---
static EXP_LIST: Lazy<Vec<Vec<&'static str>>> = Lazy::new(|| {
    vec![
        vec!["A1", "A2", "A3", "A4", "A5"],
        vec!["A6", "A7", "A8", "B1", "B2"],
        vec!["B3", "B4", "B5", "B6", "B7"],
        vec!["B8", "C1", "C2", "D1", "D2"],
        vec!["D3", "D4", "D5", "D6", "D7"],
        vec!["D8", "E1", "E2", "E3", "E4"],
        vec!["E5", "E6", "E7", "E8"],
    ]
});

static ABS_MULTI_X: Lazy<f64> = Lazy::new(|| {
    CONFIG.android_abs_monitor_size[0] as f64 / CONFIG.android_abs_input_size[0] as f64
});
static ABS_MULTI_Y: Lazy<f64> = Lazy::new(|| {
    CONFIG.android_abs_monitor_size[1] as f64 / CONFIG.android_abs_input_size[1] as f64
});

// --- Error Handling ---
#[derive(Debug, thiserror::Error)]
enum AppError {
    #[error("I/O error: {0}")]
    Io(#[from] io::Error),
    #[error("Serial port error: {0}")]
    Serial(#[from] serialport::Error),
    #[error("YAML parsing error: {0}")]
    Yaml(#[from] serde_yaml::Error),
    #[error("Image loading error: {0}")]
    Image(#[from] image::ImageError),
    #[error("Hex decoding error: {0}")]
    Hex(#[from] hex::FromHexError),
    #[error("Channel send error: {0}")]
    SendError(String),
    #[error("ADB command failed")]
    AdbCommand,
    #[error("Invalid ADB output format")]
    AdbFormat,
}

// --- Types ---
type Result<T> = std::result::Result<T, Box<dyn Error>>; // Simplified error type

#[derive(Debug, Clone, Default)]
struct TouchPoint {
    p: bool, // pressed
    x: f64,
    y: f64,
}

type TouchData = Vec<TouchPoint>;
type TouchKeys = HashSet<String>;
type TouchPacket = Vec<u8>;
type SerialCommand = (TouchPacket, Vec<String>); // (packet_bytes, keys_pressed)

// --- Utility Functions ---

fn load_config(path: &str) -> Result<Config> {
    let file = File::open(path)?;
    let config: Config = serde_yaml::from_reader(file)?;
    Ok(config)
}

fn get_color_name(pixel: &image::Rgb<u8>) -> String {
    format!("{}-{}-{}", pixel[0], pixel[1], pixel[2])
}

fn get_colors_in_area(x: f64, y: f64) -> Vec<String> {
    let mut colors = HashSet::new();
    let (img_width, img_height) = IMAGE.dimensions();
    let num_points = CONFIG.area_point_num;
    let angle_increment = 360.0 / num_points as f64;

    let center_x = x.round() as u32;
    let center_y = y.round() as u32;

    // Center point
    if center_x < img_width && center_y < img_height {
        colors.insert(get_color_name(IMAGE.get_pixel(center_x, center_y)));
    }

    // Points on the circle
    for i in 0..num_points {
        let angle = (i as f64 * angle_increment).to_radians();
        let dx = CONFIG.area_scope * angle.cos();
        let dy = CONFIG.area_scope * angle.sin();
        let px = (x + dx).round() as i64;
        let py = (y + dy).round() as i64;

        if px >= 0 && px < img_width as i64 && py >= 0 && py < img_height as i64 {
            colors.insert(get_color_name(IMAGE.get_pixel(px as u32, py as u32)));
        }
    }
    colors.into_iter().collect()
}

fn build_touch_package(sl: &[Vec<u8>]) -> Result<TouchPacket> {
    let mut sum_list = vec![0u8; sl.len()];
    for (i, row) in sl.iter().enumerate() {
        for (j, &val) in row.iter().enumerate() {
            if val == 1 {
                sum_list[i] += 1 << j;
            }
        }
    }

    let hex_parts: Vec<String> = sum_list.iter().map(|&b| format!("{:02X}", b)).collect();
    let hex_string = format!("28 {} 29", hex_parts.join(" "));
    Ok(hex::decode(hex_string.replace(" ", ""))?)
}

fn convert_touch_data(touch_data: &TouchData) -> Result<SerialCommand> {
    let mut touch_keys: TouchKeys = HashSet::new();

    for point in touch_data.iter().filter(|p| p.p) {
        let colors = get_colors_in_area(point.x, point.y);
        for color_str in colors {
            if let Some(key) = CONFIG.exp_image_dict.get(&color_str) {
                touch_keys.insert(key.clone());
            }
        }
    }

    let mut current_exp_list: Vec<Vec<u8>> = EXP_LIST
        .iter()
        .map(|row| row.iter().map(|_| 0).collect())
        .collect();

    for (i, row) in EXP_LIST.iter().enumerate() {
        for (j, key) in row.iter().enumerate() {
            if touch_keys.contains(*key) {
                current_exp_list[i][j] = 1;
            }
        }
    }

    let packet = build_touch_package(&current_exp_list)?;
    let keys_vec = touch_keys.into_iter().collect();
    Ok((packet, keys_vec))
}

// --- Serial Communication ---

fn serial_manager(
    port_name: String,
    baud_rate: u32,
    cmd_rx: crossbeam_channel::Receiver<SerialCommand>,
) -> Result<()> {
    let mut port = serialport::new(&port_name, baud_rate)
        .timeout(Duration::from_millis(10)) // Short timeout for non-blocking reads
        .open()?;
    println!("Opened serial port {} at {} baud", port_name, baud_rate);

    let mut setting_packet = [40u8, 0, 0, 0, 0, 41];
    let mut start_up = Arc::new(AtomicBool::new(false));
    let mut last_sent_packet = Vec::new();
    let mut last_printed_keys: Vec<String> = Vec::new();

    let start_up_clone = start_up.clone();
    let mut port_clone = port.try_clone()?; // Clone for reading thread

    // Serial Reading Thread
    thread::spawn(move || {
        let mut read_buf = [0u8; 6]; // Expect 6 bytes
        loop {
            match port_clone.read(&mut read_buf) {
                Ok(bytes_read) if bytes_read == 6 => {
                    // Process received data (touch_setup logic)
                    let byte_data = read_buf[3];
                    // println!("Received: {:?}", &read_buf[..bytes_read]); // Debug
                    if byte_data == 76 || byte_data == 69 { // L or E
                        start_up_clone.store(false, Ordering::Relaxed);
                        // println!("Game connection stopped.");
                    } else if byte_data == 114 || byte_data == 107 { // r or k
                        for i in 1..5 {
                            setting_packet[i] = read_buf[i];
                        }
                        if let Err(e) = port_clone.write_all(&setting_packet) {
                             eprintln!("Error writing settings packet: {}", e);
                        }
                    } else if byte_data == 65 { // A
                        if !start_up_clone.load(Ordering::Relaxed) {
                             println!("已连接到游戏");
                             start_up_clone.store(true, Ordering::Relaxed);
                        }
                    }
                }
                Ok(_) => {} // Incomplete read, ignore for now
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {} // Expected timeout
                Err(e) => {
                    eprintln!("Serial read error: {}", e);
                    // Consider adding logic to attempt reconnection or exit
                    thread::sleep(Duration::from_secs(1));
                }
            }
             // Small sleep to prevent busy-waiting if no data
             thread::sleep(Duration::from_millis(1));
        }
    });


    // Serial Writing Loop (main thread for this function)
    loop {
        // Check for new commands, non-blocking
        if let Ok((packet, keys)) = cmd_rx.try_recv() {
             if packet != last_sent_packet {
                 last_sent_packet = packet.clone();
                 if start_up.load(Ordering::Relaxed) {
                     if let Err(e) = port.write_all(&last_sent_packet) {
                         eprintln!("Serial write error: {}", e);
                         // Handle error, maybe try to reopen port?
                     }
                 }
                 // Print keys only if they changed
                 let mut sorted_keys = keys.clone();
                 sorted_keys.sort(); // Sort for consistent comparison
                 if sorted_keys != last_printed_keys {
                     println!("Touch Keys: {:?}", sorted_keys);
                     last_printed_keys = sorted_keys;
                 }
             }
        } else {
             // If no new command, potentially resend last packet if needed,
             // or just sleep briefly. The Python version seems to rely on
             // continuous updates from getevent.
             // If start_up is true and last_sent_packet is not empty,
             // we could resend it periodically, but let's stick closer
             // to the Python logic for now which sends on change.
             thread::sleep(Duration::from_micros(100)); // Small sleep to avoid busy loop
        }

        // Optional sleep based on config (though less common in Rust)
        // if CONFIG.touch_thread_sleep_mode {
        //     thread::sleep(Duration::from_micros(CONFIG.touch_thread_sleep_delay));
        // }
    }
}


// --- ADB Event Listener ---

fn adb_event_listener(cmd_tx: crossbeam_channel::Sender<SerialCommand>) -> Result<()> {
    let mut adb_cmd_vec = vec!["adb"];
    if let Some(device) = &CONFIG.specified_devices {
        if !device.is_empty() {
            adb_cmd_vec.push("-s");
            adb_cmd_vec.push(device);
        }
    }
    adb_cmd_vec.push("shell");
    adb_cmd_vec.push("getevent");
    adb_cmd_vec.push("-l"); // Use labels

    println!("Starting ADB command: {:?}", adb_cmd_vec);

    let mut child = Command::new(adb_cmd_vec[0])
        .args(&adb_cmd_vec[1..])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped()) // Capture stderr for debugging
        .spawn()
        .map_err(|e| {
            eprintln!("Failed to start ADB command: {}", e);
            AppError::AdbCommand
        })?;

    let stdout = child.stdout.take().ok_or(AppError::AdbCommand)?;
    let reader = BufReader::new(stdout);

    // Optional: Spawn a thread to read stderr
    if let Some(stderr) = child.stderr.take() {
        thread::spawn(move || {
            let err_reader = BufReader::new(stderr);
            for line in err_reader.lines() {
                match line {
                    Ok(l) => eprintln!("ADB stderr: {}", l),
                    Err(e) => eprintln!("Error reading ADB stderr: {}", e),
                }
            }
        });
    }


    let mut touch_data: TouchData = vec![TouchPoint::default(); CONFIG.max_slot];
    let mut current_slot = 0;
    let mut key_is_changed = false;

    for line in reader.lines() {
        let line = line?;
        // println!("ADB Raw: {}", line); // Debug raw output
        let parts: Vec<&str> = line.split_whitespace().collect();

        if parts.len() < 4 {
            // Ignore lines that don't fit the expected format (like device info lines)
            if !line.contains("EV_SYN") && !line.contains("EV_ABS") && !line.contains("EV_KEY") {
                 // println!("Ignoring line: {}", line);
            }
            continue;
        }

        let event_type = parts[2];
        let event_value_hex = parts[3];

        // Use try_from_int_radix for safer parsing
        let event_value = match u64::from_str_radix(event_value_hex.trim_start_matches("0x"), 16) {
            Ok(v) => v,
            Err(_) => {
                // eprintln!("Failed to parse hex value: '{}' in line: {}", event_value_hex, line);
                continue; // Skip malformed lines
            }
        };

        match event_type {
            "ABS_MT_SLOT" => {
                key_is_changed = true;
                current_slot = event_value as usize;
                if current_slot >= CONFIG.max_slot {
                    eprintln!("Warning: Slot index {} exceeds MAX_SLOT {}", current_slot, CONFIG.max_slot);
                    current_slot = CONFIG.max_slot - 1; // Clamp to max slot
                }
            }
            "ABS_MT_TRACKING_ID" => {
                key_is_changed = true;
                if event_value_hex == "ffffffff" {
                    if current_slot < touch_data.len() {
                        touch_data[current_slot].p = false;
                    }
                } else {
                     if current_slot < touch_data.len() {
                        touch_data[current_slot].p = true;
                    }
                }
            }
            "ABS_MT_POSITION_X" => {
                key_is_changed = true;
                if current_slot < touch_data.len() {
                    let raw_x = event_value as f64 * *ABS_MULTI_X;
                    touch_data[current_slot].x = if ANDROID_REVERSE_MONITOR.load(Ordering::Relaxed) {
                        CONFIG.android_abs_monitor_size[0] as f64 - raw_x
                    } else {
                        raw_x
                    };
                }
            }
            "ABS_MT_POSITION_Y" => {
                key_is_changed = true;
                 if current_slot < touch_data.len() {
                    let raw_y = event_value as f64 * *ABS_MULTI_Y;
                    touch_data[current_slot].y = if ANDROID_REVERSE_MONITOR.load(Ordering::Relaxed) {
                        CONFIG.android_abs_monitor_size[1] as f64 - raw_y
                    } else {
                        raw_y
                    };
                }
            }
            "SYN_REPORT" => {
                if key_is_changed {
                    // println!("Touch Data: {:?}", touch_data); // Debug touch state
                    match convert_touch_data(&touch_data) {
                        Ok(command) => {
                            if let Err(e) = cmd_tx.send(command) {
                                eprintln!("Failed to send command to serial thread: {}", e);
                                // Potentially break or handle channel closure
                            }
                        }
                        Err(e) => {
                            eprintln!("Error converting touch data: {}", e);
                        }
                    }
                    key_is_changed = false;
                }
            }
            _ => {
                // Ignore other event types like EV_KEY, other ABS events if not needed
            }
        }
    }

    // Wait for the ADB process to finish (optional)
    let status = child.wait()?;
    println!("ADB process exited with status: {}", status);

    Ok(())
}

// --- Main Function ---

fn main() -> Result<()> {
    // Initialize static variables (config, image, etc.)
    Lazy::force(&CONFIG);
    Lazy::force(&IMAGE);
    Lazy::force(&ANDROID_REVERSE_MONITOR);
    Lazy::force(&ABS_MULTI_X);
    Lazy::force(&ABS_MULTI_Y);

    println!("Using config: {:?}", *CONFIG);
    println!("Loaded image: {} ({}x{})", CONFIG.image_path, IMAGE.width(), IMAGE.height());
    println!("Current touch area X multiplier: {}", *ABS_MULTI_X);
    println!("Current touch area Y multiplier: {}", *ABS_MULTI_Y);
    println!("Screen reverse initially: {}", ANDROID_REVERSE_MONITOR.load(Ordering::Relaxed));

    // Create channels for communication
    let (cmd_tx, cmd_rx) = crossbeam_channel::unbounded::<SerialCommand>();

    // --- Start Threads ---
    let serial_thread = {
        let port_name = CONFIG.com_port.clone();
        let baud_rate = CONFIG.com_baudrate;
        thread::spawn(move || {
            if let Err(e) = serial_manager(port_name, baud_rate, cmd_rx) {
                eprintln!("Serial manager error: {}", e);
            }
        })
    };

    let adb_thread = {
        let cmd_tx_clone = cmd_tx.clone();
         thread::spawn(move || {
            loop { // Loop to attempt reconnection on ADB failure
                println!("Starting ADB listener...");
                if let Err(e) = adb_event_listener(cmd_tx_clone.clone()) {
                    eprintln!("ADB listener error: {}. Restarting in 5 seconds...", e);
                    thread::sleep(Duration::from_secs(5));
                } else {
                    println!("ADB listener finished unexpectedly. Restarting in 5 seconds...");
                     thread::sleep(Duration::from_secs(5));
                }
            }
        })
    };


    // --- User Input Handling ---
    println!("Enter commands (start, reverse, restart, exit):");
    let stdin = io::stdin();
    for line in stdin.lock().lines() {
        let input = line?.trim().to_lowercase();
        match input.as_str() {
            "start" => {
                // Note: 'start' is handled automatically by serial_manager receiving 'A'
                // This command might not be needed unless forcing a state.
                println!("Game connection state is managed by serial communication.");
                 // If manual override needed: find a way to signal serial_manager's start_up Arc.
            }
            "reverse" => {
                let current = ANDROID_REVERSE_MONITOR.load(Ordering::Relaxed);
                ANDROID_REVERSE_MONITOR.store(!current, Ordering::Relaxed);
                println!("Screen reverse set to: {}", !current);
            }
            "restart" => {
                // Full restart like Python's os.execv is hard.
                // We could try re-initializing parts, but it's complex.
                // Simplest is to ask the user to restart the application.
                println!("Restarting the application is not directly supported. Please exit and run again.");
                // Or attempt a limited re-init if feasible later.
            }
             "exit" => {
                 println!("Exiting...");
                 // Ideally, signal threads to stop gracefully here.
                 // For simplicity now, we just exit. Threads will terminate.
                 std::process::exit(0);
             }
            _ => {
                println!("Unknown command: {}", input);
            }
        }
    }

    // Wait for threads (optional, main loop runs indefinitely)
    // serial_thread.join().expect("Serial thread panicked");
    // adb_thread.join().expect("ADB thread panicked");

    Ok(())
}
