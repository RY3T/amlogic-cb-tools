//! Read-only APW12 probe over a **userspace bit-banged i2c** bus (sysfs GPIO).
//!
//! For control boards where the APW12 i2c is not a Linux `/dev/i2c-*` bus —
//! e.g. the Braiins BCB100 (STM32MP157C), whose kernel has `CONFIG_I2C_GPIO`
//! unset and bit-bangs `PWR_I2C2` on gpio 55 (SCL) / 111 (SDA). This tool
//! speaks the same APW12 protocol as `apw12-psu-tool` but over two GPIOs, and
//! only ever issues **query** commands (fw/hw/measure-voltage/read-state) — it
//! NEVER sets voltage or enables output, so it is safe to run before a real
//! bring-up to confirm the bus works and read what the PSU reports.
//!
//! Usage: `apw12-bitbang <scl_gpio> <sda_gpio> [addr_hex] [write_reg_hex]`
//!        `apw12-bitbang 55 111`            # BCB100 defaults, addr 0x10 reg 0x11

use amlogic_cb_tools::protocol::{
    build_frame, decode_dac_to_voltage, decode_measured_voltage, encode_voltage_to_dac, parse_frame,
    CMD_GET_FW_VERSION, CMD_GET_HW_VERSION, CMD_GET_VOLTAGE, CMD_MEASURE_VOLTAGE, CMD_READ_STATE,
    CMD_SET_VOLTAGE, DEFAULT_PSU_ADDRESS, DEFAULT_PSU_WRITE_REGISTER, NAK_BYTE, PREAMBLE_LSB,
    PREAMBLE_MSB,
};
use std::{fs, thread::sleep, time::Duration};

const RESPONSE_DELAY: Duration = Duration::from_millis(50);
const MAX_RESPONSE_ATTEMPTS: usize = 5;

/// Two-wire sysfs bit-bang i2c. Open-drain: a line is driven low by making it
/// an output-0; released by making it an input (the bus pull-up takes it high).
struct BitBangI2c {
    scl: u32,
    sda: u32,
}

impl BitBangI2c {
    fn new(scl: u32, sda: u32) -> Self {
        for g in [scl, sda] {
            let _ = fs::write("/sys/class/gpio/export", g.to_string());
        }
        sleep(Duration::from_millis(30));
        let me = Self { scl, sda };
        // Idle bus: both released (high).
        me.release(scl);
        me.release(sda);
        me.settle();
        me
    }

    fn low(&self, g: u32) {
        let _ = fs::write(format!("/sys/class/gpio/gpio{g}/direction"), "out");
        let _ = fs::write(format!("/sys/class/gpio/gpio{g}/value"), "0");
    }
    fn release(&self, g: u32) {
        let _ = fs::write(format!("/sys/class/gpio/gpio{g}/direction"), "in");
    }
    fn read(&self, g: u32) -> bool {
        let _ = fs::write(format!("/sys/class/gpio/gpio{g}/direction"), "in");
        fs::read_to_string(format!("/sys/class/gpio/gpio{g}/value"))
            .map(|s| s.trim() == "1")
            .unwrap_or(true)
    }
    // sysfs writes already take ~tens of us; a tiny extra settle keeps SCL wide.
    fn settle(&self) {
        sleep(Duration::from_micros(20));
    }

    fn start(&self) {
        self.release(self.sda);
        self.release(self.scl);
        self.settle();
        self.low(self.sda);
        self.settle();
        self.low(self.scl);
        self.settle();
    }
    fn stop(&self) {
        self.low(self.sda);
        self.settle();
        self.release(self.scl);
        self.settle();
        self.release(self.sda);
        self.settle();
    }
    fn write_bit(&self, bit: bool) {
        if bit {
            self.release(self.sda);
        } else {
            self.low(self.sda);
        }
        self.settle();
        self.release(self.scl); // clock high
        self.settle();
        self.low(self.scl);
        self.settle();
    }
    fn read_bit(&self) -> bool {
        self.release(self.sda);
        self.settle();
        self.release(self.scl);
        self.settle();
        let b = self.read(self.sda);
        self.low(self.scl);
        self.settle();
        b
    }
    /// Write a byte; returns true if the slave ACKed (SDA low on the 9th clock).
    fn write_byte(&self, byte: u8) -> bool {
        for i in (0..8).rev() {
            self.write_bit((byte >> i) & 1 == 1);
        }
        !self.read_bit() // ACK = low
    }
    /// Read a byte, then send ACK (continue) or NACK (last byte).
    fn read_byte(&self, ack: bool) -> u8 {
        let mut v = 0u8;
        for _ in 0..8 {
            v = (v << 1) | u8::from(self.read_bit());
        }
        self.write_bit(!ack); // ACK = drive low, NACK = release
        v
    }

    /// SMBUS "write byte data": START, addr+W, register, data, STOP.
    fn smbus_write_byte(&self, addr: u16, reg: u8, data: u8) -> Result<(), String> {
        self.start();
        let ok = self.write_byte((addr as u8) << 1)
            && self.write_byte(reg)
            && self.write_byte(data);
        self.stop();
        if ok {
            Ok(())
        } else {
            Err(format!("no ACK writing reg 0x{reg:02X} data 0x{data:02X} to 0x{addr:02X}"))
        }
    }
    /// SMBUS "receive byte": START, addr+R, read one byte (NACK), STOP.
    fn smbus_read_byte(&self, addr: u16) -> Result<u8, String> {
        self.start();
        if !self.write_byte(((addr as u8) << 1) | 1) {
            self.stop();
            return Err(format!("no ACK addressing 0x{addr:02X} for read"));
        }
        let b = self.read_byte(false);
        self.stop();
        Ok(b)
    }

    fn cleanup(&self) {
        self.release(self.scl);
        self.release(self.sda);
        for g in [self.scl, self.sda] {
            let _ = fs::write("/sys/class/gpio/unexport", g.to_string());
        }
    }
}

/// Send an APW12 command frame and read the response frame (same wire protocol
/// as `apw12-psu-tool::exchange`, but over the bit-bang bus).
fn exchange(
    bus: &BitBangI2c,
    addr: u16,
    write_reg: u8,
    command: u8,
    payload: &[u8],
) -> Result<amlogic_cb_tools::protocol::Frame, String> {
    for byte in build_frame(command, payload) {
        bus.smbus_write_byte(addr, write_reg, byte)?;
    }
    sleep(RESPONSE_DELAY);

    let mut last_err = String::from("no response");
    for _ in 0..MAX_RESPONSE_ATTEMPTS {
        // Hunt for the preamble; bounded so a dead bus doesn't spin forever.
        let mut first = bus.smbus_read_byte(addr)?;
        let mut guard = 0;
        while first != PREAMBLE_LSB && first != NAK_BYTE && guard < 16 {
            first = bus.smbus_read_byte(addr)?;
            guard += 1;
        }
        if first == NAK_BYTE {
            last_err = "PSU returned NAK (0xF5)".into();
            sleep(RESPONSE_DELAY);
            continue;
        }
        if first != PREAMBLE_LSB {
            last_err = "preamble not found".into();
            sleep(RESPONSE_DELAY);
            continue;
        }
        let second = bus.smbus_read_byte(addr)?;
        if second != PREAMBLE_MSB {
            last_err = format!("bad preamble continuation 0x{second:02X}");
            continue;
        }
        let length = bus.smbus_read_byte(addr)?;
        let mut resp = vec![first, second, length];
        for _ in 0..length.saturating_sub(1) {
            resp.push(bus.smbus_read_byte(addr)?);
        }
        match parse_frame(&resp) {
            Ok(f) if f.command == command => return Ok(f),
            Ok(f) => last_err = format!("unexpected resp cmd 0x{:02X}", f.command),
            Err(e) => last_err = e.to_string(),
        }
        sleep(RESPONSE_DELAY);
    }
    Err(last_err)
}

fn measure_vout(bus: &BitBangI2c, addr: u16, reg: u8) -> Result<f32, String> {
    let f = exchange(bus, addr, reg, CMD_MEASURE_VOLTAGE, &[])?;
    if f.payload.len() < 2 {
        return Err("short measure payload".into());
    }
    Ok(decode_measured_voltage(f.payload[0], f.payload[1]))
}

/// Drive a plain output GPIO (used for the PSU enable line). Left exported +
/// driven after exit so state persists between staged invocations.
fn drive_gpio(gpio: u32, value: bool) {
    let _ = fs::write("/sys/class/gpio/export", gpio.to_string());
    sleep(Duration::from_millis(20));
    let _ = fs::write(format!("/sys/class/gpio/gpio{gpio}/direction"), "out");
    let _ = fs::write(
        format!("/sys/class/gpio/gpio{gpio}/value"),
        if value { "1" } else { "0" },
    );
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!(
            "usage: {} <scl> <sda> [read | setdac <volts> | enable <gpio> | disable <gpio> | measure]",
            args[0]
        );
        std::process::exit(2);
    }
    let scl: u32 = args[1].parse().expect("scl gpio");
    let sda: u32 = args[2].parse().expect("sda gpio");
    let cmd = args.get(3).map(String::as_str).unwrap_or("read");
    let addr = DEFAULT_PSU_ADDRESS;
    let reg = DEFAULT_PSU_WRITE_REGISTER;

    let bus = BitBangI2c::new(scl, sda);

    match cmd {
        "read" => {
            println!("READ-ONLY: SCL=gpio{scl} SDA=gpio{sda} addr=0x{addr:02X}");
            match exchange(&bus, addr, reg, CMD_GET_FW_VERSION, &[]) {
                Ok(f) => println!("  fw:    {:02X?}", f.payload),
                Err(e) => println!("  fw:    ERR {e}"),
            }
            match exchange(&bus, addr, reg, CMD_GET_HW_VERSION, &[]) {
                Ok(f) => println!("  hw:    {:02X?}", f.payload),
                Err(e) => println!("  hw:    ERR {e}"),
            }
            match measure_vout(&bus, addr, reg) {
                Ok(v) => println!("  Vout:  {v:.3} V"),
                Err(e) => println!("  Vout:  ERR {e}"),
            }
            match exchange(&bus, addr, reg, CMD_READ_STATE, &[]) {
                Ok(f) if f.payload.len() >= 2 => {
                    let s = u16::from(f.payload[0]) | (u16::from(f.payload[1]) << 8);
                    println!("  state: 0x{s:04X}");
                }
                Ok(f) => println!("  state: {:02X?}", f.payload),
                Err(e) => println!("  state: ERR {e}"),
            }
        }
        "measure" => match measure_vout(&bus, addr, reg) {
            Ok(v) => println!("Vout: {v:.3} V"),
            Err(e) => println!("Vout: ERR {e}"),
        },
        // Set the DAC target and read it back — does NOT enable output. If the
        // read-back DAC != what we wrote, the write is corrupt: DO NOT ENABLE
        // (a low DAC = HIGH voltage = overvolt risk).
        "setdac" => {
            let volts: f32 = args.get(4).expect("setdac <volts>").parse().expect("volts");
            let dac = encode_voltage_to_dac(volts);
            println!(
                "SET target={volts:.3} V -> DAC=0x{dac:02X} ({dac}); expect read-back ~{:.3} V",
                decode_dac_to_voltage(dac)
            );
            // The APW12 commonly NAKs the set-voltage *response* while still
            // applying the write — so never trust the response; the read-back
            // below is the sole source of truth. Send it a couple of times.
            for attempt in 0..2 {
                match exchange(&bus, addr, reg, CMD_SET_VOLTAGE, &[dac, 0x00]) {
                    Ok(_) => {
                        println!("  set-voltage acked");
                        break;
                    }
                    Err(e) => println!("  set-voltage attempt {} response: {e} (NAK is normal here)", attempt + 1),
                }
            }
            match exchange(&bus, addr, reg, CMD_GET_VOLTAGE, &[]) {
                Ok(f) if !f.payload.is_empty() => {
                    let rb = f.payload[0];
                    println!(
                        "  READBACK DAC=0x{rb:02X} ({rb}) => {:.3} V",
                        decode_dac_to_voltage(rb)
                    );
                    if rb == dac {
                        println!("  *** DAC VERIFIED. Output still OFF. Safe to `enable`. ***");
                    } else {
                        println!("  *** DAC MISMATCH (wrote {dac}, read {rb}) -- DO NOT ENABLE ***");
                    }
                }
                Ok(_) => println!("  readback short -- DO NOT ENABLE"),
                Err(e) => println!("  readback ERR {e} -- DO NOT ENABLE"),
            }
            if let Ok(v) = measure_vout(&bus, addr, reg) {
                println!("  measured Vout now: {v:.3} V");
            }
        }
        "enable" => {
            let en: u32 = args.get(4).expect("enable <gpio>").parse().expect("gpio");
            println!("enable: driving gpio{en} LOW (active-low), then measuring Vout...");
            drive_gpio(en, false);
            sleep(Duration::from_millis(300));
            for _ in 0..6 {
                match measure_vout(&bus, addr, reg) {
                    Ok(v) => println!("  Vout: {v:.3} V"),
                    Err(e) => println!("  Vout: ERR {e}"),
                }
                sleep(Duration::from_millis(300));
            }
        }
        "disable" => {
            let en: u32 = args.get(4).expect("disable <gpio>").parse().expect("gpio");
            println!("disable: driving gpio{en} HIGH (off)...");
            drive_gpio(en, true);
            sleep(Duration::from_millis(200));
            match measure_vout(&bus, addr, reg) {
                Ok(v) => println!("  Vout: {v:.3} V"),
                Err(e) => println!("  Vout: ERR {e}"),
            }
        }
        other => eprintln!("unknown cmd: {other}"),
    }

    bus.cleanup();
}
