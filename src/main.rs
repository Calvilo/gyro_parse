use crc_all::Crc;
use num_enum::TryFromPrimitive;
use serial2::SerialPort;
use std::convert::TryFrom;
use std::f32::consts::PI;
use std::mem::transmute;
use std::sync::mpsc;
use std::thread;

struct Data_Reciever<T: std::io::Read> {
    data_len: usize,
    dev: T,
    data_flow: mpsc::Sender<Vec<u8>>,
}

impl<T: std::io::Read> Data_Reciever<T> {
    fn new(dev: T, data_flow: mpsc::Sender<Vec<u8>>) -> Self {
        Data_Reciever {
            data_len: 0,
            dev: dev,
            data_flow: data_flow,
        }
    }

    fn run(&mut self) {
        let mut buffer = [0; 128];
        loop {
            self.data_len = self.dev.read(&mut buffer).unwrap();
            if self.data_len > 0 {
                self.data_flow.send(buffer.to_vec()).unwrap();
                // print!("how many bytes: {}", self.data_len);
            }
        }
    }
}
enum ParseState {
    wait_for_header,
    header_check,
    pkg_check,
}
#[repr(u8)]
#[derive(TryFromPrimitive, Debug, Clone, Copy)]
enum PkgType {
    Nonsense = 0x00,
    IMU = 0x40,
    AHRS = 0x41,
    InsGps = 0x42,
    Raw = 0x58,
}
struct Data_Checker {
    data_flow: mpsc::Receiver<Vec<u8>>,
    data_flow_out: mpsc::Sender<(PkgType, Vec<u8>)>,
    raw_data: Vec<u8>,
    parsestate: ParseState,
    pkgtype: PkgType,
    len: u8,
}
impl Data_Checker {
    fn new(
        data_flow: mpsc::Receiver<Vec<u8>>,
        data_flow_out: mpsc::Sender<(PkgType, Vec<u8>)>,
    ) -> Self {
        Data_Checker {
            data_flow: data_flow,
            data_flow_out: data_flow_out,
            raw_data: Vec::new(),
            parsestate: ParseState::wait_for_header,
            pkgtype: PkgType::Nonsense,
            len: 0,
        }
    }
    fn run(&mut self) {
        loop {
            let data = self.data_flow.recv().unwrap();
            // print!("how much data added: {}\n", data.len());
            // append data to raw_data
            // print!("bytes left: {}\n", self.raw_data.len());
            self.raw_data.append(&mut data.to_vec());
            //print!("Big loop bytes left: {}\n", self.raw_data.len());
            // print how much data added
            while self.raw_data.len() > 2 {
                match self.parsestate {
                    ParseState::wait_for_header => {
                        // look for 0xfd 0xfc until bytes left less than 2
                        loop {
                            if self.raw_data.len() < 2 {
                                // print!("wait for more data\n");
                                self.parsestate = ParseState::wait_for_header;
                                break;
                            } else if self.raw_data[0] == 0xfd && self.raw_data[1] == 0xfc {
                                self.parsestate = ParseState::header_check;
                                self.raw_data.remove(0);
                                //print!("header found\n");
                                break;
                            } else {
                                self.parsestate = ParseState::wait_for_header;
                                self.raw_data.remove(0);
                                continue;
                            }
                        }
                        continue;
                    }
                    ParseState::header_check => {
                        if (self.raw_data.len() >= 5) {
                            // use crc8 maxim to validate
                            let mut crc8_maxin = Crc::<u8>::new(0x31, 8, 0x00, 0x00, true);
                            let head_res = crc8_maxin.update(&self.raw_data[0..5]);
                            if head_res == 0x00 {
                                // println!("header check ok\n");
                                let what_type = PkgType::try_from(self.raw_data[1]);
                                match what_type {
                                    Ok(t) => {
                                        self.pkgtype = t;
                                        self.len = self.raw_data[2];
                                        self.parsestate = ParseState::pkg_check;
                                        // print!("type is {:?}\n", self.pkgtype);
                                        continue;
                                    }
                                    Err(e) => {
                                        println!("type is {:?}", e);
                                        print!("wrong type\n");
                                        self.parsestate = ParseState::wait_for_header;
                                        self.raw_data.remove(0);
                                        continue;
                                    }
                                }
                            } else {
                                /*print failed header  */
                                print!(
                                    "header check failed, header content: {:02x?}\n",
                                    self.raw_data[0..5].to_vec()
                                );
                                self.raw_data.remove(0);
                                self.parsestate = ParseState::wait_for_header;
                                continue;
                            }
                        } else {
                            self.parsestate = ParseState::header_check;
                            // print!("wait for more header data\n");
                            break;
                        }
                    }
                    ParseState::pkg_check => {
                        if self.raw_data.len() >= self.len as usize + 8 {
                            //width=16 poly=0x1021 init=0x0000 refin=false refout=false xorout=0x0000 check=0x31c3 residue=0x0000 name="CRC-16/XMODEM"
                            let mut crc_16_xmodem =
                                crc_all::Crc::<u16>::new(0x1021, 16, 0x0000, 0x0000, false);
                            crc_16_xmodem.update(&self.raw_data[7..self.len as usize + 7]);
                            // byte 5 and byte 6 is crc16
                            let crc16 = crc_16_xmodem.update(&self.raw_data[5..7]);
                            if crc16 == 0 {
                                // println!("crc16 check ok\n");
                                self.parsestate = ParseState::wait_for_header;
                                // remove one package
                                //print!("len: {}\n", self.len);
                                let full_pkg = self
                                    .raw_data
                                    .drain(0..self.len as usize + 7)
                                    .collect::<Vec<_>>();
                                let pkg_iden = self.pkgtype;
                                // println!("pkg type: {:?} , cnt{}", pkg_iden, full_pkg[3]);
                                // println!("last showd be {:02x}, first should", full_pkg.last().unwrap());
                                self.data_flow_out.send((pkg_iden, full_pkg)).unwrap();
                                // print bytes left
                                //print!("bytes left: {}\n", self.raw_data.len());
                                self.parsestate = ParseState::wait_for_header;
                                continue;
                            } else {
                                print!("crc16 check failed\n");
                                self.raw_data.remove(0);
                                self.parsestate = ParseState::wait_for_header;
                                continue;
                            }
                        } else {
                            //println!("wait for more load data\n");
                            break;
                        }
                    }
                }
            }
        }
    }
}

#[repr(C)]
#[derive(Debug)]
struct IMU_PKG_DEF {
    GyroscopeX: f32,
    GyroscopeY: f32,
    GyroscopeZ: f32,
    AccelerometerX: f32,
    AccelerometerY: f32,
    AccelerometerZ: f32,
    MagnetometerX: f32,
    MagnetometerY: f32,
    MagnetometerZ: f32,
    Temperature: f32,
    Pressure: f32,
    PressureTemperature: f32,
    TimeStamp: u64,
}
#[repr(C)]
#[derive(Debug)]
struct AHRS_PKG {
    RollSpeed: f32,
    PitchSpeed: f32,
    HeadingSpeed: f32,
    Roll: f32,
    Pitch: f32,
    Heading: f32,
    Q1: f32,
    Q2: f32,
    Q3: f32,
    Q4: f32,
    TimeStamp: u64,
}

struct Data_parser {
    checked_data_flow: mpsc::Receiver<(PkgType, Vec<u8>)>,
}
impl Data_parser {
    fn new(checked_data_flow: mpsc::Receiver<(PkgType, Vec<u8>)>) -> Data_parser {
        Data_parser {
            checked_data_flow: checked_data_flow,
        }
    }
    fn run(&mut self) {
        loop {
            let union_data = self.checked_data_flow.recv();
            match union_data {
                Ok(data) => {
                    // println!(
                    //     "data type: {:?}, data len {}, data id {}",
                    //     data.0,
                    //     data.1.len(),
                    //     data.1[3]
                    // );
                    match data.0 {
                        PkgType::AHRS => unsafe {
                            let ahrs_pkg = std::mem::transmute::<[u8; 48], AHRS_PKG>(
                                data.1[7..7 + 48].try_into().unwrap(),
                            );
                            println!(
                                "{:?}",ahrs_pkg
                            );
                        },
                        PkgType::IMU => {}
                        _ => {}
                    }
                }
                Err(e) => {
                    print!("error: {:?}\n", e);
                }
            }
        }
    }
}

fn main() {
    println!("Hello, world!");
    let ser_port = SerialPort::open("COM3", 115200).unwrap();
    let (tx, rx) = mpsc::channel();
    let (tx2, rx2) = mpsc::channel();

    let mut reader = Data_Reciever::new(ser_port, tx);

    let read_handle = thread::spawn(move || {
        reader.run();
    });
    let check_handle = thread::spawn(move || {
        let mut parser = Data_Checker::new(rx, tx2);
        parser.run();
    });
    let mut parser = Data_parser::new(rx2);
    let mut parser_handle = thread::spawn(move || {
        parser.run();
    });

    read_handle.join().unwrap();
    check_handle.join().unwrap();
    parser_handle.join().unwrap();
}
