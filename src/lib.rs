//! # VL53L4ED drivers and example applications
//! 
//! This crate provides a platform-agnostic driver for the ST VL53L4ED proximity sensor driver.
//! The [ST page](https://www.st.com/en/imaging-and-photonics-solutions/VL53L4ED.html) provide all necessary information.
//! This driver was built using the [embedded-hal](https://docs.rs/embedded-hal/latest/embedded_hal/) traits.
//! The [stm32f4xx-hal](https://docs.rs/stm32f4xx-hal/latest/stm32f4xx_hal/) crate is also mandatory.
//! Ensure that the hardware abstraction layer of your microcontroller implements the embedded-hal traits.
//! 
//! ## Instantiating
//! 
//! Create an instance of the driver with the `new_i2c` associated function, by passing i2c and address.
//!  
//! ### Setup:
//! ```rust
//! let dp = Peripherals::take().unwrap();
//! let rcc = dp.RCC.constrain();
//! let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();
//! let tim_top = dp.TIM1.delay_ms(&clocks);
//! 
//! let gpioa = dp.GPIOA.split();
//! let gpiob = dp.GPIOB.split();
//! 
//! let xshut_pin = gpiob.pb3.into_push_pull_output_in_state(High);
//! let tx_pin = gpioa.pa2.into_alternate();
//!     
//! let mut tx = dp.USART2.tx(
    //! tx_pin,
    //! Config::default()
    //! .baudrate(460800.bps())
    //! .wordlength_8()
    //! .parity_none(),
    //! &clocks).unwrap();
//! 
//! let scl = gpiob.pb8;
//! let sda = gpiob.pb9;
//! 
//! let i2c = I2c1::new(
    //! dp.I2C1,
    //! (scl, sda),
    //! Mode::Standard{frequency:200.kHz()},
    //! &clocks);
//!     
//! let i2c_bus = RefCell::new(i2c);
//! let address = VL53L4ED_DEFAULT_I2C_ADDRESS;
//!     
//! let mut sensor_top = Vl53l4ed::new_i2c(
    //! RefCellDevice::new(&i2c_bus), 
//!         xshut_pin,
//!         tim_top
    //! ).unwrap();
//! 
//! sensor_top.init_sensor(address).unwrap(); 
//! sensor_top.start_ranging().unwrap();
//! ```
//! 
//! ### Loop:
//! ```rust
//! loop {
    //! while !sensor_top.check_data_ready().unwrap() {} // Wait for data to be ready
    //! let results = sensor_top.get_ranging_data().unwrap(); // Get and parse the result data
    //! sensor_top.clear_interrupt().unwrap(); // Clear HW interrupt to restart measurements
    //! write_results(&mut tx, &results); // Print the result to the output
//! }    
//! ```
//! 
//! ## Multiple instances with I2C

//! The default I2C address for this device (cf. datasheet) is 0x52.

//! If multiple sensors are used on the same I2C bus, consider setting off all the instances, then initializating them one by one to set up unique I2C addresses.

//! ```rust
//! sensor_top.off().unwrap();
//! sensor_left.off().unwrap();
//! sensor_right.off().unwrap();
//! 
//! sensor_top.init_sensor(address_top).unwrap(); 
//! sensor_left.init_sensor(address_left).unwrap(); 
//! sensor_right.init_sensor(address_right).unwrap(); 
//! ```


#![no_std]
#![allow(dead_code)]
#![allow(unused_imports)]

pub mod accessors;
pub mod buffers;
pub mod bus_operation;
pub mod consts;
pub mod utils;
pub mod calibration;

use accessors::*;
use buffers::*;
use bus_operation::*;
use consts::*;
use utils::*;
use calibration::*;

use embedded_hal::{
    i2c::{I2c, SevenBitAddress},
    digital::OutputPin, 
    delay::DelayNs
};

pub struct Vl53l4ed<B: BusOperation, XST: OutputPin, T: DelayNs> {
    pub(crate) xshut_pin: XST,
    pub(crate) bus: B,
    pub(crate) tim: T,
    pub(crate) chunk_size: usize,
}

#[derive(Copy, Clone, Debug)]
pub enum Error<B> {
    Bus(B),
    Other,
    Timeout,
    Mcu,
    Go2,
    CorruptedFrame,
    InvalidParam,
    CheckSumFail
}

/// Packed reading results type
#[repr(C)]
pub struct ResultsData {
    // Status of measurements. If the status is equal to 0, the data are valid
    pub range_status: u8,
    // Measured distance in mm 
    pub distance_mm: u16,
    // Ambient noise in kcps 
    pub ambient_rate_kcps: u16,
    // Ambient noise in kcps/SPAD 
    pub ambient_per_spad_kcps: u16,
    // Measured signal of the target in kcps 
    pub signal_rate_kcps: u16,
    // Measured signal of the target in kcps/SPAD 
    pub signal_per_spad_kcps: u16,
    // Number of SPADs enabled 
    pub number_of_spad: u16,
    // Estimated measurements std deviation in mm 
    pub sigma_mm: u16,
} 

impl ResultsData {
    pub fn new() -> Self {
        ResultsData {
            range_status: 0,
            distance_mm: 0,
            ambient_per_spad_kcps: 0,
            ambient_rate_kcps: 0,
            signal_rate_kcps: 0,
            signal_per_spad_kcps: 0,
            number_of_spad: 0,
            sigma_mm: 0,
        }
    }
}

impl<B: BusOperation, XST: OutputPin, T: DelayNs> Vl53l4ed<B, XST, T> {
    pub(crate) fn read_u8(&mut self, reg: u16) -> Result<u8, Error<B::Error>> {
        let mut rbuf: [u8; 1] = [0];
        self.read_from_register(reg, &mut rbuf)?;
        Ok(rbuf[0])
    }
    
    pub(crate) fn read_u16(&mut self, reg: u16) -> Result<u16, Error<B::Error>> {
        let mut rbuf: [u8; 2] = [0; 2];
        self.read_from_register(reg, &mut rbuf)?;
        let mut val: [u16; 1] = [0];
        from_u8_to_u16(&rbuf, &mut val);
        Ok(val[0])
    } 
    
    pub(crate) fn read_u32(&mut self, reg: u16) -> Result<u32, Error<B::Error>> {
        let mut rbuf: [u8; 4] = [0; 4];
        self.read_from_register(reg, &mut rbuf)?;
        let mut val: [u32; 1] = [0];
        from_u8_to_u32(&rbuf, &mut val);
        Ok(val[0])
    } 

    pub(crate) fn write_u8(&mut self, reg: u16, val: u8) -> Result<(), Error<B::Error>> {
        self.write_to_register(reg, &[val])?;
        Ok(())
    }

    pub(crate) fn write_u16(&mut self, reg: u16, val: u16) -> Result<(), Error<B::Error>> {
        let mut wbuf: [u8; 2] = [0; 2];
        from_u16_to_u8(&[val], &mut wbuf);
        self.write_to_register(reg, &wbuf)?;
        Ok(())
    }

    pub(crate) fn write_u32(&mut self, reg: u16, val: u32) -> Result<(), Error<B::Error>> {
        let mut wbuf: [u8; 4] = [0; 4];
        from_u32_to_u8(&[val], &mut wbuf);
        self.write_to_register(reg, &wbuf)?;
        Ok(())
    }

    /// Utility function to read data.
    /// * Enough bytes of data are read starting from `reg`
    /// to fill `rbuf`.
    /// 
    /// # Arguments
    /// 
    /// * `reg` : specifies internal address register to be read.
    /// * `rbuf` : array to fill with read bytes.
    pub(crate) fn read_from_register(&mut self, reg: u16, rbuf: &mut [u8]) -> Result<(), Error<B::Error>> {
        let size = rbuf.len();
        let mut read_size: usize;
        for i in (0..size).step_by(self.chunk_size) {
            read_size = if size - i > self.chunk_size { self.chunk_size } else { size - i };
            let a: u8 = (reg + i as u16 >> 8) as u8;
            let b: u8 = (reg + i as u16 & 0xFF) as u8; 
            self.bus.write_read(&[a, b], &mut rbuf[i..i+read_size]).map_err(Error::Bus)?;
        }
        Ok(())
    }

    /// Utility function to write data.
    /// The content of `wbuf` is written in registers starting from `reg`.
    /// 
    /// # Arguments
    /// 
    /// * `reg` : specifies internal address register to be overwritten.
    /// * `wbuf` : values to be written.
    pub(crate) fn write_to_register(&mut self, reg: u16, wbuf: &[u8]) -> Result<(), Error<B::Error>> {
        let size = wbuf.len();
        let mut write_size: usize;
        let mut tmp: [u8; 32] = [0; 32];
        for i in (0..size).step_by(self.chunk_size-2) {
            write_size = if size - i > self.chunk_size-2 { self.chunk_size-2 } else { size - i };
            tmp[0] = (reg + i as u16 >> 8) as u8;
            tmp[1] = (reg + i as u16 & 0xFF) as u8;
            tmp[2..2+write_size].copy_from_slice(&wbuf[i..i+write_size]);
            self.bus.write(&tmp[..2+write_size]).map_err(Error::Bus)?;    
        }   
        Ok(())
    }

    /// Utility function to wait.
    /// 
    /// # Arguments
    /// 
    /// * `ms` : milliseconds to wait.
    pub(crate) fn delay(&mut self, ms: u32) {
        self.tim.delay_ms(ms);
    }

    /// PowerOn the sensor
    pub fn on(&mut self) -> Result<(), Error<B::Error>>{
        self.xshut_pin.set_high().unwrap();
        self.delay(10);
        Ok(())
    }

    /// PowerOff the sensor
    pub fn off(&mut self) -> Result<(), Error<B::Error>>{
        self.xshut_pin.set_low().unwrap();
        self.delay(10);
        Ok(())
    }
    
    /// This function is used to check the sensor id of VL53L4ED. The sensor id should be 0xECAA.`
    pub fn is_alive(&mut self) -> Result<(), Error<B::Error>> {
        let sensor_id: u16 = self.read_u16(VL53L4ED_IDENTIFICATION_MODEL_ID)?;
        if sensor_id != 0xECAA {
            return Err(Error::Other);
        }
        Ok(())
    }
    
    /// This function is used to initialize the sensor.
    pub fn init(&mut self) -> Result<(), Error<B::Error>> {
        let mut tmp: u8;
        let mut i: u16 = 0;
        loop {
            tmp = self.read_u8(VL53L4ED_FIRMWARE_SYSTEM_STATUS)?;
            if tmp == 0x3 { // Sensor booted
                break;
            } else if i >= 1000 {
                return Err(Error::Timeout);
            }
            i += 1;
            self.delay(1);
        }

        // Load default configuration
        self.write_to_register(0x2d, &VL53L4ED_DEFAULT_CONFIGURATION)?;

        // Start VHV
        self.write_u8(VL53L4ED_SYSTEM_START,0x40)?;
        i = 0;
        loop {
            if self.check_data_ready()? {
                break;
            } else if i >= 1000 {
                return Err(Error::Timeout);
            }
            i += 1;
            self.delay(1);
        }

        self.clear_interrupt()?;
        self.stop_ranging()?;
        self.write_u8(VL53L4ED_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)?;
        self.write_u8(0x0b, 0)?;
        self.write_u16(0x0024, 0x500)?;
        self.set_range_timing(100, 0)?;

        Ok(())
    }

    /// This function clears the interrupt. It needs to be called after a ranging data reading to arm the interrupt for the next data ready event.
    pub fn clear_interrupt(&mut self) -> Result<(), Error<B::Error>> {
        self.write_u8(VL53L4ED_SYSTEM_INTERRUPT_CLEAR, 0x01)?;

        Ok(())
    }

    /// This function starts a ranging session. The ranging operation is continuous. The clear interrupt has to be done after each get data to allow the interrupt to raise when the next data is ready.
    pub fn start_ranging(&mut self) -> Result<(), Error<B::Error>> {
        let tmp: u32 = self.read_u32(VL53L4ED_INTERMEASUREMENT_MS)?;
        if tmp == 0 {
            // Sensor runs in continuous mode 
            self.write_u8(VL53L4ED_SYSTEM_START, 0x21)?;
        } else {
            // Sensor runs in autonomous mode 
            self.write_u8(VL53L4ED_SYSTEM_START, 0x40)?;
        }

        Ok(())
    }

    /// This function stops the ranging session. 
    /// It must be used when the sensor streams, after calling start_ranging().
    pub fn stop_ranging(&mut self) -> Result<(), Error<B::Error>> {
        self.write_u8(VL53L4ED_SYSTEM_START, 0x00)?;

        Ok(())
    }
    
    /// This function checks if a new data is ready by polling I2C. 
    /// If a new data is ready, a flag will be raised.
    /// 
    /// # Return
    /// 
    /// * `isReady` : Value is false if data is not ready, 
    /// or true if a new data is ready.
    pub fn check_data_ready(&mut self) -> Result<bool, Error<B::Error>> {
        let mut tmp: u8 = self.read_u8(VL53L4ED_GPIO_HV_MUX_CTRL)?;
        tmp = (tmp & 0x10) >> 4;
        let int_pol: u8 = if tmp == 1 { 0 } else { 1 };
        tmp = self.read_u8(VL53L4ED_GPIO_TIO_HV_STATUS)?;
        let is_ready: bool = tmp & 1 == int_pol;
    
        Ok(is_ready)
    }

    /// This function gets the ranging data, 
    /// using the selected output and the resolution.
    /// 
    /// # Return
    /// 
    /// * `results` : VL53L7 results structure.
    pub fn get_ranging_data(&mut self) -> Result<ResultsData, Error<B::Error>> {
        let mut result: ResultsData = ResultsData::new();
        let status_rtn: [u8; 24] = [
            255, 255, 255,   5, 
              2,   4,   1,   7, 
              3,   0, 255, 255, 
              9,  13, 255, 255, 
            255, 255,  10,   6,
            255, 255,  11,  12];

        let mut tmp = self.read_u8(VL53L4ED_RESULT_RANGE_STATUS)?;
        tmp &= 0x1f;
        if tmp < 24 {
            tmp = status_rtn[tmp as usize];
        }
        result.range_status = tmp;

        let mut tmp = self.read_u16(VL53L4ED_RESULT_SPAD_NB)?;
        result.number_of_spad = tmp / 256; 
        
        tmp = self.read_u16(VL53L4ED_RESULT_SIGNAL_RATE)?;
        result.signal_rate_kcps = tmp * 8;
        
        tmp = self.read_u16(VL53L4ED_RESULT_AMBIENT_RATE)?;
        result.ambient_rate_kcps = tmp * 8;

        tmp = self.read_u16(VL53L4ED_RESULT_SIGMA)?;
        result.sigma_mm = tmp / 4;

        tmp = self.read_u16(VL53L4ED_RESULT_DISTANCE)?;
        result.distance_mm = tmp;

        result.signal_per_spad_kcps = result.signal_rate_kcps / result.number_of_spad;
        result.ambient_per_spad_kcps = result.ambient_rate_kcps / result.number_of_spad;

        Ok(result)
    }
}