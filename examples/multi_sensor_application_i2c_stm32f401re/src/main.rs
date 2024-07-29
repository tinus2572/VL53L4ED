#![no_std]
#![no_main]

use vl53l4ed::{
    Vl53l4ed, 
    ResultsData
};

use panic_halt as _; 
use cortex_m_rt::entry;

use core::{fmt::Write, cell::RefCell};

use embedded_hal::i2c::SevenBitAddress;

use stm32f4xx_hal::{
    gpio::{
        Output, 
        Pin, 
        PinState::High,
        gpioa, 
        gpiob,
        Alternate}, 
    pac::{USART2, Peripherals, CorePeripherals, TIM1, TIM2, TIM3}, 
    prelude::*, 
    serial::{Config, Tx}, 
    timer::{Delay, SysDelay},
    rcc::{Rcc, Clocks}
};

// I2C related imports
use stm32f4xx_hal::{
    pac::I2C1,
    i2c::{I2c as StmI2c, I2c1, Mode}};
use embedded_hal_bus::i2c::RefCellDevice;

fn write_results_multi(tx: &mut Tx<USART2>, results_top: &ResultsData, results_left: &ResultsData, results_right: &ResultsData) {
    writeln!(tx, "\x1B[2H").unwrap();

    writeln!(tx, "VL53L4A3 Multi Sensor demo application\n").unwrap();
    
    writeln!(tx, "\nTop :\n").unwrap();
    writeln!(tx, 
        "\x1b[92mStatus = {sta:>4}\x1b[0m \x1b[96mDistance [mm] = {dis:>4}\x1b[0m \x1b[93mSignal [kcps/spad] = {sig:>4}\x1b[0m\r\n", 
        sta = results_top.range_status, 
        dis = results_top.distance_mm, 
        sig = results_top.signal_per_spad_kcps).unwrap();
        
    writeln!(tx, "\nLeft :\n").unwrap();
    writeln!(tx, 
        "\x1b[92mStatus = {sta:>4}\x1b[0m \x1b[96mDistance [mm] = {dis:>4}\x1b[0m \x1b[93mSignal [kcps/spad] = {sig:>4}\x1b[0m\r\n", 
        sta = results_left.range_status, 
        dis = results_left.distance_mm, 
        sig = results_left.signal_per_spad_kcps).unwrap();
    
    writeln!(tx, "\nRight :\n").unwrap();
    writeln!(tx, 
        "\x1b[92mStatus = {sta:>4}\x1b[0m \x1b[96mDistance [mm] = {dis:>4}\x1b[0m \x1b[93mSignal [kcps/spad] = {sig:>4}\x1b[0m\r\n", 
        sta = results_right.range_status, 
        dis = results_right.distance_mm, 
        sig = results_right.signal_per_spad_kcps).unwrap();
    
}


#[entry]
fn main() -> ! {
    let mut results_top: ResultsData;
    let mut results_left: ResultsData;
    let mut results_right: ResultsData;
    
    let dp: Peripherals = Peripherals::take().unwrap();
    let cp: CorePeripherals = CorePeripherals::take().unwrap();
    let rcc: Rcc = dp.RCC.constrain();
    let clocks: Clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();
    let _delay: SysDelay = cp.SYST.delay(&clocks);
    let tim_top: Delay<TIM1, 1000> = dp.TIM1.delay_ms(&clocks);
    let tim_left: Delay<TIM2, 1000> = dp.TIM2.delay_ms(&clocks);
    let tim_right: Delay<TIM3, 1000> = dp.TIM3.delay_ms(&clocks);

    let gpioa: gpioa::Parts = dp.GPIOA.split();
    let gpiob: gpiob::Parts = dp.GPIOB.split();
    
    let xshut_pin_top: Pin<'B', 3, Output> = gpiob.pb3.into_push_pull_output_in_state(High);
    let xshut_pin_left: Pin<'B', 10, Output> = gpiob.pb10.into_push_pull_output_in_state(High);
    let xshut_pin_right: Pin<'B', 5, Output> = gpiob.pb5.into_push_pull_output_in_state(High);
    
    let tx_pin: Pin<'A', 2, Alternate<7>> = gpioa.pa2.into_alternate();
     
    let mut tx: Tx<USART2> = dp.USART2.tx(
        tx_pin,
        Config::default()
        .baudrate(460800.bps())
        .wordlength_8()
        .parity_none(),
        &clocks).unwrap();
    
    let scl: Pin<'B', 8> = gpiob.pb8;
    let sda: Pin<'B', 9> = gpiob.pb9;
    
    let i2c: StmI2c<I2C1> = I2c1::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard{frequency:200.kHz()},
        &clocks);
        
    let i2c_bus: RefCell<StmI2c<I2C1>> = RefCell::new(i2c);
    let address_top: SevenBitAddress = 0x0;
    let address_left: SevenBitAddress = 0x1;
    let address_right: SevenBitAddress = 0x2;

    let i2c_top = RefCellDevice::new(&i2c_bus);
    let i2c_left = RefCellDevice::new(&i2c_bus);
    let i2c_right = RefCellDevice::new(&i2c_bus);
    
    let mut sensor_top = Vl53l4ed::new_i2c(
        i2c_top, 
        xshut_pin_top,
        tim_top).unwrap();
    let mut sensor_left = Vl53l4ed::new_i2c(
        i2c_left, 
        xshut_pin_left,
        tim_left).unwrap();
    let mut sensor_right = Vl53l4ed::new_i2c(
        i2c_right, 
        xshut_pin_right,
        tim_right).unwrap();

    sensor_top.off().unwrap();
    sensor_left.off().unwrap();
    sensor_right.off().unwrap();

    sensor_top.init_sensor(address_top).unwrap(); 
    sensor_left.init_sensor(address_left).unwrap(); 
    sensor_right.init_sensor(address_right).unwrap(); 

    sensor_top.set_range_timing(10, 0).unwrap();
    sensor_left.set_range_timing(10, 0).unwrap();
    sensor_right.set_range_timing(10, 0).unwrap();

    sensor_top.start_ranging().unwrap();
    sensor_left.start_ranging().unwrap();
    sensor_right.start_ranging().unwrap();
    
    loop {
        while !sensor_top.check_data_ready().unwrap() {} // Wait for data to be ready
        sensor_top.clear_interrupt().unwrap();
        results_top = sensor_top.get_ranging_data().unwrap(); // Get and parse the result data

        while !sensor_left.check_data_ready().unwrap() {} // Wait for data to be ready
        sensor_left.clear_interrupt().unwrap();
        results_left = sensor_left.get_ranging_data().unwrap(); // Get and parse the result data

        while !sensor_right.check_data_ready().unwrap() {} // Wait for data to be ready
        sensor_right.clear_interrupt().unwrap();
        results_right = sensor_right.get_ranging_data().unwrap(); // Get and parse the result data

        write_results_multi(&mut tx, &results_top, &results_left, &results_right); // Print the result to the output
    }

} 
