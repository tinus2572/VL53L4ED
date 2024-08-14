#![no_std]
#![no_main]

use vl53l4ed::{
    consts::VL53L4ED_DEFAULT_I2C_ADDRESS,
    Vl53l4ed, 
    ResultsData,
    accessors::{RangeTiming, TimingBudgetMs, Bounded},
};

use panic_halt as _; 
use cortex_m_rt::entry;
use embedded_alloc::Heap;

use core::{fmt::Write, cell::RefCell, mem::MaybeUninit};

use embedded_hal::i2c::SevenBitAddress;

use stm32f4xx_hal::{
    gpio::{
        Output, 
        Pin, 
        PinState::High,
        gpioa, 
        gpiob,
        Alternate}, 
    pac::{USART2, Peripherals, CorePeripherals, TIM1}, 
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

fn write_results(tx: &mut Tx<USART2>, results: &ResultsData) {

    writeln!(tx, "\x1B[2H").unwrap();

    writeln!(tx, "VL53L4A3 Simple Ranging demo application\n").unwrap();
    writeln!(tx, "Status = {sta:>4}\r\n", 
        sta = results.range_status).unwrap();
    writeln!(tx, "Distance [mm] = {dis:>4}\r\n", 
        dis = results.distance_mm).unwrap();
    writeln!(tx, "Signal [kcps/spad] = {sig:>4}\r\n", 
        sig = results.signal_per_spad_kcps).unwrap();
}



#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 1024;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

#[entry]
fn main() -> ! {
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }

    let mut results: ResultsData;
    
    let dp: Peripherals = Peripherals::take().unwrap();
    let cp: CorePeripherals = CorePeripherals::take().unwrap();
    let rcc: Rcc = dp.RCC.constrain();
    let clocks: Clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();
    let _delay: SysDelay = cp.SYST.delay(&clocks);
    let tim_top: Delay<TIM1, 1000> = dp.TIM1.delay_ms(&clocks);


    let gpioa: gpioa::Parts = dp.GPIOA.split();
    let gpiob: gpiob::Parts = dp.GPIOB.split();
    
    let xshut_pin: Pin<'B', 3, Output> = gpiob.pb3.into_push_pull_output_in_state(High);
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
    let address: SevenBitAddress = VL53L4ED_DEFAULT_I2C_ADDRESS;
        
    let sensor_top = Vl53l4ed::new_i2c(
        RefCellDevice::new(&i2c_bus),  
            xshut_pin,
            tim_top
        ).unwrap();

    let mut sensor_top = sensor_top.init_sensor(address).unwrap(); 
    
    let t: TimingBudgetMs = TimingBudgetMs::new(10).expect("");
    let r: RangeTiming = RangeTiming {timing_budget_ms: t, inter_measurement_ms: 0};

    sensor_top.set_range_timing(r).unwrap();
    let mut sensor_top = sensor_top.start_ranging().unwrap();

    loop {
        while !sensor_top.check_data_ready().unwrap() {} // Wait for data to be ready
        sensor_top.clear_interrupt().unwrap();
        results = sensor_top.get_ranging_data().unwrap(); // Get and parse the result data
        write_results(&mut tx, &results); // Print the result to the output
    }

} 

