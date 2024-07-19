use core::iter::OnceWith;

use consts::*;
use stm32f4xx_hal::{pac::tim11::or::OR_SPEC, rtc::Lse};
use utils::*;

use crate::{consts, utils, BusOperation, Vl53l4ed, Error, OutputPin, DelayNs};

#[repr(u8)]
pub enum ThresholdWindow {
    Below = 0,
    Above = 1,
    Out = 2,
    In = 3
}
pub struct DetectionThresholds{
    pub distance_high_mm: u16, 
    pub distance_low_mm: u16, 
    pub window: ThresholdWindow
}

impl DetectionThresholds {
    pub fn new() -> Self {
        DetectionThresholds { 
            distance_high_mm: 0,
            distance_low_mm: 0,
            window: ThresholdWindow::Below
        }
    }
}

impl<B: BusOperation, XST: OutputPin, T: DelayNs> Vl53l4ed<B, XST, T> {

    /// This function gets the current range timing. Timing are composed of TimingBudget and InterMeasurement. TimingBudget represents the timing during VCSEL enabled, and InterMeasurement the time between two measurements. The sensor can have different ranging mode depending of the configuration, please refer to the user manual for more information.
    /// 
    /// # Return
    /// 
    /// * `[timing_budget_ms, inter_measurement_ms]` :  Array containing the current timing budget in ms and the current inter-measurement in ms.
    pub fn get_range_timing(&mut self) -> Result<[u32; 2], Error<B::Error>> {
        let mut timing_budget_ms: u32;
        let mut inter_measurement_ms: u32;

        let osc_frequency: u16;
        let mut clock_pll: u16;
        let range_config_macrop_high: u16;
        let mut macro_period_us: u32;
        let mut ms_byte: u32;
        let ls_byte: u32;
        let tmp: u32;
        let mut clock_pll_factor: f32 = 1.065;

        // Get InterMeasurement
        inter_measurement_ms = self.read_u32(VL53L4ED_INTERMEASUREMENT_MS)?;
        tmp = inter_measurement_ms;
        clock_pll = self.read_u16(VL53L4ED_RESULT_OSC_CALIBRATE_VAL)?;
        clock_pll &= 0x3ff;
        clock_pll_factor *= clock_pll as f32;
        clock_pll = clock_pll_factor as u16;
        inter_measurement_ms /= clock_pll as u32;

        // Get TimingBudget
        osc_frequency = self.read_u16(0x0006)?;
        range_config_macrop_high = self.read_u16(VL53L4ED_RANGE_CONFIG_A)?;

        macro_period_us = (2304 * (0x40000000 / osc_frequency as u32)) >> 6;
        ls_byte = (range_config_macrop_high as u32 & 0x00ff) << 4;
        ms_byte = (range_config_macrop_high as u32 & 0xff00) >> 8;
        ms_byte = 0x04 - ms_byte; // 0x04 - (ms_byte - 1) - 1;

        macro_period_us *= 16;
        timing_budget_ms = (((ls_byte + 1) * (macro_period_us >> 6)) - ((macro_period_us >> 6) >> 1)) >> 12;

        if ms_byte < 12 {
            timing_budget_ms = timing_budget_ms >> ms_byte; 
        }

        if tmp == 0 { // Mode continuous
            timing_budget_ms += 2500;
        } else { // Mode autonomous
            timing_budget_ms *= 2;
            timing_budget_ms += 4300;
        }
        timing_budget_ms /= 1000;

        let arr: [u32; 2] = [timing_budget_ms, inter_measurement_ms];
        Ok(arr)
    }

    /// This function sets new range timing. Timing are composed of TimingBudget and InterMeasurement. TimingBudget represents the timing during VCSEL enabled, and InterMeasurement the time between two measurements. The sensor can have different ranging mode depending of the configuration, please refer to the user manual for more information.
    /// 
    /// # Arguments
    /// 
    /// * `timing_budget_ms` :  New timing budget in ms. Value can be between 10ms and 200ms. Default is 50ms.
    /// * `inter_measurement_ms` :  New inter-measurement in ms. If the value is equal to 0, the ranging period is defined by the timing budget.Otherwise, inter-measurement must be > timing budget. When all the timing budget is consumed, the device goes in low power mode until inter-measurement is done.
    pub fn set_range_timing(&mut self, timing_budget_ms: u32, inter_measurement_ms: u32) -> Result<(), Error<B::Error>> {
        let osc_frequency: u16;
        let mut clock_pll: u16;
        let mut timing_budget_us: u32;
        let macro_period_us: u32;
        let mut inter_measurement_factor: f32 = 1.055;
        let mut ls_byte: u32;
        let mut ms_byte: u16;
        let mut tmp: u32;
        
        osc_frequency = self.read_u16(0x0006)?;
        if osc_frequency != 0 {
            timing_budget_us = timing_budget_ms * 1000;
            macro_period_us = (2304 * (0x40000000 / osc_frequency as u32)) >> 6;
        } else {
            return Err(Error::InvalidParam);
        }
  
        // Timing budget check validity 
        if timing_budget_ms < 10 || timing_budget_ms > 200 {
            return Err(Error::InvalidParam);
        } 

        // Sensor runs in continuous mode 
        if inter_measurement_ms == 0 {
            self.write_u32(VL53L4ED_INTERMEASUREMENT_MS, 0)?;
            timing_budget_us -= 2500;
        }

        // Sensor runs in autonomous low power mode 
        else if inter_measurement_ms > timing_budget_ms {
            clock_pll = self.read_u16(VL53L4ED_RESULT_OSC_CALIBRATE_VAL)?;
            clock_pll &= 0x3ff;
            inter_measurement_factor *= inter_measurement_ms as f32;
            inter_measurement_factor *= clock_pll as f32;
            self.write_u32(VL53L4ED_INTERMEASUREMENT_MS, inter_measurement_factor as u32)?;
            timing_budget_us -= 4300;
            timing_budget_us /= 2;
        }

        // Invalid case
        else {
            return Err(Error::InvalidParam);
        }

        ms_byte = 0;
        timing_budget_us = timing_budget_us << 12;
        tmp = macro_period_us * 16;
        ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;

        while ls_byte & 0xFFFFFF00 > 0 {
            ls_byte = ls_byte >> 1;
            ms_byte += 1;
        }

        ms_byte = (ms_byte << 8) + (ls_byte & 0xff) as u16;
        self.write_u16(VL53L4ED_RANGE_CONFIG_A, ms_byte)?;
        
        ms_byte = 0;
        tmp = macro_period_us * 12;
        ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;

        while ls_byte & 0xFFFFFF00 > 0 {
            ls_byte = ls_byte >> 1;
            ms_byte += 1;
        }

        ms_byte = (ms_byte << 8) + (ls_byte & 0xff) as u16;
        self.write_u16(VL53L4ED_RANGE_CONFIG_B, ms_byte)?;

        Ok(())
    }

    /// This function gets the current offset correction in mm. Offset  corresponds to the difference in millimeters between real distance and measured distance.
    /// 
    /// # Return
    /// 
    /// * `offset` :  Offset value in millimeters. The minimum value is -1024mm and maximum is 1023mm.
    pub fn get_offset(&mut self) -> Result<i16, Error<B::Error>> {
        let mut tmp = self.read_u16(VL53L4ED_RANGE_OFFSET_MM)?;
        tmp = (tmp << 3) >> 5;
        let mut offset = tmp as i16;
        if offset > 1024 {
            offset -= 2048;
        }
        Ok(offset)
    }

    /// This function sets a new offset correction in mm. Offset corresponds to the difference in millimeters between real distance and measured distance.
    /// 
    /// # Arguments
    /// 
    /// * `offset` :  Offset value in millimeters. The minimum value is -1024mm and maximum is 1023mm.
    pub fn set_offset(&mut self, offset: i16) -> Result<(), Error<B::Error>> {
        self.write_u16(VL53L4ED_RANGE_OFFSET_MM, (offset * 4) as u16)?;
        self.write_u16(VL53L4ED_INNER_OFFSET_MM, 0)?;
        self.write_u16(VL53L4ED_OUTER_OFFSET_MM, 0)?;

        Ok(())
    }

    /// This function gets the current Xtalk value in kcps. Xtalk represents the correction to apply to the sensor when a protective coverglass is placed at the top of the sensor.
    /// 
    /// # Returns
    /// 
    /// * `xtalk` : Current xtalk value in kcps. 
    pub fn get_xtalk(&mut self) -> Result<u16, Error<B::Error>> {
        let mut offset: u16 = self.read_u16(VL53L4ED_XTALK_PLANE_OFFSET_KCPS)?;
        offset = (offset as f32 / 512.0) as u16;
        
        Ok(offset)
    }

    /// This function sets a new Xtalk value in kcps. Xtalk represents the correction to apply to the sensor when a protective coverglass is placed at the top of the sensor.
    /// 
    /// # Arguments
    /// 
    /// * `xtalk` : New xtalk value in kcps. The default value is 0 kcps (no coverglass). Minimum is 0 kcps , and maximum is 128 kcps.
    pub fn set_xtalk(&mut self, xtalk: u16) -> Result<(), Error<B::Error>> {
        self.write_u16(VL53L4ED_XTALK_X_PLANE_GRADIENT_KCPS, 0)?;
        self.write_u16(VL53L4ED_XTALK_Y_PLANE_GRADIENT_KCPS, 0)?;
        self.write_u16(VL53L4ED_XTALK_PLANE_OFFSET_KCPS, xtalk << 9)?;
        Ok(())
    }

    /// This function gets the current detection thresholds. The detection thresholds can be programmed to generate an interrupt on pin 7 (GPIO1), only when a condition on distance is reach. 
    /// Interrupt windows : 
    /// * 0 = below low threshold
    /// * 1 = above high threshold 
    /// * 2 = out of low/high windows
    /// * 3 = in low/high windows
    /// 
    /// # Return
    /// 
    /// * `thresholds` : Low/High distance threshold in millimeters and  Interrupt window
    pub fn get_detection_thresholds(&mut self) -> Result<DetectionThresholds, Error<B::Error>> {
        let mut thresholds: DetectionThresholds = DetectionThresholds::new();
        thresholds.distance_high_mm = self.read_u16(VL53L4ED_THRESH_HIGH)?;
        thresholds.distance_low_mm = self.read_u16(VL53L4ED_THRESH_LOW)?;
        thresholds.window =  match self.read_u8(VL53L4ED_SYSTEM_INTERRUPT)? {
            0 => ThresholdWindow::Below,
            1 => ThresholdWindow::Above,
            2 => ThresholdWindow::Out,
            3 => ThresholdWindow::In,
            _ => return Err(Error::Other)
        };

        Ok(thresholds)
    }

    /// This function sets new detection thresholds. The detection thresholds can be programmed to generate an interrupt on pin 7 (GPIO1), only when a condition on distance is reach. 
    /// Interrupt windows : 
    /// * 0 = below low threshold
    /// * 1 = above high threshold 
    /// * 2 = out of low/high windows
    /// * 3 = in low/high windows
    /// 
    /// # Arguments
    /// 
    /// * `thresholds` : Low/High distance threshold in millimeters and  Interrupt window
    pub fn set_detection_thresholds(&mut self, thresholds: DetectionThresholds) -> Result<(), Error<B::Error>> {
        self.write_u16(VL53L4ED_THRESH_HIGH, thresholds.distance_high_mm)?;
        self.write_u16(VL53L4ED_THRESH_LOW, thresholds.distance_low_mm)?;
        self.write_u8(VL53L4ED_SYSTEM_INTERRUPT, thresholds.window as u8)?;

        Ok(())
    }

    
    /// This function returns the current signal threshold in kcps. If a target has a lower signal as the programmed value, the result status in structure 'VL53L4ED_ResultsData_t' will be equal to 2.
    /// 
    /// Return
    /// 
    /// * `signal_kcps` : Pointer of signal threshold in kcps.
    pub fn get_signal_thresholds(&mut self) -> Result<u16, Error<B::Error>> {
        let mut signal_kcps = self.read_u16(VL53L4ED_MIN_COUNT_RATE_RTN_LIMIT_MCPS)?;
        signal_kcps = signal_kcps << 3;
        Ok(signal_kcps)
    }

    /// This function sets a new signal threshold in kcps. If a target has a lower signal as the programmed value, the result status in structure 'VL53L4ED_ResultsData_t' will be equal to 2.
    /// 
    /// # Arguments
    /// 
    /// * `signal_kcps` : New signal threshold in kcps. The default value is 1024 kcps. Minimum is 0 kcps (no threshold), and maximum is 16384 kcps.
    pub fn set_signal_thresholds(&mut self, signal_kcps: u16) -> Result<(), Error<B::Error>> {
        self.write_u16(VL53L4ED_MIN_COUNT_RATE_RTN_LIMIT_MCPS, signal_kcps >> 3)?;
        Ok(())
    }

    /// This function gets the current sigma threshold. The sigma corresponds to the standard deviation of the returned pulse. If the computed sigma is above the programmed value, the result status in structure 'VL53L4ED_ResultsData_t' will be equal to 1.
    /// 
    /// Return
    /// 
    /// * `sigma_mm` : Current sigma threshold in mm.
    pub fn get_sigma_thresholds(&mut self) -> Result<u16, Error<B::Error>> {
        let mut sigma_mm = self.read_u16(VL53L4ED_RANGE_CONFIG_SIGMA_THRESH)?;
        sigma_mm = sigma_mm >> 2;
        Ok(sigma_mm)
    }

    /// This function programs a new sigma threshold. The sigma corresponds to the standard deviation of the returned pulse. If the computed sigma is above the programmed value, the result status in structure 'VL53L4ED_ResultsData_t' will be equal to 1.
    /// 
    /// # Arguments
    /// 
    /// * `sigma_mm` : New sigma threshold in mm. The default value is 15mm. Minimum is 0mm (not threshold), and maximum is 16383mm.
    pub fn set_sigma_thresholds(&mut self, sigma_mm: u16) -> Result<(), Error<B::Error>> {
        if sigma_mm > 0xfff >> 2 {
            return Err(Error::InvalidParam);
        }
        self.write_u16(VL53L4ED_RANGE_CONFIG_SIGMA_THRESH, sigma_mm << 2)?;

        Ok(())
    }

    /// This function can be called when the temperature might have changed by more than 8 degrees Celsius. The function can only be used if the sensor is not ranging, otherwise, the ranging needs to be stopped using function 'stop_ranging()'. After calling this function, the ranging can restart normally.
    pub fn start_temperature_update(&mut self) -> Result<(), Error<B::Error>> {
        self.write_u8(VL53L4ED_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x81)?;
        self.write_u8(0x0b, 0x92)?;
        // self.write_u8(VL53L4ED_SYSTEM_START, 0x40)?;
        self.start_ranging()?;
        let mut i: u16 = 0;
        loop {
            if self.check_data_ready()? { // Data ready
                break;
            }
            if i >= 1000 {
                return Err(Error::Timeout);
            }
            self.delay(1);
            i += 1;
        }
        self.clear_interrupt()?;
        self.stop_ranging()?;
        self.write_u8(VL53L4ED_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)?;
        self.write_u8(0x0b, 0x0)?;
        
        Ok(())
    }
}