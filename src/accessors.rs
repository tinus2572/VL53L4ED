use consts::*;
use utils::*;

use crate::{consts, utils, BusOperation, Vl53l4ed, Error, OutputPin, DelayNs};


impl<B: BusOperation, XST: OutputPin, T: DelayNs> Vl53l4ed<B, XST, T> {
    
    /// This function sets new range timing. Timing are composed of TimingBudget and InterMeasurement. TimingBudget represents the timing during VCSEL enabled, and InterMeasurement the time between two measurements. The sensor can have different ranging mode depending of the configuration, please refer to the user manual for more information.
    /// 
    /// # Arguments
    /// 
    /// * `timing_budget_ms` :  New timing budget in ms. Value can be between 10ms and 200ms. Default is 50ms.
    /// * `inter_measurement_ms` :  New inter-measurement in ms. If the value is equal to 0, the ranging period is defined by the timing budget.Otherwise, inter-measurement must be > timing budget. When all the timing budget is consumed, the device goes in low power mode until inter-measurement is done.
    pub fn set_range_timing(&mut self, timing_budget_ms: u32, inter_measurement_ms: u32) -> Result<(), Error<B::Error>> {
        let mut osc_frequency: [u16; 1] = [0];
        let mut clock_pll: [u16; 1] = [0];
        let mut timing_budget_us: u32;
        let macro_period_us: u32;
        let mut inter_measurement_factor: f32 = 1.055;
        
        let mut tmp: [u8; 2] = [0;2];
        self.read_from_register(0x0006, &mut tmp)?;
        from_u8_to_u16(&tmp, &mut osc_frequency);
        if osc_frequency[0] != 0 {
            timing_budget_us = timing_budget_ms * 1000;
            macro_period_us = (2304 * (0x40000000 / osc_frequency[0] as u32)) >> 6;
        } else {
            return Err(Error::InvalidParam);
        }
  
        // Timing budget check validity 
        if timing_budget_ms < 10 || timing_budget_ms > 200 {
            return Err(Error::InvalidParam);
        } 

        // Sensor runs in continuous mode 
        if inter_measurement_ms == 0 {
            self.write_to_register(VL53L4ED_INTERMEASUREMENT_MS, &[0])?;
            timing_budget_us -= 2500;
        }

        // Sensor runs in autonomous low power mode 
        else if inter_measurement_ms > timing_budget_ms {
            self.read_from_register(VL53L4ED_RESULT_OSC_CALIBRATE_VAL, &mut tmp)?;
            from_u8_to_u16(&tmp, &mut clock_pll);
            clock_pll[0] &= 0x3ff;
            inter_measurement_factor *= inter_measurement_ms as f32 * clock_pll[0] as f32;
            let mut tmp: [u8; 4] = [0; 4];
            from_f32_to_u8(&[inter_measurement_factor], &mut tmp);
            self.write_to_register(VL53L4ED_INTERMEASUREMENT_MS, &tmp)?;
            timing_budget_us -= 4300;
            timing_budget_us /= 2;
        }

        // Invalid case
        else {
            return Err(Error::InvalidParam);
        }

        let mut ms_byte: u16 = 0;
        timing_budget_us = timing_budget_us << 12;
        let mut tmp: u32 = macro_period_us * 16;
        let mut ls_byte: u32 = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;

        while ls_byte & 0xFFFFFF00 > 0 {
            ls_byte = ls_byte >> 1;
            ms_byte += 1;
        }

        self.write_to_register(VL53L4ED_RANGE_CONFIG_A, &[ms_byte as u8, (ls_byte & 0xff) as u8])?;
        
        ms_byte = 0;
        tmp = macro_period_us * 12;
        ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;

        while ls_byte & 0xFFFFFF00 > 0 {
            ls_byte = ls_byte >> 1;
            ms_byte += 1;
        }

        self.write_to_register(VL53L4ED_RANGE_CONFIG_B, &[ms_byte as u8, (ls_byte & 0xff) as u8])?;
        Ok(())
    }

    /// This function gets the current range timing. Timing are composed of TimingBudget and InterMeasurement. TimingBudget represents the timing during VCSEL enabled, and InterMeasurement the time between two measurements. The sensor can have different ranging mode depending of the configuration, please refer to the user manual for more information.
    /// 
    /// # Return
    /// 
    /// * `[timing_budget_ms, inter_measurement_ms]` :  Array containing the current timing budget in ms and the current inter-measurement in ms.
    pub fn get_range_timing(&mut self) -> Result<[u32; 2], Error<B::Error>> {
        let mut timing_budget_ms: [u32; 1] = [0];
        let mut inter_measurement_ms: [u32; 1] = [0];

        let mut osc_frequency: [u16; 1] = [0];
        let mut clock_pll: [u16; 1] = [0];
        let mut range_config_macrop_high: [u16; 1] = [0];
        let mut macro_period_us: u32;
        let mut ms_byte: u32;
        let ls_byte: u32;
        let tmp: u32;
        let mut tmp16: [u8; 2] = [0; 2];
        let mut tmp32: [u8; 4] = [0; 4];
        let mut clock_pll_factor: f32 = 1.065;

        // Get InterMeasurement
        self.read_from_register(VL53L4ED_INTERMEASUREMENT_MS, &mut tmp32)?;
        from_u8_to_u32(&tmp32, &mut inter_measurement_ms);
        tmp = inter_measurement_ms[0];
        self.read_from_register(VL53L4ED_RESULT_OSC_CALIBRATE_VAL, &mut tmp16)?;
        from_u8_to_u16(&tmp16, &mut clock_pll);
        clock_pll[0] &= 0x3ff;
        clock_pll_factor *= clock_pll[0] as f32;
        clock_pll[0] = clock_pll_factor as u16;
        inter_measurement_ms[0] /= clock_pll[0] as u32;

        // Get TimingBudget
        self.read_from_register(0x0006, &mut tmp16)?;
        from_u8_to_u16(&tmp16, &mut osc_frequency);
        self.read_from_register(VL53L4ED_RANGE_CONFIG_A, &mut tmp16)?;
        from_u8_to_u16(&tmp16, &mut range_config_macrop_high);

        macro_period_us = (2304 * (0x40000000 / osc_frequency[0] as u32)) >> 6;
        ls_byte = (range_config_macrop_high[0] as u32 & 0x00ff) << 4;
        ms_byte = (range_config_macrop_high[0] as u32 & 0xff00) >> 8;
        ms_byte = 0x04 - (ms_byte - 1) - 1;

        macro_period_us *= 16;
        timing_budget_ms[0] = (((ls_byte + 1) * (macro_period_us >> 6)) - ((macro_period_us >> 6) >> 1)) >> 12;

        if ms_byte < 12 {
            timing_budget_ms[0] = timing_budget_ms[0] >> ms_byte; 
        }

        if tmp == 0 { // Mode continuous
            timing_budget_ms[0] += 2500;
        } else { // Mode autonomous
            timing_budget_ms[0] *= 2;
            timing_budget_ms[0] += 4300;
        }
        timing_budget_ms[0] /= 1000;

        let arr: [u32; 2] = [timing_budget_ms[0], inter_measurement_ms[0]];
        Ok(arr)
    }





}