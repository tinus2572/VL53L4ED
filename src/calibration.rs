use core::result;

use consts::*;
use stm32f4xx_hal::{hal_02::can::nb, pac::sdio::resp2};
use utils::*;

use crate::{consts, utils, BusOperation, DelayNs, Error, OutputPin, ResultsData, Vl53l4ed};

impl<B: BusOperation, XST: OutputPin, T: DelayNs> Vl53l4ed<B, XST, T> {
    /// This function can be used to perform an offset calibration. ST recommend to perform offset at 100m, on a grey17% reflective target, but any other distance and reflectance can be used. The function returns the offset value found and programs the offset compensation into the device.
    /// 
    /// # Arguments
    /// 
    /// * `target_dist` : Real distance between the sensor and the target in millimeters. ST recommend 100mm. Min distance is 10mm and max is 1000mm.
    /// * `nb_samples` : Number of samples (between 5 and 255). A higher number of samples increases the accuracy, but it also takes more time. ST recommend to use at least 10 samples.
    /// 
    /// # Returns
    /// 
    /// * `measured_offset` : The difference in millimeters between real distance and measured distance.
    pub fn calibration_offset(&mut self, target_dist: i16, nb_samples: i16) -> Result<i16, Error<B::Error>> {
        let mut results: ResultsData;
        let mut avg_dist: i16 = 0;
        let measured_offset: i16;

        if nb_samples < 5 || nb_samples > 255 || target_dist < 10 || target_dist > 1000 {
            return Err(Error::InvalidParam);
        }

        self.write_u16(VL53L4ED_RANGE_OFFSET_MM, 0)?;
        self.write_u16(VL53L4ED_INNER_OFFSET_MM, 0)?;
        self.write_u16(VL53L4ED_OUTER_OFFSET_MM, 0)?;

        // Device heat loop (10 samples)
        self.start_ranging()?;
        for _ in 0..10 {
            while !self.check_data_ready()? {}
            self.get_ranging_data()?;
            self.clear_interrupt()?;
        }
        self.stop_ranging()?;

        // Device ranging
        self.start_ranging()?;
        for _ in 0..nb_samples {
            while !self.check_data_ready()? {}
            results = self.get_ranging_data()?;
            self.clear_interrupt()?;
            avg_dist += results.distance_mm as i16;
        }
        self.stop_ranging()?;
        avg_dist /= nb_samples;
        measured_offset = target_dist - avg_dist;
        self.write_u16(VL53L4ED_RANGE_OFFSET_MM, (measured_offset * 4) as u16)?;

        Ok(measured_offset)
    }

    /// This function can be used to perform a Xtalk calibration. The distance for calibration depends of the coverglass, it needs to be characterized. Please refer to the User Manual for more information. The function returns the Xtalk value found and programs the Xtalk compensation into the device.
    /// 
    /// # Arguments
    /// 
    /// * `target_dist` : Real distance between the sensor and the target in millimeters. This distance needs to be characterized, as described into the User Manual.
    /// * `nb_samples` : Number of samples (between 5 and 255). A higher number of samples increases the accuracy, but it also takes more time. ST recommend to use at least 10 samples.
    ///    
    /// # Returns
    /// 
    /// * `measured_xtalk` : The correction to apply to the sensor when a protective coverglass is placed at the top of the sensor.
    pub fn calibration_xtalk(&mut self, target_dist: i16, nb_samples: i16) -> Result<i16, Error<B::Error>> {
        let mut results: ResultsData;
        let mut avg_dist: f32 = 0.0;
        let mut avg_spad: f32 = 0.0;
        let mut avg_signal: f32 = 0.0;
        let mut count_samples: f32 = 0.0;
        let measured_xtalk: i16;

        if nb_samples < 5 || nb_samples > 255 || target_dist < 10 || target_dist > 5000 {
            return Err(Error::InvalidParam);
        }

        // Disable Xtalk compensation
        self.write_u16(VL53L4ED_XTALK_PLANE_OFFSET_KCPS, 0)?;

        // Device heat loop (10 samples)
        self.start_ranging()?;
        for _ in 0..10 {
            while !self.check_data_ready()? {}
            self.get_ranging_data()?;
            self.clear_interrupt()?;
        }
        self.stop_ranging()?;

        // Device ranging
        self.start_ranging()?;
        for i in 0..nb_samples {
            while !self.check_data_ready()? {}
            results = self.get_ranging_data()?;
            self.clear_interrupt()?;
            if results.range_status == 0 && i > 0 {
                avg_dist += results.distance_mm as f32;
                avg_spad += results.number_of_spad as f32;
                avg_signal += results.signal_rate_kcps as f32;
                count_samples += 1.0;
            }
        }
        self.stop_ranging()?;
        if count_samples == 0.0 {
            return Err(Error::Other);
        }
        avg_dist /= count_samples;
        avg_spad /= count_samples;
        avg_signal /= count_samples;

        let tmp_xtalk = (1.0 - (avg_dist / target_dist as f32)) * (avg_signal / avg_spad);

        // 127kcps is the max Xtalk value (65536/512)
        if tmp_xtalk > 127.0 {
            return Err(Error::Other);
        }

        // Send data to firmware
        self.write_u16(VL53L4ED_XTALK_PLANE_OFFSET_KCPS, (tmp_xtalk * 512.0) as u16)?;
        
        measured_xtalk = tmp_xtalk as i16;
        Ok(measured_xtalk)
    }
    
}