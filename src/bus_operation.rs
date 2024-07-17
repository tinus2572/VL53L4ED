use consts::*;
use crate::{consts, Vl53l4ed, Error, SevenBitAddress, I2c, OutputPin, DelayNs};

pub trait BusOperation {
    type Error;
    fn read(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error>; 
    fn write(&mut self, wbuf: &[u8]) -> Result<(), Self::Error>;
    fn write_read(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error>;
}

pub struct Vl53l4edI2C<P> {
    i2c: P,
    address: SevenBitAddress,
}

impl<P: I2c> Vl53l4edI2C<P> {
    pub(crate) fn new(i2c: P) -> Self {
        Vl53l4edI2C { i2c: i2c, address: VL53L4ED_DEFAULT_I2C_ADDRESS }
    }
}

impl<P: I2c> BusOperation for Vl53l4edI2C<P> {
    type Error = P::Error;

    #[inline]
    fn read(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.read(self.address, rbuf)?;
        
        Ok(())
    }
    
    #[inline]
    fn write(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, wbuf)?;

        Ok(())
    }
    
    #[inline]
    fn write_read(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, wbuf, rbuf)?;
        
        Ok(())
    }
}

impl<P, XST, T> Vl53l4ed<Vl53l4edI2C<P>, XST, T>
    where
    P: I2c,
    XST: OutputPin,
    T: DelayNs
{
    pub fn new_i2c(i2c: P, xshut_pin: XST, tim: T) -> Result<Self, Error<P::Error>> 
    {
        Ok(Vl53l4ed { 
            xshut_pin: xshut_pin,
            bus: Vl53l4edI2C::new(i2c),
            tim: tim,
            chunk_size: I2C_CHUNK_SIZE
        })
    }
    
    pub fn set_i2c_address(&mut self, i2c_address: SevenBitAddress) -> Result<(), Error<P::Error>> {
        self.write_to_register(VL53L4ED_I2C_SLAVE_DEVICE_ADDRESS, &[i2c_address])?;
        self.bus.address = i2c_address;
        
        Ok(())
    }

    pub fn init_sensor(&mut self, address: u8) -> Result<(), Error<P::Error>>{
        self.off()?;
        self.on()?;
        if address != self.bus.address {
            self.set_i2c_address(address)?;
        }
        self.is_alive()?;
        self.init()?;
        Ok(())
    }
}


