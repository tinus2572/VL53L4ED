use embedded_hal::i2c::SevenBitAddress;

pub const VL53L4ED_DEFAULT_I2C_ADDRESS: SevenBitAddress = 0x52 >> 1;

pub(crate) const I2C_CHUNK_SIZE: usize = 32;

pub(crate) const VL53L4ED_SOFT_RESET : u16 = 0x0000;
pub(crate) const VL53L4ED_I2C_SLAVE_DEVICE_ADDRESS : u16 = 0x0001;
pub(crate) const VL53L4ED_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND : u16 = 0x0008;
pub(crate) const VL53L4ED_XTALK_PLANE_OFFSET_KCPS : u16 = 0x0016;
pub(crate) const VL53L4ED_XTALK_X_PLANE_GRADIENT_KCPS : u16 = 0x0018;
pub(crate) const VL53L4ED_XTALK_Y_PLANE_GRADIENT_KCPS : u16 = 0x001A;
pub(crate) const VL53L4ED_RANGE_OFFSET_MM : u16 = 0x001E;
pub(crate) const VL53L4ED_INNER_OFFSET_MM : u16 = 0x0020;
pub(crate) const VL53L4ED_OUTER_OFFSET_MM : u16 = 0x0022;
pub(crate) const VL53L4ED_GPIO_HV_MUX_CTRL : u16 = 0x0030;
pub(crate) const VL53L4ED_GPIO_TIO_HV_STATUS : u16 = 0x0031;
pub(crate) const VL53L4ED_SYSTEM_INTERRUPT : u16 = 0x0046;
pub(crate) const VL53L4ED_RANGE_CONFIG_A : u16 = 0x005E;
pub(crate) const VL53L4ED_RANGE_CONFIG_B : u16 = 0x0061;
pub(crate) const VL53L4ED_RANGE_CONFIG_SIGMA_THRESH : u16 = 0x0064;
pub(crate) const VL53L4ED_MIN_COUNT_RATE_RTN_LIMIT_MCPS : u16 = 0x0066;
pub(crate) const VL53L4ED_INTERMEASUREMENT_MS : u16 = 0x006C;
pub(crate) const VL53L4ED_THRESH_HIGH : u16 = 0x0072;
pub(crate) const VL53L4ED_THRESH_LOW : u16 = 0x0074;
pub(crate) const VL53L4ED_SYSTEM_INTERRUPT_CLEAR : u16 = 0x0086;
pub(crate) const VL53L4ED_SYSTEM_START : u16 = 0x0087;
pub(crate) const VL53L4ED_RESULT_RANGE_STATUS : u16 = 0x0089;
pub(crate) const VL53L4ED_RESULT_SPAD_NB : u16 = 0x008C;
pub(crate) const VL53L4ED_RESULT_SIGNAL_RATE : u16 = 0x008E;
pub(crate) const VL53L4ED_RESULT_AMBIENT_RATE : u16 = 0x0090;
pub(crate) const VL53L4ED_RESULT_SIGMA : u16 = 0x0092;
pub(crate) const VL53L4ED_RESULT_DISTANCE : u16 = 0x0096;

pub(crate) const VL53L4ED_RESULT_OSC_CALIBRATE_VAL : u16 = 0x00DE;
pub(crate) const VL53L4ED_FIRMWARE_SYSTEM_STATUS : u16 = 0x00E5;
pub(crate) const VL53L4ED_IDENTIFICATION_MODEL_ID : u16 = 0x010F;