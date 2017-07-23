/*
 * Driver for Analog Devices ADXL345 accelerometer.
 *
 * Code based on BMP085 driver.
 */
#include "module.h"
#include "lauxlib.h"
#include "platform.h"
#include "c_stdlib.h"
#include "c_string.h"

static const int ADXL345_REG_DEVID         = 0x00;
static const int ADXL345_REG_OFS           = 0x1E; // X Y Z
static const int ADXL345_REG_THRES_ACT     = 0x24;
static const int ADXL345_REG_THRES_INACT   = 0x25;
static const int ADXL345_REG_TIME_INACT    = 0x26;
static const int ADXL345_REG_ACT_INACT_CTL = 0x27;
static const int ADXL345_REG_POWER_CTL     = 0x2D;
static const int ADXL345_REG_INT_ENABLE    = 0x2E;
static const int ADXL345_REG_INT_MAP       = 0x2F;
static const int ADXL345_REG_DATA_FORMAT   = 0x31;
static const int ADXL345_REG_DATA          = 0x32; // X0 X1 Y0 Y1 Z0 Z1
static const int ADXL345_REG_FIFO_CTL      = 0x38;
static const int ADXL345_REG_FIFO_STATUS   = 0x39;

static const int ADXL345_DEVID = 0xE5;

static const int ADXL345_ACT_CTL_DC = 0x00;
static const int ADXL345_ACT_CTL_AC = 0x80;
static const int ADXL345_ACT_CTL_X = 0x40;
static const int ADXL345_ACT_CTL_Y = 0x20;
static const int ADXL345_ACT_CTL_Z = 0x10;
static const int ADXL345_INACT_CTL_DC = 0x00;
static const int ADXL345_INACT_CTL_AC = 0x08;
static const int ADXL345_INACT_CTL_X = 0x04;
static const int ADXL345_INACT_CTL_Y = 0x02;
static const int ADXL345_INACT_CTL_Z = 0x01;

static const int ADXL345_POWER_CTL_FLAGS_MASK = 0x3C;
static const int ADXL345_POWER_CTL_WAKEUP_MASK = 0x02;
static const int ADXL345_POWER_CTL_LINK = 0x20;
static const int ADXL345_POWER_CTL_AUTO_SLEEP = 0x10;
static const int ADXL345_POWER_CTL_MEASURE = 0x08;
static const int ADXL345_POWER_CTL_SLEEP = 0x04;

static const int ADXL345_INT_DATA_READY = 0x80;
static const int ADXL345_INT_SINGLE_TAP = 0x40;
static const int ADXL345_INT_DOUBLE_TAP = 0x20;
static const int ADXL345_INT_ACTIVITY = 0x10;
static const int ADXL345_INT_INACTIVITY = 0x08;
static const int ADXL345_INT_FREE_FALL = 0x04;
static const int ADXL345_INT_WATERMARK = 0x02;
static const int ADXL345_INT_OVERRUN = 0x01;

static const int ADXL345_DATA_FORMAT_SELF_TEST = 0x80;
static const int ADXL345_DATA_FORMAT_SPI = 0x40;
static const int ADXL345_DATA_FORMAT_INT_INVERT = 0x20;
static const int ADXL345_DATA_FORMAT_FULL_RES = 0x08;
static const int ADXL345_DATA_FORMAT_JUSTIFY = 0x04;
static const int ADXL345_DATA_FORMAT_RANGE_2G = 0x00;
static const int ADXL345_DATA_FORMAT_RANGE_4G = 0x01;
static const int ADXL345_DATA_FORMAT_RANGE_8G = 0x02;
static const int ADXL345_DATA_FORMAT_RANGE_16G = 0x03;

static const int ADXL345_FIFO_MODE_MASK = 0xC0;
static const int ADXL345_FIFO_TRIGGER_MASK = 0x20;
static const int ADXL345_FIFO_SAMPLES_MASK = 0x1F;
static const int ADXL345_FIFO_MODE_BYPASS = 0x00;
static const int ADXL345_FIFO_MODE_FIFO = 0x40;
static const int ADXL345_FIFO_MODE_STREAM = 0x80;
static const int ADXL345_FIFO_MODE_TRIGGER = 0xC0;
static const int ADXL345_FIFO_TRIGGER_INT1 = 0x00;
static const int ADXL345_FIFO_TRIGGER_INT2 = 0x20;

static const int ADXL345_FIFO_STATUS_TRIG = 0x80;
static const int ADXL345_FIFO_STATUS_ENTRIES_MASK = 0x3F;

static const unsigned adxl345_i2c_id = 0;
static const uint16_t adxl345_i2c_addr = 0x53;

static void adxl345_send0(uint8_t reg) {
  platform_i2c_send_start(adxl345_i2c_id);
  platform_i2c_send_address(adxl345_i2c_id, adxl345_i2c_addr, PLATFORM_I2C_DIRECTION_TRANSMITTER);
  platform_i2c_send_byte(adxl345_i2c_id, reg);
  platform_i2c_send_stop(adxl345_i2c_id);
}

static void adxl345_send1(uint8_t reg, uint8_t v) {
  platform_i2c_send_start(adxl345_i2c_id);
  platform_i2c_send_address(adxl345_i2c_id, adxl345_i2c_addr, PLATFORM_I2C_DIRECTION_TRANSMITTER);
  platform_i2c_send_byte(adxl345_i2c_id, reg);
  platform_i2c_send_byte(adxl345_i2c_id, v);
  platform_i2c_send_stop(adxl345_i2c_id);
}

static void adxl345_send3(uint8_t reg, uint8_t v1, uint8_t v2, uint8_t v3) {
  platform_i2c_send_start(adxl345_i2c_id);
  platform_i2c_send_address(adxl345_i2c_id, adxl345_i2c_addr, PLATFORM_I2C_DIRECTION_TRANSMITTER);
  platform_i2c_send_byte(adxl345_i2c_id, reg);
  platform_i2c_send_byte(adxl345_i2c_id, v1);
  platform_i2c_send_byte(adxl345_i2c_id, v2);
  platform_i2c_send_byte(adxl345_i2c_id, v3);
  platform_i2c_send_stop(adxl345_i2c_id);
}

static uint8_t adxl345_recv1() {
  uint8_t ret;

  platform_i2c_send_start(adxl345_i2c_id);
  platform_i2c_send_address(adxl345_i2c_id, adxl345_i2c_addr, PLATFORM_I2C_DIRECTION_RECEIVER);
  ret = platform_i2c_recv_byte(adxl345_i2c_id, 0);
  platform_i2c_send_stop(adxl345_i2c_id);

  return ret;
}

static void adxl345_recv(uint8_t *data, size_t len) {
  platform_i2c_send_start(adxl345_i2c_id);
  platform_i2c_send_address(adxl345_i2c_id, adxl345_i2c_addr, PLATFORM_I2C_DIRECTION_RECEIVER);
  for (int i = 0; i < len; ) {
    *data++ = platform_i2c_recv_byte(adxl345_i2c_id, ++i < len);
  }
  platform_i2c_send_stop(adxl345_i2c_id);
}

static void adxl345_read(uint8_t reg, uint8_t *data, size_t len) {
    adxl345_send0(reg);

    return adxl345_recv(data, len);
}
static uint8_t adxl345_read_u8(uint8_t reg) {
    adxl345_send0(reg);

    return adxl345_recv1();
}

static void adxl345_write_u8(uint8_t reg, uint8_t val) {
    adxl345_send1(reg, val);
}

static void adxl345_write_3i8(uint8_t reg, int8_t v1, int8_t v2, int8_t v3) {
    adxl345_send3(reg, v1, v2, v3);
}

// Lua API
static int Ladxl345_setup(lua_State* L) {
    uint8_t  devid;

    devid = adxl345_read_u8(ADXL345_REG_DEVID);

    if (devid != 0xE5) {
        return luaL_error(L, "device not found");
    }

    // Enable sensor
    adxl345_write_u8(ADXL345_REG_POWER_CTL, ADXL345_POWER_CTL_MEASURE);

    return 0;
}

static int Ladxl345_init(lua_State* L) {

    uint32_t sda;
    uint32_t scl;

    platform_print_deprecation_note("adxl345.init() is replaced by adxl345.setup()", "in the next version");

    sda = luaL_checkinteger(L, 1);
    scl = luaL_checkinteger(L, 2);

    luaL_argcheck(L, sda > 0 && scl > 0, 1, "no i2c for D0");

    platform_i2c_setup(adxl345_i2c_id, sda, scl, PLATFORM_I2C_SPEED_SLOW);

    return Ladxl345_setup(L);
}

static int Ladxl345_read(lua_State* L) {
    uint8_t data[6];
    int x,y,z;

    adxl345_read(ADXL345_REG_DATA, data, sizeof(data));

    x = (int16_t) ((data[1] << 8) | data[0]);
    y = (int16_t) ((data[3] << 8) | data[2]);
    z = (int16_t) ((data[5] << 8) | data[4]);

    lua_pushinteger(L, x);
    lua_pushinteger(L, y);
    lua_pushinteger(L, z);

    return 3;
}

static int Ladxl345_get(lua_State* L) {
  unsigned reg = luaL_checkinteger(L, 1);
  uint8_t value;

  luaL_argcheck(L, reg <= 0xff, 1, "invalid register");

  value = adxl345_read_u8(reg);

  lua_pushinteger(L, value);

  return 1;
}

static int Ladxl345_set(lua_State* L) {
  unsigned reg = luaL_checkinteger(L, 1);
  unsigned value = luaL_checkinteger(L, 2);

  luaL_argcheck(L, reg <= 0xff, 1, "invalid register 0x00..ff");
  luaL_argcheck(L, value <= 0xff, 2, "invalid 8-bit value");

  adxl345_write_u8(reg, value);

  return 0;
}

static int Ladxl345_get_offset(lua_State* L) {
  int8_t ofs[3];

  adxl345_read(ADXL345_REG_OFS, ofs, sizeof(ofs));

  lua_pushinteger(L, ofs[0]);
  lua_pushinteger(L, ofs[1]);
  lua_pushinteger(L, ofs[2]);

  return 3;
}

static int Ladxl345_set_offset(lua_State* L) {
  int x = luaL_checkinteger(L, 1);
  int y = luaL_checkinteger(L, 2);
  int z = luaL_checkinteger(L, 3);

  luaL_argcheck(L, -128 <= x && x <= 127, 1, "invalid signed 8-bit value");
  luaL_argcheck(L, -128 <= y && y <= 127, 2, "invalid signed 8-bit value");
  luaL_argcheck(L, -128 <= z && z <= 127, 3, "invalid signed 8-bit value");

  adxl345_write_3i8(ADXL345_REG_OFS, x, y, z);

  return 0;
}

static const LUA_REG_TYPE Ladxl345_map[] = {
    { LSTRKEY( "get" ),          LFUNCVAL( Ladxl345_get )},
    { LSTRKEY( "set" ),          LFUNCVAL( Ladxl345_set )},
    { LSTRKEY( "get_offset" ),   LFUNCVAL( Ladxl345_get_offset )},
    { LSTRKEY( "set_offset" ),   LFUNCVAL( Ladxl345_set_offset )},
    { LSTRKEY( "read" ),         LFUNCVAL( Ladxl345_read )},
    { LSTRKEY( "setup" ),        LFUNCVAL( Ladxl345_setup )},
    /// init() is deprecated
    { LSTRKEY( "init" ),         LFUNCVAL( Ladxl345_init )},
    { LNILKEY, LNILVAL}
};

NODEMCU_MODULE(ADXL345, "adxl345", Ladxl345_map, NULL);
