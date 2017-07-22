#include "module.h"
#include "lauxlib.h"
#include "platform.h"

const int nexa_pulse_high_us = 245;
const int nexa_pulse_low_us = 260;
const int nexa_repeat = 5;

#define NEXA_SINGLE 0
#define NEXA_GROUP 1

#define NEXA_ON 0
#define NEXA_OFF 1

#define NEXA_TYPE 0

#define NEXA_UNIT_ALL 0
#define NEXA_UNIT_1 1
#define NEXA_UNIT_2 2
#define NEXA_UNIT_3 3

static void nexa_write_pulse(int pin, int high, int low)
{
  platform_gpio_write(pin, PLATFORM_GPIO_HIGH);
  os_delay_us(high * nexa_pulse_high_us);
  platform_gpio_write(pin, PLATFORM_GPIO_LOW);
  os_delay_us(low * nexa_pulse_low_us);
}

static inline void nexa_write_sync(int pin)
{
  nexa_write_pulse(pin, 1, 10);
}

static inline void nexa_write_bit(int pin, int bit)
{
  if (bit) {
    nexa_write_pulse(pin, 1, 5);
    nexa_write_pulse(pin, 1, 1);
  } else {
    nexa_write_pulse(pin, 1, 1);
    nexa_write_pulse(pin, 1, 5);
  }
}

static inline void nexa_write_pause(int pin)
{
  nexa_write_pulse(pin, 1, 40);
}


// Sending one frame takes 68.6ms
//
// @param pin GPIO*
// @param data 32-bit frame
static void nexa_write(int pin, uint32_t data)
{
    nexa_write_sync(pin);
    for (int i = 31; i >= 0; i--) {
      nexa_write_bit(pin, data & (0x1 << i));
    }
    nexa_write_pause(pin);
}

// Each repeat is 68.6ms
//
// @param pin GPIOX
// @param id 26-bit ID
// @param group 1-bit NEXA_GROUP/SINGLE
// @param on 1-bit NEXA_ON/OFF
// @param type 2-bit NEXA_TYPE
// @param chan 2-bit
static void nexa_send(int pin, uint32_t id, bool group, bool on, uint8_t type, uint8_t chan, int repeat)
{
  uint32_t data = (
    // bits: id[26] !group !on chan[2] unit[2] = 32 bits
    (id << 6) | (group ? 0x20 : 0) | (on ? 0x10 : 0) | ((type & 0x3) << 2) | ((chan & 0x3) << 0)
  );

  for (int i = 0; i < repeat; i++) {
    nexa_write(pin, data);
  }
}

// @param id 26-bit
// @param unit NEXA_UNIT_ALL/1/2/3
// @param repeat
static inline void nexa_send_on(int pin, uint32_t id, uint8_t unit, int repeat)
{
  nexa_send(pin, id, !unit, NEXA_ON, NEXA_TYPE, unit - 1, repeat);
}

// @param id 26-bit
// @param unit NEXA_UNIT_ALL/1/2/3
// @param repeat
static inline void nexa_send_off(int pin, uint32_t id, uint8_t unit, int repeat)
{
  nexa_send(pin, id, !unit, NEXA_OFF, NEXA_TYPE, unit - 1, repeat);
}

static int nexa_lua_write(lua_State *L)
{
  unsigned int pin = luaL_checkinteger(L, 1);
  unsigned int data = luaL_checkinteger(L, 2);

  MOD_CHECK_ID(gpio, pin);

  if (platform_gpio_mode(pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT) < 0)
    return luaL_error(L, "invalid gpio");

  nexa_write(pin, data);

  return 0;
}

static int nexa_lua_send(lua_State *L)
{
  unsigned int pin = luaL_checkinteger(L, 1);
  unsigned int id = luaL_checkinteger(L, 2);
  unsigned int nogroup = luaL_checkinteger(L, 3);
  unsigned int off = luaL_checkinteger(L, 4);
  unsigned int type = luaL_checkinteger(L, 5);
  unsigned int chan = luaL_checkinteger(L, 6);
  unsigned int repeat = luaL_optinteger(L, 7, nexa_repeat);

  MOD_CHECK_ID(gpio, pin);

  if (platform_gpio_mode(pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT) < 0)
    return luaL_error(L, "invalid gpio");

  nexa_send(pin, id, nogroup, off, type, chan, repeat);

  return 0;
}

static int nexa_lua_on(lua_State *L)
{
  unsigned int pin = luaL_checkinteger(L, 1);
  unsigned int id = luaL_checkinteger(L, 2);
  unsigned int unit = luaL_checkinteger(L, 3);
  unsigned int repeat = luaL_optinteger(L, 4, nexa_repeat);

  MOD_CHECK_ID(gpio, pin);

  if (platform_gpio_mode(pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT) < 0)
    return luaL_error(L, "invalid gpio");

  nexa_send_on(pin, id, unit, repeat);

  return 0;
}

static int nexa_lua_off(lua_State *L)
{
  unsigned int pin = luaL_checkinteger(L, 1);
  unsigned int id = luaL_checkinteger(L, 2);
  unsigned int unit = luaL_checkinteger(L, 3);
  unsigned int repeat = luaL_optinteger(L, 4, nexa_repeat);

  MOD_CHECK_ID(gpio, pin);

  if (platform_gpio_mode(pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT) < 0)
    return luaL_error(L, "invalid gpio");

  nexa_send_off(pin, id, unit, repeat);

  return 0;
}

// Module function map
static const LUA_REG_TYPE nexa_map[] =
{
  { LSTRKEY("write"),       LFUNCVAL(nexa_lua_write) },
  { LSTRKEY("send"),        LFUNCVAL(nexa_lua_send) },
  { LSTRKEY("on"),          LFUNCVAL(nexa_lua_on) },
  { LSTRKEY("off"),         LFUNCVAL(nexa_lua_off) },

  // constants
  { LSTRKEY( "SINGLE" ),    LNUMVAL( NEXA_SINGLE ) },
  { LSTRKEY( "GROUP" ),     LNUMVAL( NEXA_GROUP ) },
  { LSTRKEY( "ON" ),        LNUMVAL( NEXA_ON ) },
  { LSTRKEY( "OFF" ),       LNUMVAL( NEXA_OFF ) },
  { LSTRKEY( "TYPE" ),      LNUMVAL( NEXA_TYPE ) },
  { LSTRKEY( "UNIT_ALL" ),  LNUMVAL( NEXA_UNIT_ALL ) },
  { LSTRKEY( "UNIT_1" ),    LNUMVAL( NEXA_UNIT_1 ) },
  { LSTRKEY( "UNIT_2" ),    LNUMVAL( NEXA_UNIT_2 ) },
  { LSTRKEY( "UNIT_3" ),    LNUMVAL( NEXA_UNIT_3 ) },

  { LNILKEY, LNILVAL }
};

NODEMCU_MODULE(NEXA, "nexa", nexa_map, NULL);
