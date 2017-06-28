#include "module.h"
#include "lauxlib.h"
#include "platform.h"

const int nexa_pulse_high_us = 245;
const int nexa_pulse_low_us = 260;
const int nexa_repeat = 5;

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
    nexa_write_pulse(pin, 1, 1);
    nexa_write_pulse(pin, 1, 5);
  } else {
    nexa_write_pulse(pin, 1, 5);
    nexa_write_pulse(pin, 1, 1);
  }
}

static inline void nexa_write_pause(int pin)
{
  nexa_write_pulse(pin, 1, 40);
}

static void nexa_write(int pin, uint32_t data)
{
    nexa_write_sync(pin);
    for (int i = 31; i >= 0; i--) {
      nexa_write_bit(pin, data & (0x1 << i));
    }
    nexa_write_pause(pin);
}

static void nexa_send(int pin, uint32_t id, bool nogroup, bool off, uint8_t chan, uint8_t unit, int repeat)
{
  uint32_t data = (
    (id << 6) | (nogroup ? 0x20 : 0) | (off ? 0x10 : 0) | ((chan & 0x3) << 2) | ((unit & 0x3) << 0)
  );

  for (int i = 0; i < repeat; i++) {
    nexa_write(pin, data);
  }
}

static inline void nexa_send_on(int pin, uint32_t id, uint8_t unit, int repeat)
{
  nexa_send(pin, id, !!unit, 0, 0x11, unit, repeat);
}

static inline void nexa_send_off(int pin, uint32_t id, uint8_t unit, int repeat)
{
  nexa_send(pin, id, !!unit, 1, 0x11, unit, repeat);
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
  unsigned int chan = luaL_checkinteger(L, 5);
  unsigned int unit = luaL_checkinteger(L, 6);
  unsigned int repeat = luaL_optinteger(L, 7, nexa_repeat);

  MOD_CHECK_ID(gpio, pin);

  if (platform_gpio_mode(pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT) < 0)
    return luaL_error(L, "invalid gpio");

  nexa_send(pin, id, nogroup, off, chan, unit, repeat);

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
  { LNILKEY, LNILVAL }
};

NODEMCU_MODULE(NEXA, "nexa", nexa_map, NULL);
