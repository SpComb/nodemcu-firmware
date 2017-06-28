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

static void nexa_write(int pin, uint32_t data, int repeat)
{
  for (int i = 0; i < repeat; i++) {
    nexa_write_sync(pin);
    for (int i = 31; i >= 0; i--) {
      nexa_write_bit(pin, data & (0x1 << i));
    }
    nexa_write_pause(pin);
  }
}

static int nexa_send(lua_State *L)
{
  unsigned int pin = luaL_checkinteger(L, 1);
  unsigned int data = luaL_checkinteger(L, 2);
  unsigned int repeat = luaL_optinteger(L, 3, nexa_repeat);

  MOD_CHECK_ID(gpio, pin);

  if (platform_gpio_mode(pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT) < 0)
    return luaL_error(L, "invalid gpio");

  nexa_write(pin, data, repeat);

  return 0;
}

// Module function map
static const LUA_REG_TYPE nexa_map[] =
{
  { LSTRKEY("send"),       LFUNCVAL(nexa_send) },
  { LNILKEY, LNILVAL }
};

NODEMCU_MODULE(NEXA, "nexa", nexa_map, NULL);
