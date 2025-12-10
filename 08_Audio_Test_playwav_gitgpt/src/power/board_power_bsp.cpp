#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "board_power_bsp.h"

// If ESP_ERROR_CHECK_WITHOUT_ABORT isn't available in this build environment,
// provide a no-op fallback so the code still compiles.
#ifndef ESP_ERROR_CHECK_WITHOUT_ABORT
#define ESP_ERROR_CHECK_WITHOUT_ABORT(x) (void)(x)
#endif

board_power_bsp_t::board_power_bsp_t(uint8_t _epd_power_pin,uint8_t _audio_power_pin,uint8_t _vbat_power_pin) :
    epd_power_pin(_epd_power_pin),
    audio_power_pin(_audio_power_pin),
    vbat_power_pin(_vbat_power_pin)
{
    gpio_config_t gpio_conf = {};
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (0x1ULL << epd_power_pin) | (0x1ULL << audio_power_pin) | (0x1ULL << vbat_power_pin);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gpio_conf));
}

board_power_bsp_t::~board_power_bsp_t()
{
    // nothing to clean up
}

void board_power_bsp_t::POWEER_EPD_ON()
{
    gpio_set_level((gpio_num_t)epd_power_pin, 0);
}

void board_power_bsp_t::POWEER_EPD_OFF()
{
    gpio_set_level((gpio_num_t)epd_power_pin, 1);
}

void board_power_bsp_t::POWEER_Audio_ON()
{
    gpio_set_level((gpio_num_t)audio_power_pin, 0);
}

void board_power_bsp_t::POWEER_Audio_OFF()
{
    gpio_set_level((gpio_num_t)audio_power_pin, 1);
}

void board_power_bsp_t::VBAT_POWER_ON()
{
    gpio_set_level((gpio_num_t)vbat_power_pin, 1);
}

void board_power_bsp_t::VBAT_POWER_OFF()
{
    gpio_set_level((gpio_num_t)vbat_power_pin, 0);
}

/* C-compatible wrappers so C translation units can use the board-power API
   without seeing the C++ 'class' declaration. These functions are declared
   in the header for C builds. */
extern "C" {

board_power_handle_t board_power_create(int epd_pin, int audio_pin, int vbat_pin)
{
    return reinterpret_cast<board_power_handle_t>(new board_power_bsp_t((uint8_t)epd_pin, (uint8_t)audio_pin, (uint8_t)vbat_pin));
}

void board_power_epd_on(board_power_handle_t h)
{
    if (h) reinterpret_cast<board_power_bsp_t*>(h)->POWEER_EPD_ON();
}

void board_power_epd_off(board_power_handle_t h)
{
    if (h) reinterpret_cast<board_power_bsp_t*>(h)->POWEER_EPD_OFF();
}

void board_power_audio_on(board_power_handle_t h)
{
    if (h) reinterpret_cast<board_power_bsp_t*>(h)->POWEER_Audio_ON();
}

void board_power_audio_off(board_power_handle_t h)
{
    if (h) reinterpret_cast<board_power_bsp_t*>(h)->POWEER_Audio_OFF();
}

void board_power_vbat_on(board_power_handle_t h)
{
    if (h) reinterpret_cast<board_power_bsp_t*>(h)->VBAT_POWER_ON();
}

void board_power_vbat_off(board_power_handle_t h)
{
    if (h) reinterpret_cast<board_power_bsp_t*>(h)->VBAT_POWER_OFF();
}

void board_power_delete(board_power_handle_t h)
{
    delete reinterpret_cast<board_power_bsp_t*>(h);
}

} // extern "C"