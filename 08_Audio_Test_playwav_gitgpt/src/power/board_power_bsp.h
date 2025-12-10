#ifndef BOARD_POWER_BSP_H
#define BOARD_POWER_BSP_H

#include <stdint.h>

#ifdef __cplusplus

// C++ class interface (visible only to C++ files)
class board_power_bsp_t
{
private:
    const uint8_t epd_power_pin;
    const uint8_t audio_power_pin;
    const uint8_t vbat_power_pin;

public:
    board_power_bsp_t(uint8_t _epd_power_pin,uint8_t _audio_power_pin,uint8_t _vbat_power_pin);
    ~board_power_bsp_t();

    void POWEER_EPD_ON();
    void POWEER_EPD_OFF();
    void POWEER_Audio_ON();
    void POWEER_Audio_OFF();
    void VBAT_POWER_ON();
    void VBAT_POWER_OFF();
};

// C++ convenience typedef for handle
typedef board_power_bsp_t* board_power_handle_t;

// Optional inline helpers for C++ callers (do not provide C linkage)
static inline board_power_handle_t board_power_create_cpp(int epd_pin, int audio_pin, int vbat_pin) {
    return new board_power_bsp_t((uint8_t)epd_pin, (uint8_t)audio_pin, (uint8_t)vbat_pin);
}
static inline void board_power_epd_on_cpp(board_power_handle_t h) {
    if (h) h->POWEER_EPD_ON();
}
static inline void board_power_epd_off_cpp(board_power_handle_t h) {
    if (h) h->POWEER_EPD_OFF();
}
static inline void board_power_audio_on_cpp(board_power_handle_t h) {
    if (h) h->POWEER_Audio_ON();
}
static inline void board_power_audio_off_cpp(board_power_handle_t h) {
    if (h) h->POWEER_Audio_OFF();
}
static inline void board_power_vbat_on_cpp(board_power_handle_t h) {
    if (h) h->VBAT_POWER_ON();
}
static inline void board_power_vbat_off_cpp(board_power_handle_t h) {
    if (h) h->VBAT_POWER_OFF();
}
static inline void board_power_delete_cpp(board_power_handle_t h) {
    delete h;
}

// C ABI declarations (implemented in the .cpp file) so C files can include this header
extern "C" {
    board_power_handle_t board_power_create(int epd_pin, int audio_pin, int vbat_pin);
    void board_power_epd_on(board_power_handle_t h);
    void board_power_epd_off(board_power_handle_t h);
    void board_power_audio_on(board_power_handle_t h);
    void board_power_audio_off(board_power_handle_t h);
    void board_power_vbat_on(board_power_handle_t h);
    void board_power_vbat_off(board_power_handle_t h);
    void board_power_delete(board_power_handle_t h);
}

#else /* __cplusplus */

// Plain C callers see only an opaque handle and C function declarations.
#ifdef __cplusplus
extern "C" {
#endif

typedef void* board_power_handle_t;

board_power_handle_t board_power_create(int epd_pin, int audio_pin, int vbat_pin);
void board_power_epd_on(board_power_handle_t h);
void board_power_epd_off(board_power_handle_t h);
void board_power_audio_on(board_power_handle_t h);
void board_power_audio_off(board_power_handle_t h);
void board_power_vbat_on(board_power_handle_t h);
void board_power_vbat_off(board_power_handle_t h);
void board_power_delete(board_power_handle_t h);

#ifdef __cplusplus
}
#endif

#endif /* __cplusplus */

#endif // BOARD_POWER_BSP_H