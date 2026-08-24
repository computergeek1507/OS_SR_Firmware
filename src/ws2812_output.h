#pragma once

#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>

#include "ws2812.pio.h"

// Drives one WS281x-style output port from a dedicated PIO state machine.
// Pixel words are packed GRB in the low 24 bits (matches WS2811/2812/2812B
// wire order); RGBW strings are not needed on this board.
class Ws2812Output {
public:
    void begin(PIO pio, uint pin, float freqHz = 800000.0f) {
        pio_ = pio;
        pin_ = pin;
        sm_ = pio_claim_unused_sm(pio_, true);
        offset_ = pio_add_program(pio_, &ws2812_program);
        ws2812_program_init(pio_, sm_, offset_, pin_, freqHz);
    }

    // Blocks (briefly) on TX FIFO backpressure; call after the previous
    // frame's reset gap (>50us idle) has elapsed.
    void show(const uint32_t *grbPixels, size_t count) {
        for (size_t i = 0; i < count; i++) {
            put(grbPixels[i]);
        }
    }

private:
    static inline void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freqHz) {
        pio_gpio_init(pio, pin);
        pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

        pio_sm_config c = ws2812_program_get_default_config(offset);
        sm_config_set_sideset_pins(&c, pin);
        sm_config_set_out_shift(&c, false, true, 24);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

        int cyclesPerBit = ws2812_T1 + ws2812_T2 + ws2812_T3;
        float div = (float)clock_get_hz(clk_sys) / (freqHz * cyclesPerBit);
        sm_config_set_clkdiv(&c, div);

        pio_sm_init(pio, sm, offset, &c);
        pio_sm_set_enabled(pio, sm, true);
    }

    inline void put(uint32_t grb) {
        // Left-justify: out_shift threshold is 24, autopull shifts from MSB.
        pio_sm_put_blocking(pio_, sm_, grb << 8u);
    }

    PIO pio_ = nullptr;
    uint pin_ = 0;
    uint sm_ = 0;
    uint offset_ = 0;
};
