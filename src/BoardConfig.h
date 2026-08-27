#pragma once

// Pin mapping for the PB16 "SM Receiver Out SMD" board. Two hardware
// revisions are supported, selected by which PlatformIO environment you
// build (see platformio.ini's sm_receiver_rev1/sm_receiver_rev2, which set
// -D BOARD_REV1 / -D BOARD_REV2 respectively):
//
//   REV1: Pico module. Net names match Micro.kicad_sch/Outputs.kicad_sch/
//         Diff_In.kicad_sch in
//         https://github.com/computergeek1507/PB_16/tree/master/SM_Receiver_Out_SMD.
//         Current hardware -- pin numbers confirmed against a real board.
//   REV2: Waveshare RP2040-Tiny, to shrink the PCB. EN1-4 and the address
//         dial moved off REV1's GPIO10-13/19-21 because the RP2040-Tiny's
//         header doesn't break out GPIO18-25 at all (GPIO16/17 are used
//         on-board for its own WS2812 status LED) -- see its schematic:
//         https://files.waveshare.com/upload/7/7a/RP2040-Tiny_Schematic.pdf.
//         Not yet built -- pin numbers are a planned repin, not confirmed
//         against real hardware.
//
// Every other pin (DATA1-4, DIFF_EN, status LED, test button, OLED) is
// identical on both revisions.

#if !defined(BOARD_REV1) && !defined(BOARD_REV2)
#error "Define BOARD_REV1 or BOARD_REV2 (see platformio.ini environments)"
#endif

// Each of the 4 ports shares ONE GPIO pin for both directions: as DATA_n it
// reads the incoming differential stream (controller/upstream), and the same
// pin is reconfigured to GPIO output to drive that port's local pixel string
// (DOUT_n) once the port's segment has been decoded. Not simultaneous.
#define PIN_DATA1 2
#define PIN_DATA2 3
#define PIN_DATA3 4
#define PIN_DATA4 5

#define PIN_DOUT1 PIN_DATA1
#define PIN_DOUT2 PIN_DATA2
#define PIN_DOUT3 PIN_DATA3
#define PIN_DOUT4 PIN_DATA4

#define PIN_DIFF_EN 8

// Status / UI.
#define PIN_STATUS_LED 6
#define PIN_TEST_BUTTON 7

// Per-port output driver enables (must be driven active before DOUT_n will
// actually drive its string), and the board-address rotary dial (SW1,
// Nidec Copal SH-7010, 10-position BCD -- only the weight-1/2/4 bits are
// wired, so the dial only distinguishes addresses 0-7; positions 8 and 9
// are not usable as distinct addresses). These are the only pins that
// differ between revisions -- see the file header above for why.
#if defined(BOARD_REV2)
#define PIN_EN1 9
#define PIN_EN2 10
#define PIN_EN3 11
#define PIN_EN4 12

#define PIN_DIAL_BIT1 13
#define PIN_DIAL_BIT2 14
#define PIN_DIAL_BIT4 15
#else // BOARD_REV1
#define PIN_EN1 10
#define PIN_EN2 11
#define PIN_EN3 12
#define PIN_EN4 13

#define PIN_DIAL_BIT1 19
#define PIN_DIAL_BIT2 20
#define PIN_DIAL_BIT4 21
#endif

// OLED (SSD1306, 128x64) on I2C1.
#define PIN_OLED_SDA 26
#define PIN_OLED_SCL 27
#define OLED_I2C_ADDRESS 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
