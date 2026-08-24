#pragma once

// Pin mapping for the PB16 "SM Receiver Out SMD" board (RP2040, Pico module).
// Net names below match Micro.kicad_sch / Outputs.kicad_sch / Diff_In.kicad_sch
// in https://github.com/computergeek1507/PB_16/tree/master/SM_Receiver_Out_SMD
//
// All pin numbers below are confirmed against the board.

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

// Per-port output driver enables (must be driven active before DOUT_n will
// actually drive its string). DIFF_EN enables the shared input differential
// receiver for DATA1-4.
#define PIN_EN1 10
#define PIN_EN2 11
#define PIN_EN3 12
#define PIN_EN4 13

#define PIN_DIFF_EN 8

// Status / UI.
#define PIN_STATUS_LED 6
#define PIN_TEST_BUTTON 7

// Board-address rotary dial (SW1, Nidec Copal SH-7010, 10-position BCD).
// Only the weight-1/2/4 bits are wired -- weight-8 is not connected, so the
// dial only distinguishes addresses 0-7; positions 8 and 9 are not usable
// as distinct addresses.
#define PIN_DIAL_BIT1 19
#define PIN_DIAL_BIT2 20
#define PIN_DIAL_BIT4 21

// OLED (SSD1306, 128x64) on I2C1.
#define PIN_OLED_SDA 26
#define PIN_OLED_SCL 27
#define OLED_I2C_ADDRESS 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
