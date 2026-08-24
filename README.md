# OS Smart Receiver Firmware

Firmware for the PB16 "SM Receiver Out SMD" board:
https://github.com/computergeek1507/PB_16/tree/master/SM_Receiver_Out_SMD

RP2040, built with [PlatformIO](https://platformio.org/) on
[earlephilhower/arduino-pico](https://github.com/earlephilhower/arduino-pico).

## Build

Install the PlatformIO extension (VS Code) or the `pio` CLI, then:

```
pio run           # build
pio run -t upload # build + flash
```

## Architecture

Each of the board's 4 ports shares one GPIO pin for both directions:

- **DATA_n (input):** a dedicated PIO state machine (`gap_capture.pio`,
  `pio0`) oversamples the pin and streams raw levels to `PixelGapReceiver`,
  which decodes WS281x bits and classifies inter-segment gaps / end-of-frame
  resets in software (see `src/PixelGapReceiver.h` for why this is done in
  software rather than in PIO).
- **DOUT_n (output):** once a frame's segments are decoded, the matching
  segment is driven back out the same pin via a second PIO state machine
  (`ws2812.pio`, `pio1`) through `Ws2812Output` -- the canonical
  pico-examples WS2812 program, unmodified.

`EN1-4` gate each port's output driver (only asserted while actively writing
DOUT_n) and `DIFF_EN` gates the shared input differential receiver for all 4
DATA pins.

Core1 (`setup1`/`loop1`) does nothing but drain the 4 receivers' PIO FIFOs;
core0 handles routing decoded frames to outputs, the OLED status display, and
the address dial.

### Protocol support

- **FPP v2 (gap-based):** implemented per the public FPP source (WS281x pixel
  bits back-to-back, low-gap between virtual-receiver segments, longer low =
  end of frame). The gap/reset duration thresholds in
  `PixelGapReceiver::onLowPulse` have been verified against a real scope
  capture of 3 virtual receivers (2026-08-23): normal bit-cell low jitter
  tops out around 12us, the two real inter-receiver gaps measured
  ~102.5us/~105us, and the final end-of-frame reset measured ~300us --
  `kGapThresholdNs`/`kResetThresholdNs` (20us/150us) sit with wide margin
  between those bands and were confirmed stable across a 15-80us /
  150-250us sweep against that capture. Caveat: the capture's test rig was
  configured for 3 virtual receivers x 10 pixels each (30 total), but the
  capture only contains enough bit transitions for ~17-18 pixels -- almost
  certainly a partial/truncated scope trigger, not a protocol issue. Doesn't
  affect the gap/reset thresholds above (those come from the low-pulse
  durations *between* segments, not from how many pixels are in them), but a
  fuller capture would be needed to validate per-pixel decode.
- **Falcon v2 (UART-framed config packet):** not implemented. Timing/framing
  analyzed against a real scope capture (2026-08-23, 275 frames / 6.68s):
  - Frame period is a steady ~23.39ms (~42.8Hz).
  - The config packet is *not* sent every frame -- it appears on exactly
    every 11th frame (~257ms / ~3.9Hz), confirmed identically across all 25
    occurrences in the capture.
  - Each occurrence: pixel data ends, ~20.5us low, ~54us high (idle/mark),
    then a short (~800us) burst of UART-scale activity, then the long
    inter-frame idle.
  - The exact byte encoding is **not** reliably recovered from this capture.
    A bit-period sweep (scored by valid start/stop framing, consistent
    across all 25 occurrences) found two competing candidates -- ~8.15us/bit
    (~122.9kbaud) and ~6.2us/bit (~161kbaud) -- but decoding at the
    stronger-scoring ~6.2us candidate produces bytes that are bit-shifted
    versions of each other across consecutive "bytes", a classic sign of
    aliasing against a periodic pattern rather than true byte alignment.
    Likely cause: this is a single-ended probe of what's probably a
    differential (RS-485-class) line, and the resulting ringing/reflections
    are enough to defeat a naive edge-based decode. A real logic analyzer
    (or the RP2040's own receiver on real hardware, immune to that scope
    probing issue) is needed for a trustworthy byte layout -- see
    "Falcon hunt capture" below.
  - Open hypothesis: the packet may be per-receiver rather than one fixed
    payload -- e.g. occurrence N describing the 1st configured virtual
    receiver, N+1 the 2nd, N+2 the 3rd, cycling. Tested against the scope
    capture two ways (per-occurrence pulse count: 55-59, no clean grouping;
    a periodicity score across candidate cycle lengths 1-11 on the decoded
    bytes: period 3 wasn't the winner) but neither is conclusive given the
    decode-quality caveat above, and this capture's 3 receivers all shared
    the same pixel count, so a per-receiver field might be small enough to
    be lost in the noise anyway. The hunt-capture batch mode below is built
    to test this cleanly.

#### Falcon hunt capture

`PixelGapReceiver::startHunting()`/`main.cpp`'s `huntBuf`/`dumpHuntCapture()`
add a debug capture path: press `PIN_TEST_BUTTON` to arm port 0, which then
re-arms itself every frame, capturing up to `kHuntWindowUs` (3ms) of raw
level-transition edges starting exactly at that frame's end-of-frame reset.
Once a capture lands with a non-trivial edge count (i.e. it caught a real
post-frame burst, not just idle), it's dumped over USB serial in the same
"Time [s],Channel 0" CSV format an oscilloscope export uses, so it drops
straight into the same offline analysis tooling used above -- then the hunt
automatically re-arms for the *next* occurrence, `kHuntBatchSize` (4) times
per button press, so one press yields several consecutive packets to diff
against each other. This is the way to test whether the packet is per-receiver
(e.g. cycling 1st/2nd/3rd-receiver config across occurrences, as opposed to
one fixed periodic packet) -- look for a field that steps 0,1,2,0,... (or
similar) across the batch. Since each occurrence only appears once every ~11
frames, a full batch can take ~1 second. Capture it with `pio device monitor`
or any serial terminal, save each CSV block between the `---` markers to a
file, and re-run the same bit-period/framing analysis against it.

### Board addressing

`SW1` (Nidec Copal SH-7010 rotary switch) sets a single board address used
identically by all 4 ports: each port's incoming stream is checked for gap
segments, and the segment at index `boardAddress` is treated as this port's
own data. If a port's stream has no gaps at all, the whole frame is used
regardless of dial position (direct point-to-point feed). Only the
weight-1/2/4 pins are wired, so valid addresses are 0-7.

## Needs real hardware to finish

1. Run the Falcon hunt capture (see "Falcon hunt capture" above) against a
   real Falcon v2 controller to get a clean, on-device capture of the
   post-pixel-data packet, then reverse-engineer its byte layout -- the scope
   capture analyzed so far isn't reliable enough to decode byte-for-byte.
2. Confirm `EN1-4` / `DIFF_EN` active level (currently assumed active-high --
   `kEnableActiveHigh` in `main.cpp`).
3. Confirm the dial's electrical polarity (currently assumed active-low with
   internal pull-ups -- `dial.h`).

FPP v2 gap/reset thresholds (former item 1) are now verified against a real
capture -- see "Protocol support" above. Worth revisiting with a longer/busier
capture (more pixels, more receivers) to widen confidence in the jitter
ceiling used for `kGapThresholdNs`.
