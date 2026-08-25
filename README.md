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

`DATA_n` is a **pure input** at all times -- a dedicated PIO state machine
(`gap_capture.pio`, `pio0`) oversamples the pin and streams raw levels to
`PixelGapReceiver`, which classifies inter-segment gaps / end-of-frame resets
in software (see `src/PixelGapReceiver.h` for why this is done in software
rather than in PIO). The RP2040 never drives this pin.

The actual `DOUT_n` (LED string output) is driven by an external
**74AHCT1G125** single-gate buffer per port, wired *outside* the MCU: its
input is the same live incoming bus as `DATA_n`, and its output only reaches
the string while its enable pin (`EN_n`) is asserted. So driving output isn't
"decode a segment, then replay it" -- `PixelGapReceiver` counts gaps as they
go by on `DATA_n` in real time and holds `EN_n` low for exactly the duration
of the segment matching this board's address (`configureGating()` assigns
each receiver's `EN_n` pin once in `setup()`; `setTargetSegment()`/
`setDumbMode()` pick the actual addressing and are safe to call again any
time -- see "Board addressing" below for the live dial polling that does
exactly that), letting the live bits flow straight through the buffer with
no decode/regenerate step. `pixels()`/`segments()` are decoded purely for the
OLED diagnostic display; they don't drive output.

Known limitation of real-time gating: a non-zero board address on a frame
with *no* gaps at all (a direct point-to-point feed, not a multi-drop chain)
can't be gated correctly, since "no gaps occurred" is only known once the
frame's final reset arrives -- by then the addressed window already passed.
Accepted tradeoff: a non-zero dial address wouldn't normally be paired with a
single-receiver direct feed anyway.

`DIFF_EN` gates the shared input differential receiver for all 4 `DATA_n`
pins (always on during normal operation).

Core1 (`setup1`/`loop1`) does nothing but drain the 4 receivers' PIO FIFOs;
core0 handles diagnostic bookkeeping for the OLED status display, Test Mode,
and the address dial -- the actual output gating happens directly inside
`PixelGapReceiver` on core1 as gaps/resets are recognized, independent of
whatever core0 happens to be doing.

### Test Mode

`PIN_TEST_BUTTON` toggles Test Mode: de-asserts `DIFF_EN` (disabling the
input receiver), asserts all four `EN1-4`, and drives a local 300-pixel
solid red/green/blue cycle (`kTestModePixelCount`, ~1s per color) out all 4
ports via `Ws2812Output`/`pio1` -- the same PIO-generate-and-drive mechanism
normal operation deliberately avoids (see above). It's safe here specifically
*because* `DIFF_EN` is off: with the input receiver disabled there's no live
signal on the bus to contend with, so the RP2040 can safely reclaim the
shared pin as an output. Lets each port's LED string be checked without any
upstream signal. Press the button again to return to normal operation.

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
add a debug capture path: trigger it (see below) to arm a port, which then
re-arms itself every frame, capturing up to `kHuntWindowUs` (3ms) of raw
level-transition edges starting exactly at that frame's end-of-frame reset.
Once a capture lands with a non-trivial edge count (i.e. it caught a real
post-frame burst, not just idle), it's dumped over USB serial in the same
"Time [s],Channel 0" CSV format an oscilloscope export uses, so it drops
straight into the same offline analysis tooling used above -- then the hunt
automatically re-arms for the *next* occurrence, `kHuntBatchSize` (4) times
per trigger, so one trigger yields several consecutive packets to diff against
each other. This is the way to test whether the packet is per-receiver (e.g.
cycling 1st/2nd/3rd-receiver config across occurrences, as opposed to one
fixed periodic packet) -- look for a field that steps 0,1,2,0,... (or
similar) across the batch. Since each occurrence only appears once every ~11
frames, a full batch can take ~1 second. Capture it with `pio device monitor`
or any serial terminal, save each CSV block between the `---` markers to a
file, and re-run the same bit-period/framing analysis against it.

Trigger it by sending a byte over serial: `'1'`-`'4'` selects the port
(1-indexed), any other byte defaults to port 1. `PIN_TEST_BUTTON` is no
longer wired to this -- it now toggles Test Mode (see "Test Mode" above)
instead. (Earlier real-hardware testing suspected the button itself was
unresponsive, but it turned out to work fine -- there just wasn't a debug
print visible without a serial monitor already open to show a press had
registered; the serial trigger was added as this feature's primary path
regardless, since it doesn't depend on the button either way.)

### Board addressing

`SW1` (Nidec Copal SH-7010 rotary switch) sets a single board address used
identically by all 4 ports. Only the weight-1/2/4 pins are wired, so valid
positions are 0-7:

- **0 -- dumb mode:** `EN_n` is held permanently open rather than gated to
  any one segment, passing every virtual receiver's data through unfiltered.
  Also the robust choice for a direct point-to-point feed (no gap protocol,
  a single segment) -- unlike a specific non-zero ID, it doesn't depend on
  gaps ever occurring.
- **1-6 -- ID 1 through ID 6:** human-facing, 1-based; `EN_n` gates open only
  for the matching 0-based segment index (`boardAddress - 1`) as gaps are
  counted in real time. See "Architecture" above for how the gating itself
  works, including its one known edge case.
- **7:** follows the same `boardAddress - 1` mapping (segment index 6) for
  consistency, though not a case that's been explicitly exercised.

The dial is read **live**: `loop()` polls it every ~250ms (piggybacked on the
OLED refresh) and re-applies it (`applyBoardAddress()` in `main.cpp`) on any
change, so a physical dial turn takes effect within that poll interval --
no reset required. Not polled while Test Mode is active (gating is disabled
and `EN1-4` are forced on there regardless); the last-known address is
re-applied automatically when Test Mode exits.

## Hardware bring-up findings (2026-08-24)

First real-hardware bring-up surfaced two firmware bugs neither the scope
captures nor a desk review caught, both root-caused live via boot checkpoint
diagnostics (`checkpoint()` in `main.cpp`: blinks `PIN_STATUS_LED` N times
then holds solid, and logs over serial, after each `setup()` stage --
pinpointed exactly which call was hanging without needing a debugger):

- `setup()` hung indefinitely in `Wire.begin()`. `PIN_OLED_SDA`/`PIN_OLED_SCL`
  (GPIO26/27) are I2C1-only pins on the RP2040, but the display was wired to
  the default `Wire` object, which arduino-pico maps to I2C0 -- fixed by
  switching to `Wire1`. `setup()` was also reordered so receiver/pixel-routing
  init happens before the OLED, so an OLED fault can't take the rest of the
  board down with it, and display calls are now guarded behind whether
  `display.begin()` actually succeeded.
- The `"OS SM RECEIVER  addr:N"` status line ran past the 128px display width
  at textSize(1) (6px/char) and clipped/wrapped the address digit -- split
  across two lines instead.

Also confirmed on real hardware: `DIFF_EN` and `EN1-4` are both active-low
(`kEnableActiveHigh = false` in `main.cpp`) -- `DIFF_EN` via a live Falcon v2
signal on port 1 (0 segments/frames under active-high, started routing as
soon as this flipped), `EN1-4` confirmed directly. Not yet confirmed: the
dial's polarity (no dial populated on the board tested so far, so
`boardAddress` reads 0 by default -- see `dial.h`).

Also corrected: the original implementation had `DATA_n` doing double duty as
both input (via the differential receiver) and output (regenerating a
segment's WS2812 waveform via a second PIO block and driving it back out the
*same* pin). That's not how this board is actually wired -- `DOUT_n` is
driven by an external 74AHCT1G125 buffer fed by the same live bus, gated by
`EN_n`, entirely outside the MCU. Replaced with real-time gap-counted `EN_n`
gating; see "Architecture" above. `framesRouted` (shown on the OLED) changed
meaning as a result -- it now counts every fully-decoded frame on a port, not
only ones where this board's address happened to match a segment, since
addressing is now handled by the gate itself rather than by choosing what to
replay.

## Needs real hardware to finish

1. Verify the real-time `EN_n` gating: with a scope on `EN_n` vs `DATA_n`,
   confirm `EN_n` pulses low for exactly the duration of the segment matching
   `boardAddress`. Not verifiable from this session alone -- no LEDs were
   connected to check visually, and this specific behavior wasn't exercised
   before the architecture correction.
2. Run the Falcon hunt capture (see "Falcon hunt capture" above) against a
   real Falcon v2 controller to get a clean, on-device capture of the
   post-pixel-data packet, then reverse-engineer its byte layout -- the scope
   capture analyzed so far isn't reliable enough to decode byte-for-byte.
3. Confirm the dial's electrical polarity once a board with the dial
   populated is available (currently assumed active-low with internal
   pull-ups -- `dial.h`).

FPP v2 gap/reset thresholds are verified against a real capture -- see
"Protocol support" above. Worth revisiting with a longer/busier capture (more
pixels, more receivers) to widen confidence in the jitter ceiling used for
`kGapThresholdNs`.
