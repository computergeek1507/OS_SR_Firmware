#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <hardware/pio.h>

#include "BoardConfig.h"
#include "dial.h"
#include "PixelGapReceiver.h"
#include "ws2812_output.h"

// DIFF_EN and EN1-4 confirmed active-low on real hardware (2026-08-24):
// DIFF_EN via a live signal test (port 1 showed 0 segments/frames under
// active-high, started routing as soon as this flipped to false); EN1-4
// confirmed the same polarity directly.
static constexpr bool kEnableActiveHigh = false;
static inline void setEnable(uint pin, bool on) {
    digitalWrite(pin, on == kEnableActiveHigh ? HIGH : LOW);
}

static const uint kDataPins[4] = {PIN_DATA1, PIN_DATA2, PIN_DATA3, PIN_DATA4};
static const uint kEnPins[4] = {PIN_EN1, PIN_EN2, PIN_EN3, PIN_EN4};

static PixelGapReceiver receivers[4];
static Ws2812Output outputs[4];
static bool outputsBegun[4] = {false, false, false, false};

static uint8_t boardAddress = 0;
// GPIO26/27 (PIN_OLED_SDA/SCL) are I2C1-only pins on the RP2040 -- must use
// Wire1, not the default Wire (I2C0). Using the wrong instance here isn't
// just wrong output, it hangs Wire.begin() outright (see README).
static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire1, -1);
static bool oledOk = false;

static uint32_t framesRouted[4] = {0, 0, 0, 0};

// Debug: hold PIN_TEST_BUTTON to hunt for whatever a transmitter sends right
// after a port's pixel data ends (e.g. Falcon v2's undocumented post-frame
// UART packet -- see README) and dump it over USB serial in the same
// "Time [s],Channel 0" CSV format an oscilloscope export uses, so it can be
// fed straight into the same offline analysis tooling.
//
// Window widened to 22ms, then to 23.8ms (2026-08-24): a live capture at the
// original 3ms window kept getting cut short mid-cycle -- the window-timeout
// check only runs when an edge arrives, so during a long quiet stretch with
// no edges it doesn't fire until whatever edge comes next, however much
// later that is. That accidentally revealed real reset-to-reset periods of
// ~23.9-24.0ms with a second burst of activity ~16.7ms into the cycle, which
// a 3ms window can't capture cleanly. Since huntOnReset() restarts the
// window on every reset, a window has to stay *under* that observed cadence
// to reliably time out on its own (rather than only completing on the rare
// larger gap) -- but 22ms turned out to leave a ~2ms blind spot right before
// every reset that a config packet sitting late in the cycle would fall
// into and never get captured. 23.8ms trades a little of that reliability
// margin to shrink the blind spot to ~100-200us instead.
//
// Captures kHuntBatchSize *consecutive* occurrences per button press (not
// just one) so the dumps can be diffed against each other -- e.g. to check
// whether the packet cycles per-receiver (see README) by looking for a field
// that steps 0,1,2,0,1,2... across consecutive captures.
//
// kHuntBatchSize/kSkipUninteresting: for the Falcon config-packet hunt, bump
// kHuntBatchSize way up and set kSkipUninteresting true so dumpHuntCapture()
// only prints the full CSV for an occurrence containing an edge in
// [kInterestingDurationMinNs, kInterestingDurationMaxNs] (longer than
// ordinary WS281x bit-cell timing, shorter than a real reset -- i.e.
// UART-burst-scale, not pixel-scale); everything else gets a one-line
// summary. For verifying actual pixel *content* (e.g. against a known solid
// color set on the transmitter), set it false to always get the full dump.
// kHuntBatchSize bumped to 400 (2026-08-24): told the config packet should
// recur every 12th frame at 20fps (~600ms) -- but our own measured wire-level
// reset period has been rock-solid at ~24ms all session, not 50ms, so it's
// unclear whether that 12-frame count is against this wire rate (~288ms) or
// the show's logical frame rate (~600ms). Rather than guess which, 400
// cycles (~9.6s at ~24ms/cycle) comfortably spans several repetitions either
// way.
static constexpr int kHuntCapacity = 1000;
static constexpr uint32_t kHuntWindowUs = 23800;
static constexpr int kHuntBatchSize = 400;
static constexpr bool kSkipUninteresting = true;
static constexpr uint32_t kInterestingDurationMinNs = 4000;
static constexpr uint32_t kInterestingDurationMaxNs = 200000;
static PixelGapReceiver::RawEdge huntBuf[kHuntCapacity];
static int huntPort = -1;
static int huntBatchRemaining = 0;
static int huntBatchIndex = 0;

static void startFalconHunt(int port) {
    huntPort = port;
    huntBatchRemaining = kHuntBatchSize;
    huntBatchIndex = 0;
    receivers[huntPort].startHunting(huntBuf, kHuntCapacity, kHuntWindowUs);
    Serial.print("Falcon hunt: capturing ");
    Serial.print(kHuntBatchSize);
    Serial.print(" consecutive post-frame packets on port ");
    Serial.print(port + 1);
    Serial.println(" (may take ~11 frames per packet)...");
}

static void dumpHuntCapture(int port) {
    int n = receivers[port].huntCaptureCount();

    uint32_t maxInterestingNs = 0;
    for (int i = 0; i < n; i++) {
        uint32_t d = huntBuf[i].durationNs;
        if (d >= kInterestingDurationMinNs && d <= kInterestingDurationMaxNs && d > maxInterestingNs) {
            maxInterestingNs = d;
        }
    }
    if (kSkipUninteresting && maxInterestingNs == 0) {
        Serial.print("occurrence ");
        Serial.print(huntBatchIndex);
        Serial.print(": ");
        Serial.print(n);
        Serial.println(" edges, nothing unusual (all pixel-scale) -- skipped");
        return;
    }

    Serial.print("--- falcon hunt capture: port ");
    Serial.print(port + 1);
    Serial.print(", occurrence ");
    Serial.print(huntBatchIndex);
    Serial.print(", ");
    Serial.print(n);
    Serial.print(" edges, max interesting duration ");
    Serial.print(maxInterestingNs);
    Serial.println("ns ---");
    Serial.println("Time [s],Channel 0");
    double t = 0.0;
    for (int i = 0; i < n; i++) {
        Serial.print(t, 9);
        Serial.print(',');
        Serial.println(huntBuf[i].level);
        t += huntBuf[i].durationNs / 1e9;
    }
    Serial.println("--- end capture ---");
}

// Temporary bring-up instrumentation: whenever a receiver decodes an info
// packet (sync byte checked out), dump its raw bytes and the resulting
// gating decision over serial so the byte layout can be sanity-checked
// against a known configuration on real hardware -- e.g. does the reported
// pixel count match what's actually configured for this receiver? Fires on
// every decode (~every 11 frames) for any port that looks like Falcon v2, so
// expect it to be noisy; remove once the layout's confirmed.
static void dumpFalconInfo(int port) {
    uint8_t reason = receivers[port].falconAbortReason();
    if (reason != 0) {
        Serial.print("falcon info: port ");
        Serial.print(port + 1);
        Serial.print(" FAILED reason=");
        Serial.print(reason); // 1=bad start bit, 2=bad stop bit, 3=timeout, 4=sync mismatch
        Serial.print(" at byte ");
        Serial.println(receivers[port].falconAbortByteIndex());
        return;
    }
    Serial.print("falcon info: port ");
    Serial.print(port + 1);
    Serial.print(" bytes=[");
    const uint8_t *b = receivers[port].falconInfoBytes();
    for (int i = 0; i < PixelGapReceiver::kFalconInfoByteCount; i++) {
        if (i) Serial.print(',');
        Serial.print(b[i]);
    }
    Serial.print("] gateMode=");
    Serial.print((int)receivers[port].gateMode());
    Serial.println();
}

// Applies a dial reading to all 4 receivers' gating -- called once at boot
// and again on any live dial change (see loop()), and when Test Mode exits.
// Dial 0 is "dumb mode": EN_n held permanently open rather than gated,
// passing every segment through unfiltered (also the robust choice for a
// gap-less/direct-feed frame, unlike a specific non-zero ID -- see the
// gating edge-case note in the README). Dial 1-6 select "ID 1"-"ID 6"
// (human-facing, 1-based) against the 0-based segment index gating uses.
static void applyBoardAddress(uint8_t addr) {
    boardAddress = addr;
    for (int i = 0; i < 4; i++) {
        if (addr == 0) {
            receivers[i].setDumbMode();
        } else {
            receivers[i].setTargetSegment(addr - 1);
        }
    }
}

// Test mode: PIN_TEST_BUTTON toggles this. Disables the shared input receiver
// (DIFF_EN off) and drives a local WS2812 test pattern out all 4 ports
// instead, so each port's LED string can be checked without any upstream
// signal. This is the one place `Ws2812Output`/PIO1-on-the-shared-pin is
// still used post-refactor -- normal operation gates the live incoming bus
// through the external 74AHCT1G125 in real time instead (see
// PixelGapReceiver::configureGating), but that's specifically *not* safe here
// since it would mean two drivers on the pin; test mode sidesteps that by
// turning the input receiver off first, so there's only ever one.
static constexpr int kTestModePixelCount = 300;
static bool testModeActive = false;
static uint32_t testPattern[kTestModePixelCount];
static uint32_t lastTestColorChangeMs = 0;
static int testColorIndex = -1;

static inline uint32_t grbColor(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
}

static void enterTestMode() {
    testModeActive = true;
    if (huntPort >= 0) {
        // Disabling PIO0 input capture below would otherwise strand an
        // in-progress hunt with no more resets ever arriving to complete it.
        receivers[huntPort].stopHunting();
        huntPort = -1;
    }
    setEnable(PIN_DIFF_EN, false);
    for (int i = 0; i < 4; i++) {
        receivers[i].disable();
        if (!outputsBegun[i]) {
            outputs[i].begin(pio1, kDataPins[i]);
            outputsBegun[i] = true;
        }
        setEnable(kEnPins[i], true);
        receivers[i].syncGateState(true); // EN_n was just driven directly above
    }
    testColorIndex = -1; // force an immediate color set on the next service call
    Serial.println("Test mode: ON");
}

static void exitTestMode() {
    testModeActive = false;
    for (int i = 0; i < 4; i++) {
        receivers[i].enable(); // reclaims the pin for PIO0 input
    }
    // Re-applies (rather than a raw setEnable) so each receiver's gate
    // bookkeeping -- left accurately "open" by enterTestMode()'s
    // syncGateState() above -- drives a real write where one's actually
    // needed instead of a stale dedupe skipping it.
    applyBoardAddress(boardAddress);
    setEnable(PIN_DIFF_EN, true);
    Serial.println("Test mode: OFF");
}

static void serviceTestMode() {
    uint32_t now = millis();
    if (testColorIndex >= 0 && now - lastTestColorChangeMs < 1000) return;
    lastTestColorChangeMs = now;
    testColorIndex = (testColorIndex + 1) % 3;
    uint32_t color = testColorIndex == 0 ? grbColor(255, 0, 0)
                    : testColorIndex == 1 ? grbColor(0, 255, 0)
                                           : grbColor(0, 0, 255);
    for (int i = 0; i < kTestModePixelCount; i++) testPattern[i] = color;
    for (int port = 0; port < 4; port++) outputs[port].show(testPattern, kTestModePixelCount);
}

// Runs continuously on core1: just drains the 4 receivers' PIO FIFOs so the
// RX FIFOs (8 words / 256 samples deep each) never overflow. Kept minimal on
// purpose -- anything heavier here risks starving a port during a run of
// back-to-back samples on another one.
void setup1() {}
void loop1() {
    for (int i = 0; i < 4; i++) {
        receivers[i].poll();
    }
}

// Output is driven in real time by PixelGapReceiver's gap-counting gate (see
// PixelGapReceiver::configureGating) as segments go by on DATA_n -- it needs
// no help from here. This is just diagnostic bookkeeping for the OLED: note
// that framesRouted now counts every fully-decoded frame, not only ones where
// this port's address matched a segment (the gate itself is address-aware;
// this counter no longer needs to be).
static void routeReadyFrames() {
    for (int port = 0; port < 4; port++) {
        if (!receivers[port].frameReady()) continue;
        framesRouted[port]++;
        receivers[port].consumeFrame();
        receivers[port].enable();
    }
}

static void updateDisplay() {
    if (!oledOk) return;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    if (testModeActive) {
        display.println("TEST MODE");
        display.println("local RGB cycle,");
        display.println("input disabled.");
        display.println("Press button to");
        display.println("return to normal.");
        display.display();
        return;
    }
    // Kept on two short lines deliberately: at textSize(1) each char is 6px,
    // and a single line with both the title and address digit ran past the
    // 128px-wide display and wrapped/clipped the digit.
    display.println("OS SM RECEIVER");
    if (boardAddress == 0) {
        display.println("addr:0 (dumb)");
    } else {
        display.print("addr:");
        display.println(boardAddress);
    }
    for (int i = 0; i < 4; i++) {
        display.print("P");
        display.print(i + 1);
        display.print(':');
        display.print(receivers[i].segmentCount());
        display.print("seg ");
        display.print(framesRouted[i]);
        display.print("f");
        // "F" flags a port whose traffic looks like Falcon v2 (an
        // anomalously long high pulse after real pixel data -- see
        // PixelGapReceiver::suspectedFalconV2()) rather than plain FPP v2.
        // Diagnostic only -- doesn't change how the port is gated yet.
        if (receivers[i].suspectedFalconV2()) display.print(" F");
        display.println();
    }
    display.display();
}

// Debug: blinks PIN_STATUS_LED `n` times then holds it solidly on, and prints
// a matching line over serial. Called after each setup() stage completes, so
// if setup() hangs, the LED is left showing (by blink count, no serial
// terminal needed) the last stage that actually finished.
static void checkpoint(int n, const char *label) {
    for (int i = 0; i < n; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(120);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(120);
    }
    digitalWrite(PIN_STATUS_LED, HIGH);
    Serial.print("checkpoint ");
    Serial.print(n);
    Serial.print(": ");
    Serial.println(label);
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_DIFF_EN, OUTPUT);
    setEnable(PIN_DIFF_EN, true); // shared input receiver, always on

    for (int i = 0; i < 4; i++) {
        pinMode(kEnPins[i], OUTPUT);
        setEnable(kEnPins[i], false); // only driven true while writing DOUT_n
    }

    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
    pinMode(PIN_TEST_BUTTON, INPUT_PULLUP);
    checkpoint(1, "pins configured");

#if defined(FIXED_BOARD_ID)
    boardAddress = FIXED_BOARD_ID; // no dial populated -- see BoardConfig.h
    checkpoint(2, "board address fixed");
#else
    boardAddress = readBoardAddress();
    checkpoint(2, "board address read");
#endif

    // Pixel routing doesn't depend on the OLED at all, so bring it up first --
    // an OLED problem (wrong address, unplugged, miswired SDA/SCL) shouldn't
    // be able to take the actual receiver/output functionality down with it.
    for (int i = 0; i < 4; i++) {
        receivers[i].begin(pio0, kDataPins[i]);
        receivers[i].configureGating(kEnPins[i], kEnableActiveHigh);
        checkpoint(3 + i, "receiver started"); // checkpoints 3-6, one per port
    }
    applyBoardAddress(boardAddress);

#if !defined(NO_OLED)
    Wire1.setSDA(PIN_OLED_SDA);
    Wire1.setSCL(PIN_OLED_SCL);
    Wire1.begin();
    checkpoint(7, "wire begin");

    oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
    checkpoint(8, oledOk ? "oled begin ok" : "oled begin FAILED");
    if (oledOk) {
        display.setRotation(0);
        updateDisplay();
    }
#else
    // oledOk stays false -- updateDisplay() already no-ops on that, so no
    // other guards are needed anywhere else in this file.
    checkpoint(7, "oled skipped (NO_OLED)");
    checkpoint(8, "oled skipped (NO_OLED)");
#endif
    checkpoint(9, "setup complete");
}

void loop() {
    if (testModeActive) {
        serviceTestMode();
    } else {
        routeReadyFrames();
        for (int i = 0; i < 4; i++) {
            if (receivers[i].falconInfoReady()) {
                dumpFalconInfo(i);
                receivers[i].consumeFalconInfo();
            }
        }
        // The info packet only arrives on port 1 (confirmed on real
        // hardware -- see README), but its table covers all 4 ports' own
        // chain positions, so whichever receiver actually decodes it (in
        // practice, always receivers[0]) is applied to every receiver here,
        // each self-selecting its own column via its own port index.
        if (receivers[0].falconTableReady()) {
            const uint16_t *channels = receivers[0].falconChannels();
            for (int i = 0; i < 4; i++) {
                receivers[i].applyFalconChannels(channels, i);
            }
            receivers[0].consumeFalconTable();
        }
    }

    static uint32_t lastDisplay = 0;
    uint32_t now = millis();
    if (now - lastDisplay > 250) {
        lastDisplay = now;
#if !defined(FIXED_BOARD_ID)
        // Live dial: not meaningful mid-Test-Mode (gating is disabled and
        // EN1-4 are forced on there regardless), so skip while active --
        // applyBoardAddress() would otherwise fight that by writing EN_n via
        // the gate.
        if (!testModeActive) {
            uint8_t addr = readBoardAddress();
            if (addr != boardAddress) {
                applyBoardAddress(addr);
                Serial.print("Board address changed to ");
                Serial.println(addr);
            }
        }
#endif
        updateDisplay();
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
    }

    // PIN_TEST_BUTTON toggles Test Mode (see enterTestMode/exitTestMode).
    // Falcon-hunt capture (see comment at huntBuf above) is serial-triggered
    // only now -- '1'-'4' selects a port, any other byte defaults to port 1.
    static bool lastButtonState = HIGH;
    static uint32_t lastButtonChangeMs = 0;
    bool buttonState = digitalRead(PIN_TEST_BUTTON);
    if (buttonState != lastButtonState && (now - lastButtonChangeMs) > 50) {
        lastButtonChangeMs = now;
        lastButtonState = buttonState;
        if (buttonState == LOW) {
            if (testModeActive) {
                exitTestMode();
            } else {
                enterTestMode();
            }
        }
    }
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        // No point hunting while test mode has PIO0 input capture disabled --
        // no resets will ever fire to advance/complete the capture.
        if (huntPort < 0 && !testModeActive) {
            if (c >= '1' && c <= '4') {
                startFalconHunt(c - '1');
            } else if (c != '\r' && c != '\n') {
                startFalconHunt(0);
            }
        }
    }
    if (huntPort >= 0 && receivers[huntPort].huntCaptureReady()) {
        dumpHuntCapture(huntPort);
        receivers[huntPort].consumeHuntCapture();
        huntBatchIndex++;
        huntBatchRemaining--;
        if (huntBatchRemaining > 0) {
            receivers[huntPort].startHunting(huntBuf, kHuntCapacity, kHuntWindowUs);
        } else {
            receivers[huntPort].stopHunting();
            huntPort = -1;
            Serial.println("Falcon hunt: batch complete.");
        }
    }
}
