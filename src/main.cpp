#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <hardware/pio.h>

#include "BoardConfig.h"
#include "dial.h"
#include "PixelGapReceiver.h"
#include "ws2812_output.h"

// TODO(verify on hardware): active level for DIFF_EN / EN1-4.
static constexpr bool kEnableActiveHigh = true;
static inline void setEnable(uint pin, bool on) {
    digitalWrite(pin, on == kEnableActiveHigh ? HIGH : LOW);
}

static const uint kDataPins[4] = {PIN_DATA1, PIN_DATA2, PIN_DATA3, PIN_DATA4};
static const uint kEnPins[4] = {PIN_EN1, PIN_EN2, PIN_EN3, PIN_EN4};

static PixelGapReceiver receivers[4];
static Ws2812Output outputs[4];
static bool outputsBegun[4] = {false, false, false, false};

static uint8_t boardAddress = 0;
static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

static uint32_t framesRouted[4] = {0, 0, 0, 0};

// Debug: hold PIN_TEST_BUTTON to hunt for whatever a transmitter sends right
// after a port's pixel data ends (e.g. Falcon v2's undocumented post-frame
// UART packet -- see README) and dump it over USB serial in the same
// "Time [s],Channel 0" CSV format an oscilloscope export uses, so it can be
// fed straight into the same offline analysis tooling. Capacity/window sized
// generously against the one real capture analyzed so far (~59 edges over
// ~800us); adjust if a real capture needs more of either.
static constexpr int kHuntCapacity = 600;
static constexpr uint32_t kHuntWindowUs = 3000;
static PixelGapReceiver::RawEdge huntBuf[kHuntCapacity];
static int huntPort = -1;

static void dumpHuntCapture(int port) {
    int n = receivers[port].huntCaptureCount();
    Serial.print("--- falcon hunt capture: port ");
    Serial.print(port);
    Serial.print(", ");
    Serial.print(n);
    Serial.println(" edges ---");
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

static void routeReadyFrames() {
    for (int port = 0; port < 4; port++) {
        if (!receivers[port].frameReady()) continue;

        int segCount = receivers[port].segmentCount();
        const PixelGapReceiver::Segment *segs = receivers[port].segments();
        const uint32_t *px = receivers[port].pixels();

        // No gaps at all -> this port's whole frame is ours regardless of
        // dial address (direct point-to-point feed, not a multi-drop chain).
        int chosen = -1;
        if (segCount == 1) {
            chosen = 0;
        } else if (boardAddress < segCount) {
            chosen = boardAddress;
        }

        if (chosen >= 0) {
            receivers[port].disable();

            if (!outputsBegun[port]) {
                outputs[port].begin(pio1, kDataPins[port]);
                outputsBegun[port] = true;
            }
            setEnable(kEnPins[port], true);
            outputs[port].show(px + segs[chosen].startPixel, segs[chosen].count);
            setEnable(kEnPins[port], false);

            receivers[port].consumeFrame();
            receivers[port].enable();
            framesRouted[port]++;
        } else {
            // Gap protocol is active but no segment matches our address --
            // just drop the frame and go back to listening.
            receivers[port].consumeFrame();
        }
    }
}

static void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("OS SM RECEIVER  addr:");
    display.println(boardAddress);
    for (int i = 0; i < 4; i++) {
        display.print("P");
        display.print(i + 1);
        display.print(':');
        display.print(receivers[i].segmentCount());
        display.print("seg ");
        display.print(framesRouted[i]);
        display.println("f");
    }
    display.display();
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

    boardAddress = readBoardAddress();

    Wire.setSDA(PIN_OLED_SDA);
    Wire.setSCL(PIN_OLED_SCL);
    Wire.begin();
    display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
    display.setRotation(0);
    updateDisplay();

    for (int i = 0; i < 4; i++) {
        receivers[i].begin(pio0, kDataPins[i]);
    }
}

void loop() {
    routeReadyFrames();

    static uint32_t lastDisplay = 0;
    uint32_t now = millis();
    if (now - lastDisplay > 250) {
        lastDisplay = now;
        updateDisplay();
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
    }

    // Debug: PIN_TEST_BUTTON press starts a Falcon-hunt capture on port 0
    // (see comment at huntBuf above). Debounced; ignored while a hunt is
    // already in progress.
    static bool lastButtonState = HIGH;
    static uint32_t lastButtonChangeMs = 0;
    bool buttonState = digitalRead(PIN_TEST_BUTTON);
    if (buttonState != lastButtonState && (now - lastButtonChangeMs) > 50) {
        lastButtonChangeMs = now;
        lastButtonState = buttonState;
        if (buttonState == LOW && huntPort < 0) {
            huntPort = 0;
            receivers[huntPort].startHunting(huntBuf, kHuntCapacity, kHuntWindowUs);
            Serial.println("Falcon hunt: listening on port 0 after each frame's reset "
                            "(may take ~11 frames to land on a packet)...");
        }
    }
    if (huntPort >= 0 && receivers[huntPort].huntCaptureReady()) {
        dumpHuntCapture(huntPort);
        receivers[huntPort].consumeHuntCapture();
        receivers[huntPort].stopHunting();
        huntPort = -1;
    }
}
