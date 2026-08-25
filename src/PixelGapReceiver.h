#pragma once

#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>

#include "gap_capture.pio.h"

// Decodes one WS281x-style pixel-data input line that may carry the FPP v2
// "gap" protocol: back-to-back WS281x pixel bits for N virtual receivers'
// worth of pixels, each receiver's block separated from the next by a low
// period longer than a normal bit cell but shorter than a full end-of-frame
// reset, with a final (longer) reset low ending the whole transmission.
//
// The PIO side (gap_capture.pio) only does raw oversampling of the pin; all
// bit/gap/reset classification happens here in software against thresholds
// derived from the observed pulse widths, so it isn't locked to one exact
// WS2811/WS2812/etc. bit rate.
//
// kGapThresholdNs / kResetThresholdNs below are tuned against a real 3-virtual-
// receiver FPP v2 capture rather than guessed -- see the comment at their
// definition and the project README.
class PixelGapReceiver {
public:
    static constexpr int kMaxPixelsPerPort = 512;
    static constexpr int kMaxSegments = 8;

    struct Segment {
        uint16_t startPixel;
        uint16_t count;
    };

    void begin(PIO pio, uint pin, float sampleHz = 2000000.0f) {
        pio_ = pio;
        pin_ = pin;
        sm_ = pio_claim_unused_sm(pio_, true);
        offset_ = pio_add_program(pio_, &gap_capture_program);

        pio_gpio_init(pio_, pin_);
        pio_sm_set_consecutive_pindirs(pio_, sm_, pin_, 1, false);

        pio_sm_config c = gap_capture_program_get_default_config(offset_);
        sm_config_set_in_pins(&c, pin_);
        sm_config_set_in_shift(&c, false /*shift left*/, true /*autopush*/, 32);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

        float div = (float)clock_get_hz(clk_sys) / sampleHz;
        sm_config_set_clkdiv(&c, div);
        sampleIntervalNs_ = 1e9f / sampleHz;

        pio_sm_init(pio_, sm_, offset_, &c);
        resetFrameState();
        pio_sm_set_enabled(pio_, sm_, true);
    }

    void disable() { pio_sm_set_enabled(pio_, sm_, false); }

    // Re-claims the pin's GPIO function for this SM (it may have been handed
    // to a different PIO block/SM for output in between frames) and resumes
    // capture.
    void enable() {
        pio_gpio_init(pio_, pin_);
        pio_sm_set_consecutive_pindirs(pio_, sm_, pin_, 1, false);
        pio_sm_clear_fifos(pio_, sm_);
        resetFrameState();
        pio_sm_set_enabled(pio_, sm_, true);
    }

    // Call frequently (e.g. every iteration of a dedicated core1 loop) to
    // drain the PIO RX FIFO and advance the decode state machine. Cheap and
    // non-blocking; does nothing if no new samples are available.
    void poll() {
        while (!pio_sm_is_rx_fifo_empty(pio_, sm_)) {
            uint32_t word = pio_sm_get(pio_, sm_);
            // A decoded frame is waiting for the consumer (core0) to read
            // segments()/pixels() and call consumeFrame(); keep draining the
            // FIFO so it can't overflow, but don't touch the frame buffers
            // until it's safe to start filling them again.
            if (frameReady_) {
                if (huntCaptureActive_) {
                    for (int i = 31; i >= 0; i--) huntOnSample((word >> i) & 1u);
                }
                continue;
            }
            for (int i = 31; i >= 0; i--) {
                uint32_t bit = (word >> i) & 1u;
                processSample(bit);
                if (huntCaptureActive_) huntOnSample(bit);
            }
        }
    }

    // True once a full frame (ended by a reset-length low) has been decoded.
    // Caller should read segments()/pixels(), then call consumeFrame().
    bool frameReady() const { return frameReady_; }
    const Segment *segments() const { return segments_; }
    int segmentCount() const { return segmentCount_; }
    const uint32_t *pixels() const { return pixels_; }

    void consumeFrame() {
        frameReady_ = false;
        resetFrameState();
    }

    // Debug: captures raw level-transition edges (exactly what feeds bit/gap
    // classification above, before pixel decode) into a caller-owned buffer,
    // starting right at the next end-of-frame reset and running for up to
    // `windowUs` afterward. Meant for grabbing whatever a transmitter sends
    // right after pixel data ends without needing an external scope -- e.g.
    // Falcon v2's undocumented post-pixel-data UART packet (see README).
    // Re-arms itself every frame while hunting, since that packet has been
    // observed to appear only once every ~11 frames.
    struct RawEdge {
        uint8_t level;
        uint32_t durationNs;
    };

    void startHunting(RawEdge *buf, int capacity, uint32_t windowUs) {
        huntBuf_ = buf;
        huntCapacity_ = capacity;
        huntWindowUs_ = windowUs;
        hunting_ = true;
        huntCaptureActive_ = false;
        huntReady_ = false;
        huntCount_ = 0;
    }
    void stopHunting() { hunting_ = false; huntCaptureActive_ = false; }
    bool huntCaptureReady() const { return huntReady_; }
    int huntCaptureCount() const { return huntCount_; }
    void consumeHuntCapture() { huntReady_ = false; huntCount_ = 0; }

    // Real-time output path: DATA_n is a pure input -- the LED string is
    // actually driven by an external buffer (74AHCT1G125) whose input is
    // wired outside the MCU to this same live bus, and whose output only
    // reaches the string while `enPin` is asserted. So driving output isn't
    // "decode a segment, then replay it" -- it's "count gaps as they go by,
    // and hold the gate open for exactly the live segment matching our
    // address," done from onLowPulse() as gaps/resets are recognized, with
    // no dependency on the bit/pixel decode below (that's diagnostic-only,
    // for the OLED). Call once per receiver right after begin() to assign the
    // pin, then setTargetSegment()/setDumbMode() to pick a mode -- both of
    // those are safe to call again later too (e.g. a live dial change), and
    // take effect immediately (closing the gate now, letting the next
    // segment-boundary event reopen it correctly) rather than waiting for a
    // reset to notice.
    void configureGating(uint enPin, bool enableActiveHigh) {
        gateEnPin_ = enPin;
        gateActiveHigh_ = enableActiveHigh;
        gatingConfigured_ = true;
    }

    void setTargetSegment(uint8_t targetSegment) {
        gateTargetSegment_ = targetSegment;
        gateDumbMode_ = false;
        gateSegmentIndex_ = 0;
        setGate(false);
    }

    // Dumb mode: hold the gate open unconditionally, ignoring segment
    // boundaries entirely -- passes every virtual receiver's data through
    // unfiltered. Also the robust choice for a gap-less/direct-feed frame,
    // unlike a specific target segment (see the gating edge-case note in the
    // README): it doesn't depend on gaps ever occurring.
    void setDumbMode() {
        gateDumbMode_ = true;
        setGate(true);
    }

    // For when something else drives EN_n directly, bypassing setGate() (e.g.
    // Test Mode) -- keeps setGate()'s open/closed bookkeeping accurate so its
    // redundant-write dedupe doesn't skip a write that's actually needed once
    // control returns here. Doesn't touch the pin itself.
    void syncGateState(bool open) { gateOpen_ = open; }

private:
    void resetFrameState() {
        level_ = 0;
        runLen_ = 0;
        bitAccum_ = 0;
        bitCount_ = 0;
        pixelCount_ = 0;
        segmentCount_ = 0;
        segStart_ = 0;
        shortHighNs_ = 0;
        longHighNs_ = 0;
        frameReady_ = false;
    }

    void processSample(uint32_t sample) {
        if (sample == level_) {
            runLen_++;
            return;
        }
        // Level changed: the run that just ended was `runLen_` samples of
        // `level_`. Classify it, then start a new run at the new level.
        float durationNs = runLen_ * sampleIntervalNs_;
        if (level_ == 1) {
            onHighPulse(durationNs);
        } else {
            onLowPulse(durationNs);
        }
        level_ = sample;
        runLen_ = 1;
    }

    void onHighPulse(float durationNs) {
        // Adaptive short/long threshold: track the shortest and longest high
        // pulses seen so far this frame and split at their midpoint. Robust
        // to different WS281x variants without hardcoding exact timing.
        if (shortHighNs_ == 0 || durationNs < shortHighNs_) shortHighNs_ = durationNs;
        if (durationNs > longHighNs_) longHighNs_ = durationNs;
        float threshold = (shortHighNs_ + longHighNs_) / 2.0f;
        uint32_t bit = (longHighNs_ > 0 && durationNs > threshold) ? 1 : 0;
        pushBit(bit);
    }

    // Verified against a real 3-virtual-receiver FPP v2 capture (scope trace,
    // 2026-08-23): normal bit-cell low jitter (bit-banged TX timing noise)
    // topped out around 12us; the two real inter-receiver gaps measured
    // ~102.5us and ~105us; the final end-of-frame reset measured ~300us.
    // These aren't tied to bit period -- unlike the WS281x bit encoding
    // itself, the gap/reset timing is a fixed protocol-level interval on the
    // transmitter side, so absolute thresholds (with wide margin against the
    // bands above) are more robust here than bit-period-relative ones.
    static constexpr float kGapThresholdNs = 20000.0f;    // > this: virtual-receiver boundary
    static constexpr float kResetThresholdNs = 150000.0f; // > this: end of composite frame

    void onLowPulse(float durationNs) {
        if (durationNs > kResetThresholdNs) {
            // Long reset: end of the whole composite frame.
            if (gatingConfigured_ && !gateDumbMode_) {
                setGate(false);
                gateSegmentIndex_ = 0;
                // Segment 0 of the *new* frame starts right now -- opening
                // here (rather than only on a gap) is what makes an address-0
                // receiver handle a gap-less direct-feed frame correctly too,
                // since that case never reaches the gap branch below at all.
                if (gateTargetSegment_ == 0) setGate(true);
            }
            closeSegment();
            frameReady_ = (segmentCount_ > 0);
            huntOnReset();
        } else if (durationNs > kGapThresholdNs) {
            // Gap: boundary between two virtual receivers' segments.
            if (gatingConfigured_ && !gateDumbMode_) {
                if (gateSegmentIndex_ == gateTargetSegment_) setGate(false);
                gateSegmentIndex_++;
                if (gateSegmentIndex_ == gateTargetSegment_) setGate(true);
            }
            closeSegment();
        }
        // Otherwise this is just the normal tail-low of a bit cell -- ignore.
    }

    void setGate(bool open) {
        if (open == gateOpen_) return;
        gateOpen_ = open;
        digitalWrite(gateEnPin_, open == gateActiveHigh_ ? HIGH : LOW);
    }

    void pushBit(uint32_t bit) {
        bitAccum_ = (bitAccum_ << 1) | bit;
        bitCount_++;
        if (bitCount_ == 24) {
            if (pixelCount_ < kMaxPixelsPerPort) {
                pixels_[pixelCount_++] = bitAccum_;
            }
            bitAccum_ = 0;
            bitCount_ = 0;
        }
    }

    void closeSegment() {
        if (bitCount_ != 0) {
            // Partial pixel at a boundary means our gap/bit thresholds are
            // off, or we've lost sync -- drop it rather than emit garbage.
            bitAccum_ = 0;
            bitCount_ = 0;
        }
        uint16_t count = pixelCount_ - segStart_;
        if (count > 0 && segmentCount_ < kMaxSegments) {
            segments_[segmentCount_].startPixel = segStart_;
            segments_[segmentCount_].count = count;
            segmentCount_++;
        }
        segStart_ = pixelCount_;
    }

    // Starts (or restarts) a hunt-capture window; called from onLowPulse()
    // right as a reset is recognized, so the window always begins exactly at
    // end-of-frame regardless of how long the consumer takes to notice.
    void huntOnReset() {
        if (!hunting_) return;
        huntCaptureActive_ = true;
        huntCount_ = 0;
        huntStartUs_ = micros();
        huntLevel_ = 2; // sentinel: no run started yet
        huntRunLen_ = 0;
    }

    // Independent of processSample()'s bit/gap state machine so it keeps
    // seeing samples even while frameReady_ is gating pixel decode (that's
    // exactly the post-reset window this exists to capture).
    void huntOnSample(uint32_t bit) {
        if (huntLevel_ == 2) {
            huntLevel_ = bit;
            huntRunLen_ = 1;
            return;
        }
        if (bit == huntLevel_) {
            huntRunLen_++;
            return;
        }
        huntPushEdge();
        huntLevel_ = bit;
        huntRunLen_ = 1;
        // poll() only calls us while huntCaptureActive_ is true, so it's
        // still true here unless huntPushEdge() just hit capacity.
        if (huntCaptureActive_ && micros() - huntStartUs_ > huntWindowUs_) {
            huntCaptureActive_ = false;
            huntReady_ = (huntCount_ > 0);
        }
    }

    void huntPushEdge() {
        if (huntBuf_ && huntCount_ < huntCapacity_) {
            huntBuf_[huntCount_++] = {(uint8_t)huntLevel_, (uint32_t)(huntRunLen_ * sampleIntervalNs_)};
        }
        if (huntCount_ >= huntCapacity_) {
            huntCaptureActive_ = false;
            huntReady_ = true;
        }
    }

    RawEdge *huntBuf_ = nullptr;
    int huntCapacity_ = 0;
    uint32_t huntWindowUs_ = 0;
    bool hunting_ = false;
    bool huntCaptureActive_ = false;
    bool huntReady_ = false;
    int huntCount_ = 0;
    uint32_t huntStartUs_ = 0;
    uint32_t huntLevel_ = 2;
    uint32_t huntRunLen_ = 0;

    uint gateEnPin_ = 0;
    bool gateActiveHigh_ = false;
    uint8_t gateTargetSegment_ = 0;
    bool gatingConfigured_ = false;
    bool gateDumbMode_ = false;
    int gateSegmentIndex_ = 0;
    bool gateOpen_ = false;

    PIO pio_ = nullptr;
    uint pin_ = 0;
    uint sm_ = 0;
    uint offset_ = 0;
    float sampleIntervalNs_ = 0;

    uint32_t level_ = 0;
    uint32_t runLen_ = 0;
    uint32_t bitAccum_ = 0;
    int bitCount_ = 0;
    float shortHighNs_ = 0;
    float longHighNs_ = 0;

    uint16_t segStart_ = 0;
    Segment segments_[kMaxSegments];
    int segmentCount_ = 0;
    volatile bool frameReady_ = false;

    uint32_t pixels_[kMaxPixelsPerPort];
    uint16_t pixelCount_ = 0;
};
