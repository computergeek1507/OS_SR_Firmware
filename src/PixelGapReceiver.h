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

    enum class GateMode { kSegment, kDumb, kPixelRange };

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
    // no dependency on the bit/pixel decode below (used only diagnostically
    // otherwise, for the OLED). Call once per receiver right after begin()
    // to assign the pin, then setTargetSegment()/setDumbMode()/
    // setPixelRangeGating() to pick a mode -- all three are safe to call
    // again later too (e.g. a live dial change), and take effect immediately
    // (closing the gate now, letting the next boundary event reopen it
    // correctly) rather than waiting for a reset to notice.
    void configureGating(uint enPin, bool enableActiveHigh) {
        gateEnPin_ = enPin;
        gateActiveHigh_ = enableActiveHigh;
        gatingConfigured_ = true;
    }

    void setTargetSegment(uint8_t targetSegment) {
        gateTargetSegment_ = targetSegment;
        gateMode_ = GateMode::kSegment;
        gateSegmentIndex_ = 0;
        setGate(false);
    }

    // Dumb mode: hold the gate open unconditionally, ignoring segment
    // boundaries entirely -- passes every virtual receiver's data through
    // unfiltered. Also the robust choice for a gap-less/direct-feed frame,
    // unlike a specific target segment (see the gating edge-case note in the
    // README): it doesn't depend on gaps ever occurring.
    void setDumbMode() {
        gateMode_ = GateMode::kDumb;
        setGate(true);
    }

    // Falcon v2 addressing: unlike FPP v2, Falcon's own pixel data doesn't
    // appear to use gap-delimited segments at all (see README) -- addressing
    // instead comes from a separate config packet giving this receiver's
    // pixel range within one continuous composite stream. Byte decode of
    // that packet isn't implemented yet (still unknown -- see
    // suspectedFalconV2() below and the README), so `startPixel`/`count`
    // have to come from somewhere else for now (e.g. hardcoded for a bench
    // test) until that's wired in. Takes effect on the pixel *count*
    // reaching these bounds (hooked in pushBit()), not on gaps/resets.
    void setPixelRangeGating(uint16_t startPixel, uint16_t count) {
        gateStartPixel_ = startPixel;
        gateEndPixel_ = startPixel + count;
        gateMode_ = GateMode::kPixelRange;
        setGate(false);
    }

    // For when something else drives EN_n directly, bypassing setGate() (e.g.
    // Test Mode) -- keeps setGate()'s open/closed bookkeeping accurate so its
    // redundant-write dedupe doesn't skip a write that's actually needed once
    // control returns here. Doesn't touch the pin itself.
    void syncGateState(bool open) { gateOpen_ = open; }

    // Best-effort, sticky (not cleared per-frame -- meant to answer "does
    // this port's traffic look like Falcon v2", a fixed property of what's
    // connected there) flag for whether this port's traffic looks like
    // Falcon v2 rather than plain FPP v2. Set in onHighPulse() -- see the
    // comment there for why an anomalously long *high* pulse, not a low-side
    // duration band, is the signature used. As of the info-packet decode
    // below, this also marks the point where the UART deserializer arms
    // itself for the burst that follows.
    bool suspectedFalconV2() const { return sawFalconBurst_; }

    // One-shot (like frameReady()/huntCaptureReady()): true once a fresh
    // info packet has been captured and passed its sync check. Meant for a
    // temporary bring-up dump (see main.cpp) to verify the byte layout
    // against known configuration on real hardware; not needed for gating
    // itself -- see falconTableReady()/applyFalconChannels() below.
    bool falconInfoReady() const { return falconInfoReady_; }
    void consumeFalconInfo() { falconInfoReady_ = false; }
    static constexpr int kFalconInfoByteCount = 57;
    const uint8_t *falconInfoBytes() const { return uartByteBuf_; }
    GateMode gateMode() const { return gateMode_; }

    // The info packet only ever arrives on one physical port (confirmed on
    // real hardware -- see README), but its 24-word channel table covers
    // all 4 ports' chain positions. So decoding (this class, per-instance,
    // armed off this instance's own marker) and applying (below) are split:
    // whichever instance actually decodes a valid table exposes it here,
    // one-shot like falconInfoReady() above, and the caller (main.cpp) is
    // responsible for calling applyFalconChannels() on *all four* receiver
    // instances with that same table, each passing its own port index.
    static constexpr int kFalconChannelWordCount = 24;
    bool falconTableReady() const { return falconTableReady_; }
    void consumeFalconTable() { falconTableReady_ = false; }
    const uint16_t *falconChannels() const { return falconChannels_; }

    // Applies a previously-decoded 24-word channel table (see above) to
    // *this* receiver's own gating, self-selecting its column via the
    // board's dial ID (gateTargetSegment_, already set by
    // setTargetSegment()) and the given port index (0-3, this receiver's
    // own position among the board's 4 physical ports -- the caller knows
    // this since it owns the receivers[] array; this class doesn't need to
    // track it separately). Same dumb-mode/refresh-not-latch/sentinel
    // behavior previously inline in finishInfoPacketDecode().
    void applyFalconChannels(const uint16_t *channels, uint8_t portIndex) {
        if (gateMode_ == GateMode::kDumb) return;
        uint8_t letterIdx = gateTargetSegment_;
        if (letterIdx >= 6) return;
        int idx = letterIdx * 4 + portIndex;
        uint16_t endCh = channels[idx];
        if (endCh == 0xFFFF) return; // this chain position isn't configured
        uint16_t startCh = (letterIdx == 0) ? 0 : channels[idx - 4];
        if (endCh <= startCh) return; // corrupt/implausible table, discard
        setPixelRangeGating(startCh / 3, (endCh - startCh) / 3); // RGB: 3 channels/pixel
    }

    // 0=completed all 57 bytes (check falconInfoBytes()[0] for sync pass/
    // fail), 1=bad start bit, 2=bad stop bit, 3=timed out mid-burst, 4=byte
    // count completed but sync byte didn't match, 5=implausibly long single
    // pulse (safety abort, see kFalconMaxBitsPerPulse). See uartAbort() above.
    uint8_t falconAbortReason() const { return falconAbortReason_; }
    int falconAbortByteIndex() const { return falconAbortByteIndex_; }

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
        // Falcon v2 detection: a real WS281x "1" bit never gets anywhere
        // near kGapThresholdNs (20us) even on the slowest variants (~1.2us).
        // A real capture of Falcon's post-pixel-data traffic (see README)
        // showed a sustained ~54us high (idle/mark) right after pixel data
        // ends and before its UART-scale burst -- wildly anomalous for a bit
        // pulse, and a much cleaner signature than trying to classify *low*
        // pulse durations, which legitimately overlap with real FPP v2 gap
        // timing (~102.5-105us in that protocol's own capture). Requiring at
        // least one real pixel already decoded this frame keeps this from
        // false-triggering on, say, a long pre-frame idle high.
        if (durationNs > kGapThresholdNs && pixelCount_ > 0) {
            sawFalconBurst_ = true;
            startInfoPacketCapture(); // arm the UART deserializer for the
                                       // burst that follows this pulse --
                                       // see the block above onLowPulse().
            return; // clearly not a real bit -- don't let it poison the
                     // adaptive short/long threshold below for the rest of
                     // this frame's (diagnostic-only) pixel decode.
        }
        if (uartActive_) {
            uartOnPulse(1, durationNs);
            return;
        }
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

    // Info-packet UART deserializer: decodes the fixed-length burst that
    // follows the anomalous high pulse classified in onHighPulse() above.
    // Driven directly off the same pulse callbacks rather than a separate
    // buffered replay -- each pulse's duration is converted to a whole
    // number of bit-times (rounded, with the remainder carried into the
    // next pulse so error doesn't accumulate across the burst) and expanded
    // into that many repeated bits of the pulse's level, fed one at a time
    // into a standard start/8-data/stop-bit byte framer.
    static constexpr float kFalconBitPeriodNs = 1250.0f;
    static constexpr uint8_t kFalconSyncByte = 0xAA;
    static constexpr uint32_t kFalconUartTimeoutUs = 2000; // >> ~712us expected

    void startInfoPacketCapture() {
        uartActive_ = true;
        uartTimeDebtNs_ = 0.0f;
        uartBitPos_ = 0;
        uartByteAccum_ = 0;
        uartByteIndex_ = 0;
        uartStartUs_ = micros();
    }

    // reason: 1=bad start bit, 2=bad stop bit, 3=timeout -- temporary
    // bring-up diagnostics (see falconAbortReason()/main.cpp's
    // dumpFalconInfo()) to see how far a failed decode got without needing
    // a scope, since a silent abort otherwise looks identical to "no signal
    // at all" from the outside.
    void uartAbort(uint8_t reason) {
        uartActive_ = false;
        falconAbortReason_ = reason;
        falconAbortByteIndex_ = uartByteIndex_;
        falconInfoReady_ = true;
        uartBitPos_ = 0;
        uartByteIndex_ = 0;
    }

    void uartFeedBit(uint32_t bit) {
        if (uartBitPos_ == 0) {
            if (bit != 0) { uartAbort(1); return; } // start bit must be low
            uartByteAccum_ = 0;
            uartBitPos_ = 1;
            return;
        }
        if (uartBitPos_ <= 8) {
            uartByteAccum_ |= (uint8_t)(bit << (uartBitPos_ - 1)); // LSB first
            uartBitPos_++;
            return;
        }
        // uartBitPos_ == 9: stop bit.
        if (bit != 1) { uartAbort(2); return; }
        uartByteBuf_[uartByteIndex_++] = uartByteAccum_;
        uartBitPos_ = 0;
        if (uartByteIndex_ >= kFalconInfoByteCount) {
            uartActive_ = false;
            falconAbortReason_ = 0; // 0 = completed, see finishInfoPacketDecode()
            finishInfoPacketDecode();
        }
    }

    // A single legitimate pulse can never span more than ~9-10 bit-times
    // (start/stop framing guarantees a level change at least that often --
    // see uartFeedBit()), so this is a generous safety margin, not a normal
    // codepath. Without it, a single implausibly long pulse fed in here by
    // mistake (e.g. if uartActive_ were ever left stuck true across a
    // multi-hundred-ms idle gap by a bug elsewhere) would iterate the loop
    // below hundreds of thousands of times before returning, stalling
    // poll() long enough to starve the other 3 ports' PIO FIFOs -- this
    // caps the damage to one aborted packet instead.
    static constexpr int kFalconMaxBitsPerPulse = 32;

    void uartOnPulse(uint32_t level, float durationNs) {
        if (micros() - uartStartUs_ > kFalconUartTimeoutUs) { uartAbort(3); return; }
        uartTimeDebtNs_ += durationNs;
        int bitsThisPulse = 0;
        while (uartTimeDebtNs_ >= kFalconBitPeriodNs * 0.5f) {
            if (++bitsThisPulse > kFalconMaxBitsPerPulse) { uartAbort(5); return; }
            uartTimeDebtNs_ -= kFalconBitPeriodNs;
            uartFeedBit(level);
            if (!uartActive_) return; // completed or aborted mid-pulse
        }
    }

    // Byte 0 is a fixed sync value -- treated as a sanity check, not
    // protocol content: a mismatch means a misaligned/corrupted capture of
    // this occurrence, discarded outright rather than risking a garbage
    // gating window (the transmitter repeats this packet every ~11 frames,
    // so one dropped occurrence costs nothing). Bytes 9.. are 24
    // little-endian 16-bit cumulative channel counts, 4 per chain position
    // (this board's dial ID, 0-based) across the board's 4 physical ports --
    // confirmed against a real capture with a known 5-pixel receiver (byte
    // order gave exactly 15 channels = 5 * 3; big-endian gave a nonsensical
    // 1280). Only decodes into falconChannels_ here -- see
    // applyFalconChannels() for why applying it to gating is the caller's
    // job, not this function's.
    void finishInfoPacketDecode() {
        falconInfoReady_ = true; // available for the bring-up dump either way
        if (uartByteBuf_[0] != kFalconSyncByte) { falconAbortReason_ = 4; return; }
        for (int w = 0; w < kFalconChannelWordCount; w++) {
            int off = 9 + w * 2;
            falconChannels_[w] = ((uint16_t)uartByteBuf_[off + 1] << 8) | uartByteBuf_[off];
        }
        falconTableReady_ = true;
    }

    bool uartActive_ = false;
    float uartTimeDebtNs_ = 0.0f;
    int uartBitPos_ = 0; // 0=expecting start bit, 1-8=data bits, 9=stop bit
    uint8_t uartByteAccum_ = 0;
    uint8_t uartByteBuf_[kFalconInfoByteCount];
    int uartByteIndex_ = 0;
    uint32_t uartStartUs_ = 0;
    bool falconInfoReady_ = false;
    uint8_t falconAbortReason_ = 0;
    int falconAbortByteIndex_ = 0;
    bool falconTableReady_ = false;
    uint16_t falconChannels_[kFalconChannelWordCount];

    void onLowPulse(float durationNs) {
        if (uartActive_) {
            // Mid-burst: every low pulse here is UART bit content (the
            // burst's own pulses are all well under kGapThresholdNs -- see
            // the comment at startInfoPacketCapture() -- so this can't
            // collide with real gap/reset classification below).
            uartOnPulse(0, durationNs);
            return;
        }
        if (durationNs > kResetThresholdNs) {
            // Long reset: end of the whole composite frame.
            if (gatingConfigured_) {
                if (gateMode_ == GateMode::kSegment) {
                    setGate(false);
                    gateSegmentIndex_ = 0;
                    // Segment 0 of the *new* frame starts right now --
                    // opening here (rather than only on a gap) is what makes
                    // an address-0 receiver handle a gap-less direct-feed
                    // frame correctly too, since that case never reaches the
                    // gap branch below at all.
                    if (gateTargetSegment_ == 0) setGate(true);
                } else if (gateMode_ == GateMode::kPixelRange) {
                    // Safety: don't let a still-open pixel-range gate carry
                    // across a frame boundary if gateEndPixel_ was never
                    // reached this frame (e.g. a short/corrupt frame).
                    setGate(false);
                }
            }
            closeSegment();
            frameReady_ = (segmentCount_ > 0);
            huntOnReset();
        } else if (durationNs > kGapThresholdNs) {
            // Gap: boundary between two virtual receivers' segments.
            if (gatingConfigured_ && gateMode_ == GateMode::kSegment) {
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
                // Falcon v2 addressing (see setPixelRangeGating()): unlike
                // kSegment mode, this doesn't wait for a gap/reset event --
                // the running pixel count itself is the only signal, since
                // Falcon's pixel data isn't gap-delimited.
                if (gatingConfigured_ && gateMode_ == GateMode::kPixelRange) {
                    if (pixelCount_ == gateStartPixel_) setGate(true);
                    if (pixelCount_ == gateEndPixel_) setGate(false);
                }
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
    GateMode gateMode_ = GateMode::kSegment;
    int gateSegmentIndex_ = 0;
    bool gateOpen_ = false;
    uint16_t gateStartPixel_ = 0;
    uint16_t gateEndPixel_ = 0;
    bool sawFalconBurst_ = false;

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
