/*
  ==============================================================================
    HentaiFilter.h
    Phase 85: Diffuser Enhancement (Added Delay Component)

    Changes:
    - Diffuser: Added short delay (5-30ms) for more audible spatial effect
    - Delay time controlled by cutoff, creates subtle room-like quality

    All Filters (21 total):
    0-3: SVF (LP, HP, BP, Notch)
    4: MoogLadder, 5: Peak, 6: Comb, 7: OberheimSEM
    8: Flanger, 9: Diffuser, 10: SteinerParker
    11: DiodeLadder, 12: Formant, 13: Phaser
    14: Wasp, 15: Korg35, 16: LowpassGate, 17: Resonator
    18: RingMod, 19: Bitcrusher, 20: VocalFilter

    Previous: Phase 83.5 - CPU Optimization
  ==============================================================================
*/

#pragma once
#include <JuceHeader.h>
#include <cmath>
#include <complex>
#include <vector>

// ======================================================================
// ★ Phase 83.5: Fast Math Functions for CPU Optimization
// ======================================================================

// Fast tanh approximation (Pade approximant)
// Accuracy: 99%+ for |x| < 4, still good beyond
// Speed: ~5x faster than std::tanh
inline float fastTanh(float x)
{
    // Clamp to prevent overflow
    if (x < -4.0f) return -1.0f;
    if (x > 4.0f) return 1.0f;
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

// Fast tan approximation (Taylor series optimized)
// Accurate for 0 < x < π/4 (0.785), good up to π/2
// Used for filter coefficient calculation where x = π * fc / fs
// Speed: ~4x faster than std::tan
inline float fastTan(float x)
{
    // For filter coefficients, x is typically 0 to ~1.5
    // Use polynomial approximation
    float x2 = x * x;
    return x * (1.0f + x2 * (0.333333333f + x2 * (0.133333333f + x2 * 0.053968254f)));
}

// Fast sin approximation (Bhaskara I formula, modified)
// Input: phase in range 0-1 (not radians)
// Speed: ~3x faster than std::sin
inline float fastSinNorm(float phase)
{
    // Convert 0-1 to 0-2π internally
    float x = phase - 0.5f;  // Center around 0
    x = x - std::floor(x + 0.5f);  // Wrap to -0.5 to 0.5
    x *= 2.0f;  // Scale to -1 to 1

    // Parabolic approximation: 4x(1-|x|) with correction
    float y = 4.0f * x * (1.0f - std::abs(x));
    // Refine for better accuracy
    return y * (0.775f + 0.225f * std::abs(y));
}

class HentaiFilter
{
public:
    enum Mode {
        LowPass = 0,
        HighPass,
        BandPass,
        Notch,
        MoogLadder,
        Peak,
        Comb,
        OberheimSEM,
        Flanger,
        Diffuser,
        SteinerParker,
        DiodeLadder,    // ★ Phase 73: NEW
        Formant,        // ★ Phase 73: NEW
        Phaser,         // ★ Phase 75: NEW
        Wasp,           // ★ Phase 76: NEW
        Korg35,         // ★ Phase 77: NEW
        LowpassGate,    // ★ Phase 78: NEW
        Resonator,      // ★ Phase 79: NEW
        RingMod,        // ★ Phase 80: NEW
        Bitcrusher,     // ★ Phase 81: NEW
        VocalFilter     // ★ Phase 82: NEW
    };

    HentaiFilter() { reset(); }

    void prepare(double newSampleRate) {
        sampleRate = newSampleRate;

        // Comb/Flanger buffer (max 50ms delay)
        int maxDelaySamples = static_cast<int>(0.05 * sampleRate) + 1;
        combBuffer.resize(maxDelaySamples, 0.0f);
        combWritePos = 0;

        // Flanger buffer
        flangerBuffer.resize(maxDelaySamples, 0.0f);
        flangerWritePos = 0;
        flangerLfoPhase = 0.0f;

        // Diffuser allpass stages
        for (int i = 0; i < 8; ++i) {
            diffuserState[i] = 0.0f;
        }

        // ★ Phase 85: Diffuser delay buffer (max 50ms)
        int maxDiffuserDelay = static_cast<int>(0.05 * sampleRate) + 1;
        diffuserDelayBuffer.resize(maxDiffuserDelay, 0.0f);
        diffuserDelayWritePos = 0;

        // ★ Phase 79: Resonator delay line (max ~20Hz = 50ms at 44.1kHz)
        int maxResonatorDelay = static_cast<int>(0.05 * sampleRate) + 1;
        resonatorBuffer.resize(maxResonatorDelay, 0.0f);
        resonatorWritePos = 0;

        reset();
    }

    void reset() {
        s1 = s2 = 0.0f;
        for (int i = 0; i < 4; ++i) z[i] = 0.0f;
        smoothedCutoff = 1000.0f;
        smoothedRes = 0.707f;
        smoothedP1 = 0.0f;
        smoothedP2 = 0.0f;
        smoothedP3 = 0.0f;

        random.setSeed(juce::Time::currentTimeMillis());
        driftTarget = 0.0f;
        driftCurrent = 0.0f;

        // Peak filter states
        peakState1 = peakState2 = 0.0f;

        // Comb filter buffer
        std::fill(combBuffer.begin(), combBuffer.end(), 0.0f);
        combWritePos = 0;

        // Oberheim states
        obState1 = obState2 = 0.0f;

        // Flanger reset
        std::fill(flangerBuffer.begin(), flangerBuffer.end(), 0.0f);
        flangerWritePos = 0;
        flangerLfoPhase = 0.0f;

        // Diffuser reset
        for (int i = 0; i < 8; ++i) {
            diffuserState[i] = 0.0f;
        }
        // ★ Phase 85: Diffuser delay buffer reset
        std::fill(diffuserDelayBuffer.begin(), diffuserDelayBuffer.end(), 0.0f);
        diffuserDelayWritePos = 0;

        // Steiner-Parker reset
        spState1 = spState2 = 0.0f;

        // ★ Phase 73: Diode Ladder reset
        for (int i = 0; i < 4; ++i) diodeZ[i] = 0.0f;

        // ★ Phase 73: Formant reset (3 bandpass filters)
        for (int i = 0; i < 3; ++i) {
            formantS1[i] = formantS2[i] = 0.0f;
        }

        // ★ Phase 75: Phaser reset (6 allpass stages)
        for (int i = 0; i < 6; ++i) {
            phaserState[i] = 0.0f;
        }
        phaserLfoPhase = 0.0f;
        phaserFeedback = 0.0f;

        // ★ Phase 76: Wasp reset
        waspState1 = waspState2 = 0.0f;

        // ★ Phase 77: Korg35 reset
        korg35Lpf1 = korg35Lpf2 = korg35Hpf1 = 0.0f;

        // ★ Phase 78: Lowpass Gate reset
        lpgState1 = lpgState2 = 0.0f;
        lpgVactrolState = 0.0f;

        // ★ Phase 79: Resonator reset
        std::fill(resonatorBuffer.begin(), resonatorBuffer.end(), 0.0f);
        resonatorWritePos = 0;
        resonatorLpfState = 0.0f;

        // ★ Phase 80: RingMod reset
        ringModPhase = 0.0f;

        // ★ Phase 81: Bitcrusher reset
        bitcrushHeldSample = 0.0f;
        bitcrushSampleCounter = 0;

        // ★ Phase 82: VocalFilter reset (5 formant stages)
        for (int i = 0; i < 5; ++i) {
            vocalS1[i] = vocalS2[i] = 0.0f;
        }
    }

    void softReset(float decayFactor = 0.95f) {
        s1 *= decayFactor;
        s2 *= decayFactor;
        for (int i = 0; i < 4; ++i) z[i] *= decayFactor;
        driftCurrent *= decayFactor;
        driftTarget *= decayFactor;

        peakState1 *= decayFactor;
        peakState2 *= decayFactor;
        obState1 *= decayFactor;
        obState2 *= decayFactor;

        for (auto& sample : combBuffer) {
            sample *= decayFactor * decayFactor;
        }

        for (auto& sample : flangerBuffer) {
            sample *= decayFactor * decayFactor;
        }
        for (int i = 0; i < 8; ++i) {
            diffuserState[i] *= decayFactor;
        }
        // ★ Phase 85: Diffuser delay buffer decay
        for (auto& sample : diffuserDelayBuffer) {
            sample *= decayFactor * decayFactor;
        }
        spState1 *= decayFactor;
        spState2 *= decayFactor;

        // ★ Phase 73: Soft reset new states
        for (int i = 0; i < 4; ++i) diodeZ[i] *= decayFactor;
        for (int i = 0; i < 3; ++i) {
            formantS1[i] *= decayFactor;
            formantS2[i] *= decayFactor;
        }

        // ★ Phase 75: Phaser soft reset
        for (int i = 0; i < 6; ++i) {
            phaserState[i] *= decayFactor;
        }
        phaserFeedback *= decayFactor;

        // ★ Phase 76-79: New filters soft reset
        waspState1 *= decayFactor;
        waspState2 *= decayFactor;
        korg35Lpf1 *= decayFactor;
        korg35Lpf2 *= decayFactor;
        korg35Hpf1 *= decayFactor;
        lpgState1 *= decayFactor;
        lpgState2 *= decayFactor;
        lpgVactrolState *= decayFactor;
        for (auto& sample : resonatorBuffer) {
            sample *= decayFactor * decayFactor;
        }
        resonatorLpfState *= decayFactor;

        // ★ Phase 80-82: New filters soft reset
        bitcrushHeldSample *= decayFactor;
        for (int i = 0; i < 5; ++i) {
            vocalS1[i] *= decayFactor;
            vocalS2[i] *= decayFactor;
        }
    }

    // ======================================================================
    // AUDIO PROCESSING
    // ======================================================================
    float process(float x, int type, float cutoff, float res, float drive, float p1, float p2, float p3)
    {
        if (std::isnan(x) || std::isinf(x)) x = 0.0f;
        x = juce::jlimit(-20.0f, 20.0f, x);

        float out = 0.0f;
        float agc = 1.0f;

        switch (type)
        {
        case MoogLadder:
        {
            // ★ Phase 79.2: Moog output gain increased
            out = processMoog(x, cutoff, res, drive, p1, p2, p3);
            out *= 1.3f;  // Boost output
            break;
        }

        case Peak:
            out = processPeak(x, cutoff, res, p1, p2);
            break;

        case Comb:
            out = processComb(x, cutoff, res, p1, p2, p3);
            break;

        case OberheimSEM:
            out = processOberheim(x, cutoff, res, p1, p2);
            break;

        case Flanger:
            out = processFlanger(x, cutoff, res, p1, p2, p3);
            break;

        case Diffuser:
            out = processDiffuser(x, cutoff, res, p1, p2, p3);
            break;

        case SteinerParker:
            out = processSteinerParker(x, cutoff, res, p1, p2, drive);
            break;

        case DiodeLadder:
            // ★ Phase 79.2: Diode output gain increased
            out = processDiodeLadder(x, cutoff, res, p1, p2, p3, drive);
            out *= 1.2f;  // Additional boost
            break;

        case Formant:
            out = processFormant(x, cutoff, res, p1, p2, p3);
            break;

        case Phaser:
            out = processPhaser(x, cutoff, res, p1, p2, p3);
            break;

        case Wasp:
            // ★ Phase 79.2: Wasp output gain increased
            out = processWasp(x, cutoff, res, p1, p2, drive);
            out *= 1.2f;  // Boost output
            break;

        case Korg35:
            out = processKorg35(x, cutoff, res, p1, p2, drive);
            break;

        case LowpassGate:
            // ★ Phase 79.2: LPG output gain increased
            out = processLowpassGate(x, cutoff, res, p1, p2, p3);
            out *= 1.3f;  // Boost output
            break;

        case Resonator:
            out = processResonator(x, cutoff, res, p1, p2, p3);
            break;

            // ★ Phase 80-82: NEW FILTERS
        case RingMod:
            out = processRingMod(x, cutoff, res, p1, p2, p3);
            break;

        case Bitcrusher:
            out = processBitcrusher(x, cutoff, res, p1, p2, p3);
            break;

        case VocalFilter:
            out = processVocalFilter(x, cutoff, res, p1, p2, p3);
            break;

        default: // SVF types (LP, HP, BP, Notch)
        {
            if (res > 1.0f) {
                agc = 1.0f / std::sqrt(res);
            }
            out = processSVF(x, (Mode)type, cutoff, res);
            out *= agc;
            break;
        }
        }

        // ★ Phase 79.2: Universal Drive Processing (applied to OUTPUT)
        // This makes drive audible on ALL filters
        if (drive > 0.01f) {
            float driveAmount = drive * 4.0f;  // 0-4x gain boost
            float driven = out * (1.0f + driveAmount);
            // Soft saturation
            driven = fastTanh(driven * 0.7f) / 0.7f;
            // Mix based on drive amount
            out = out * (1.0f - drive) + driven * drive;
            // Slight overall boost for audibility
            out *= (1.0f + drive * 0.3f);
        }

        // ★ Phase 79.2: Global AGC for all filter types (after drive)
        if (type != LowPass && type != HighPass && type != BandPass && type != Notch) {
            if (res > 1.0f) {
                float globalAgc = 1.0f / (1.0f + (res - 1.0f) * 0.12f);
                out *= globalAgc;
            }
        }

        // Soft saturation for values exceeding threshold
        if (std::abs(out) > 4.0f) out = fastTanh(out);

        if (std::isnan(out) || std::isinf(out)) {
            reset();
            return 0.0f;
        }
        return out;
    }

    // ======================================================================
    // VISUALIZER CALCULATION
    // ======================================================================
    static float getMagnitudeForFrequency(float freq, float cutoff, float res, int typeInt,
        float drive = 0.0f, float p1 = 0.0f, float p2 = 0.0f, float p3 = 0.0f)
    {
        Mode type = (Mode)typeInt;
        if (freq <= 0.0f || cutoff <= 0.0f) return 0.0f;

        double w = 2.0 * juce::MathConstants<double>::pi * freq;
        double wc = 2.0 * juce::MathConstants<double>::pi * cutoff;
        double r = w / wc;

        float mag = 0.0f;

        switch (type) {
        case LowPass: {
            double q = 0.5 + (double)res * 1.5;
            mag = (float)(1.0 / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));
            break;
        }
        case HighPass: {
            double q = 0.5 + (double)res * 1.5;
            mag = (float)((r * r) / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));
            break;
        }
        case BandPass: {
            double q = 0.5 + (double)res * 1.5;
            mag = (float)((r / q) / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));
            break;
        }
        case Notch: {
            double q = 0.5 + (double)res * 1.5;
            mag = (float)(std::abs(1.0 - r * r) / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));
            break;
        }
        case MoogLadder: {
            float visualCutoff = cutoff;
            if (p3 > 0.01f) {
                visualCutoff *= (1.0f + p3 * 0.05f);
            }
            double wcMoog = 2.0 * juce::MathConstants<double>::pi * visualCutoff;
            double rMoog = w / wcMoog;

            double k = (res / 10.0) * 3.98;
            float makeupGain = 1.0f + (float)(k * 0.5 * p2);
            float driveFactor = 1.0f + (drive * 0.3f) + (p1 * 0.1f);

            std::complex<double> s(0.0, rMoog);
            std::complex<double> one(1.0, 0.0);
            std::complex<double> pole = one + s;
            std::complex<double> pole4 = std::pow(pole, 4.0);
            std::complex<double> denom = pole4 + k;
            mag = (float)(1.0 / std::abs(denom));
            mag *= makeupGain;
            mag *= driveFactor;
            if (res > 1.0) mag *= (1.0f + res * 0.1f);
            break;
        }

        case Peak: {
            float gainDb = p1 * 24.0f;
            float bandwidth = 0.1f + p2 * 3.9f;
            float Q = cutoff / (bandwidth * cutoff / 1.41f);
            Q = juce::jlimit(0.1f, 20.0f, Q);

            float A = std::pow(10.0f, gainDb / 40.0f);
            double omega = freq / cutoff;
            double omegaSq = omega * omega;

            double num = std::sqrt(std::pow(omegaSq - 1.0, 2.0) + std::pow(omega * A / Q, 2.0));
            double den = std::sqrt(std::pow(omegaSq - 1.0, 2.0) + std::pow(omega / Q, 2.0));
            mag = (float)(num / std::max(den, 0.001));

            if (std::abs(freq - cutoff) < cutoff * 0.5f / Q) {
                mag = std::max(mag, A);
            }
            break;
        }

        case Comb: {
            float delayMs = 0.1f + p1 * 49.9f;
            float feedback = (p2 * 2.0f - 1.0f) * 0.95f;
            float polarity = (p3 > 0.5f) ? -1.0f : 1.0f;

            float delaySec = delayMs / 1000.0f;
            float phase = 2.0f * juce::MathConstants<float>::pi * freq * delaySec;

            float fbScaled = feedback * polarity;
            float denomSq = 1.0f - 2.0f * fbScaled * std::cos(phase) + feedback * feedback;
            mag = 1.0f / std::sqrt(std::max(0.01f, denomSq));
            mag = juce::jlimit(0.1f, 10.0f, mag);
            break;
        }

        case OberheimSEM: {
            float mode = p1;
            double q = 0.5 + (double)res * 1.5;

            float lpMag = (float)(1.0 / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));
            float hpMag = (float)((r * r) / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));
            float notchMag = (float)(std::abs(1.0 - r * r) / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));

            if (mode < 0.5f) {
                float blend = mode * 2.0f;
                mag = lpMag * (1.0f - blend) + notchMag * blend;
            }
            else {
                float blend = (mode - 0.5f) * 2.0f;
                mag = notchMag * (1.0f - blend) + hpMag * blend;
            }

            if (p2 > 0.01f) {
                float warmthBoost = 1.0f + p2 * 0.2f * std::exp(-std::pow((freq - cutoff) / cutoff, 2.0f));
                mag *= warmthBoost;
            }
            break;
        }

        case Flanger: {
            float baseDelayMs = 1.0f + p1 * 9.0f;
            float depth = p2;
            float feedback = (p3 * 2.0f - 1.0f) * 0.9f;

            float delaySec = baseDelayMs / 1000.0f;
            float phase = 2.0f * juce::MathConstants<float>::pi * freq * delaySec;

            float cosPhase = std::cos(phase);
            float magNum = std::sqrt(1.0f + 2.0f * depth * cosPhase + depth * depth);
            float magDen = std::sqrt(1.0f - 2.0f * feedback * cosPhase + feedback * feedback);
            mag = magNum / std::max(0.1f, magDen);
            mag = juce::jlimit(0.1f, 5.0f, mag);
            break;
        }

        case Diffuser: {
            int stages = 2 + static_cast<int>(p2 * 6.0f);
            float diffusion = p1;

            float phaseAccum = 0.0f;
            for (int i = 0; i < stages; ++i) {
                float stageDelay = (5.0f + i * 3.0f) / 1000.0f;
                phaseAccum += 2.0f * juce::MathConstants<float>::pi * freq * stageDelay * diffusion;
            }
            mag = 1.0f + 0.1f * std::sin(phaseAccum) * diffusion;
            mag = juce::jlimit(0.8f, 1.2f, mag);
            break;
        }

        case SteinerParker: {
            float mode = p1;
            double q = 0.5 + (double)res * 2.0;

            float lpMag = (float)(1.0 / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));
            float bpMag = (float)((r / q) / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));
            float hpMag = (float)((r * r) / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));

            if (mode < 0.5f) {
                float blend = mode * 2.0f;
                mag = lpMag * (1.0f - blend) + bpMag * blend;
            }
            else {
                float blend = (mode - 0.5f) * 2.0f;
                mag = bpMag * (1.0f - blend) + hpMag * blend;
            }

            if (p2 > 0.01f) {
                mag *= (1.0f + p2 * 0.5f);
            }
            break;
        }

                          // ★ Phase 73: NEW FILTER VISUALIZATIONS
        case DiodeLadder: {
            // Similar to Moog but with asymmetric response
            double k = (res / 10.0) * 3.5;  // Slightly less resonance than Moog
            float satFactor = 1.0f + p1 * 0.3f;  // Saturation affects curve

            std::complex<double> s(0.0, r);
            std::complex<double> one(1.0, 0.0);
            std::complex<double> pole = one + s;
            std::complex<double> pole4 = std::pow(pole, 4.0);
            std::complex<double> denom = pole4 + k;
            mag = (float)(1.0 / std::abs(denom));

            // Asymmetry affects the resonance peak shape
            if (p2 > 0.01f && freq > cutoff * 0.8f && freq < cutoff * 1.2f) {
                mag *= (1.0f + p2 * 0.3f);
            }

            mag *= satFactor;
            if (res > 1.0) mag *= (1.0f + res * 0.15f);
            break;
        }

        case Formant: {
            // Formant filter: 3 bandpass peaks for vowel sounds
            // P1 = vowel (0-1: A-E-I-O-U)
            // P2 = gender shift
            // P3 = resonance

            // Formant frequencies for vowels (male voice base)
            // A: F1=730, F2=1090, F3=2440
            // E: F1=530, F2=1840, F3=2480
            // I: F1=270, F2=2290, F3=3010
            // O: F1=570, F2=840, F3=2410
            // U: F1=440, F2=1020, F3=2240

            float vowel = p1;
            float genderShift = 1.0f + (p2 - 0.5f) * 0.6f;  // 0.7x to 1.3x
            float formantRes = 2.0f + p3 * 8.0f;  // Q = 2 to 10

            // Interpolate formant frequencies based on vowel position
            float f1, f2, f3;
            getFormantFrequencies(vowel, f1, f2, f3);

            // Apply gender shift and cutoff scaling
            float scale = (cutoff / 1000.0f) * genderShift;
            f1 *= scale;
            f2 *= scale;
            f3 *= scale;

            // Calculate magnitude for each formant peak
            float mag1 = getFormantPeakMagnitude(freq, f1, formantRes);
            float mag2 = getFormantPeakMagnitude(freq, f2, formantRes * 0.8f);
            float mag3 = getFormantPeakMagnitude(freq, f3, formantRes * 0.6f);

            // Combine with decreasing weights
            mag = mag1 * 1.0f + mag2 * 0.7f + mag3 * 0.4f;
            mag = juce::jlimit(0.01f, 10.0f, mag);
            break;
        }

                    // ★ Phase 75: Phaser visualizer
        case Phaser: {
            // Phaser creates notches via allpass phase cancellation
            // P1 = LFO Rate (for display, show average position)
            // P2 = Depth
            // P3 = Feedback

            float depth = p2;
            float feedback = p3 * 0.9f;

            // Base frequency for notches (center of sweep)
            float centerFreq = cutoff;
            float modRange = centerFreq * depth * 2.0f;

            // Calculate approximate notch positions (6-stage phaser has 3 notches)
            float notch1 = centerFreq * 0.5f;
            float notch2 = centerFreq;
            float notch3 = centerFreq * 2.0f;

            // Combined magnitude response with notches
            float dist1 = std::abs(freq - notch1) / (notch1 * 0.3f);
            float dist2 = std::abs(freq - notch2) / (notch2 * 0.3f);
            float dist3 = std::abs(freq - notch3) / (notch3 * 0.3f);

            float notchDepth1 = 1.0f / (1.0f + std::exp(-dist1 * 3.0f + 1.5f));
            float notchDepth2 = 1.0f / (1.0f + std::exp(-dist2 * 3.0f + 1.5f));
            float notchDepth3 = 1.0f / (1.0f + std::exp(-dist3 * 3.0f + 1.5f));

            // Feedback increases resonance at notches
            float fbBoost = 1.0f + feedback * 2.0f;

            mag = notchDepth1 * notchDepth2 * notchDepth3;
            mag = 0.3f + mag * 0.7f * fbBoost;

            // Resonance from main knob
            if (res > 1.0f) {
                mag *= (1.0f + (res - 1.0f) * 0.1f);
            }

            mag = juce::jlimit(0.1f, 5.0f, mag);
            break;
        }

                   // ★ Phase 76: Wasp visualizer (12dB multimode)
        case Wasp: {
            float morphLP_HP = p1;
            double q = 0.5 + (double)res * 1.2;  // Wasp doesn't self-oscillate

            float lpMag = (float)(1.0 / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));
            float hpMag = (float)((r * r) / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));

            mag = lpMag * (1.0f - morphLP_HP) + hpMag * morphLP_HP;

            // CMOS grit adds harmonics
            if (p2 > 0.1f) {
                mag *= (1.0f + p2 * 0.15f);
            }
            mag = juce::jlimit(0.01f, 5.0f, mag);
            break;
        }

                 // ★ Phase 77: Korg35 visualizer (aggressive 12dB)
        case Korg35: {
            bool isHp = (p1 > 0.5f);
            double q = 0.5 + (double)res * 2.5;  // Very resonant

            if (isHp) {
                mag = (float)((r * r) / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));
            }
            else {
                mag = (float)(1.0 / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));
            }

            // Korg35 has aggressive resonance
            if (res > 2.0f) {
                float peakBoost = 1.0f + (res - 2.0f) * 0.3f;
                float peakDist = std::abs(freq - cutoff) / (cutoff * 0.2f);
                mag *= (1.0f + peakBoost * std::exp(-peakDist * peakDist));
            }
            mag = juce::jlimit(0.01f, 10.0f, mag);
            break;
        }

                   // ★ Phase 78: Lowpass Gate visualizer (VCA + LPF)
        case LowpassGate: {
            int mode = static_cast<int>(p1 * 2.99f);  // 0=VCA, 1=LPF, 2=Both
            double q = 0.5 + (double)res * 1.0;

            float lpfMag = (float)(1.0 / std::sqrt(std::pow(1.0 - r * r, 2.0) + std::pow(r / q, 2.0)));

            if (mode == 0) {
                // VCA only: flat response
                mag = 1.0f;
            }
            else if (mode == 1) {
                // LPF only
                mag = lpfMag;
            }
            else {
                // Both: combination
                mag = lpfMag * 0.8f;
            }
            mag = juce::jlimit(0.1f, 5.0f, mag);
            break;
        }

                        // ★ Phase 79: Resonator visualizer (comb-like peaks)
        case Resonator: {
            // Resonator creates strong peak at fundamental and harmonics
            float fundFreq = cutoff;
            float damping = 0.5f + p1 * 0.49f;
            float brightness = p2;

            // Main resonant peak
            float peakQ = 5.0f + damping * 50.0f;
            float dist = std::abs(freq - fundFreq) / (fundFreq / peakQ);
            mag = 1.0f / (1.0f + dist * dist);

            // Add harmonics (2nd, 3rd)
            for (int h = 2; h <= 3; ++h) {
                float harmFreq = fundFreq * h;
                float harmDist = std::abs(freq - harmFreq) / (harmFreq / peakQ);
                float harmMag = (1.0f / (1.0f + harmDist * harmDist)) * brightness / h;
                mag += harmMag;
            }

            mag = juce::jlimit(0.01f, 10.0f, mag);
            break;
        }

                      // ★ Phase 80: RingMod visualizer
        case RingMod: {
            // Ring modulation creates sidebands
            float modFreq = 20.0f + p1 * cutoff * 2.0f;
            float mix = p2;

            // Sidebands at f±modFreq
            float sb1 = std::abs(freq - modFreq);
            float sb2 = freq + modFreq;

            float dist1 = std::abs(freq - sb1) / (cutoff * 0.5f);
            float dist2 = std::abs(freq - sb2) / (cutoff * 0.5f);

            float sidebandMag = std::exp(-dist1 * dist1) + std::exp(-dist2 * dist2) * 0.5f;
            mag = (1.0f - mix) + mix * sidebandMag;
            mag = juce::jlimit(0.1f, 3.0f, mag);
            break;
        }

                    // ★ Phase 81: Bitcrusher visualizer
        case Bitcrusher: {
            // Bitcrusher adds harmonics/aliasing
            float bits = 1.0f + p1 * 15.0f;
            float downsample = 1.0f + p2 * 49.0f;

            // Show stepped frequency response
            float quantSteps = std::pow(2.0f, bits);
            float aliasFreq = cutoff / downsample;

            // Aliasing creates mirror frequencies
            float aliasMag = 1.0f;
            if (freq > aliasFreq) {
                float foldFreq = std::fmod(freq, aliasFreq * 2.0f);
                if (foldFreq > aliasFreq) foldFreq = aliasFreq * 2.0f - foldFreq;
                aliasMag = 0.3f + 0.7f * (foldFreq / aliasFreq);
            }

            mag = aliasMag;
            mag = juce::jlimit(0.1f, 2.0f, mag);
            break;
        }

                       // ★ Phase 82: VocalFilter visualizer (5 formant peaks)
        case VocalFilter: {
            float vowelMorph = p1;
            float character = p2;
            float formantQ = 3.0f + p3 * 12.0f;

            // Extended formant table (5 formants)
            float f1, f2, f3, f4, f5;
            getExtendedFormantFrequencies(vowelMorph, f1, f2, f3, f4, f5);

            float scale = cutoff / 1000.0f;
            f1 *= scale; f2 *= scale; f3 *= scale; f4 *= scale; f5 *= scale;

            // Calculate magnitude for each formant
            float mag1 = getFormantPeakMagnitude(freq, f1, formantQ);
            float mag2 = getFormantPeakMagnitude(freq, f2, formantQ * 0.8f);
            float mag3 = getFormantPeakMagnitude(freq, f3, formantQ * 0.6f);
            float mag4 = getFormantPeakMagnitude(freq, f4, formantQ * 0.4f);
            float mag5 = getFormantPeakMagnitude(freq, f5, formantQ * 0.3f);

            mag = mag1 * 0.4f + mag2 * 0.3f + mag3 * 0.15f + mag4 * 0.1f + mag5 * 0.05f;
            mag = juce::jlimit(0.01f, 5.0f, mag);
            break;
        }

        default: return 0.0f;
        }

        return juce::jlimit(0.001f, 50.0f, mag);
    }

private:
    double sampleRate = 44100.0;

    // SVF states
    float s1 = 0.0f, s2 = 0.0f;

    // Moog states
    float z[4] = { 0.0f };

    // Smoothed parameters
    float smoothedCutoff = 1000.0f;
    float smoothedRes = 0.707f;
    float smoothedP1 = 0.0f;
    float smoothedP2 = 0.0f;
    float smoothedP3 = 0.0f;

    // Drift (Moog P3)
    juce::Random random;
    float driftTarget = 0.0f;
    float driftCurrent = 0.0f;

    // Peak filter states
    float peakState1 = 0.0f;
    float peakState2 = 0.0f;

    // Comb filter
    std::vector<float> combBuffer;
    int combWritePos = 0;

    // Oberheim SEM states
    float obState1 = 0.0f;
    float obState2 = 0.0f;

    // Flanger
    std::vector<float> flangerBuffer;
    int flangerWritePos = 0;
    float flangerLfoPhase = 0.0f;

    // Diffuser (8 allpass stages)
    float diffuserState[8] = { 0.0f };
    // ★ Phase 85: Diffuser delay buffer
    std::vector<float> diffuserDelayBuffer;
    int diffuserDelayWritePos = 0;;

    // Steiner-Parker
    float spState1 = 0.0f;
    float spState2 = 0.0f;

    // ★ Phase 73: Diode Ladder
    float diodeZ[4] = { 0.0f };

    // ★ Phase 73: Formant (3 parallel bandpass filters)
    float formantS1[3] = { 0.0f };
    float formantS2[3] = { 0.0f };

    // ★ Phase 75: Phaser (6 allpass stages)
    float phaserState[6] = { 0.0f };
    float phaserLfoPhase = 0.0f;
    float phaserFeedback = 0.0f;

    // ★ Phase 76: Wasp
    float waspState1 = 0.0f;
    float waspState2 = 0.0f;

    // ★ Phase 77: Korg35
    float korg35Lpf1 = 0.0f;
    float korg35Lpf2 = 0.0f;
    float korg35Hpf1 = 0.0f;

    // ★ Phase 78: Lowpass Gate
    float lpgState1 = 0.0f;
    float lpgState2 = 0.0f;
    float lpgVactrolState = 0.0f;

    // ★ Phase 79: Resonator
    std::vector<float> resonatorBuffer;
    int resonatorWritePos = 0;
    float resonatorLpfState = 0.0f;

    // ★ Phase 80: RingMod
    float ringModPhase = 0.0f;

    // ★ Phase 81: Bitcrusher
    float bitcrushHeldSample = 0.0f;
    int bitcrushSampleCounter = 0;

    // ★ Phase 82: VocalFilter (5 formant stages)
    float vocalS1[5] = { 0.0f };
    float vocalS2[5] = { 0.0f };

    // ======================================================================
    // Helper: Formant frequency table (3 formants)
    // ======================================================================
    static void getFormantFrequencies(float vowel, float& f1, float& f2, float& f3) {
        // Vowel positions: A=0, E=0.25, I=0.5, O=0.75, U=1.0
        // Frequencies in Hz (male voice reference)
        const float formants[5][3] = {
            { 730.0f, 1090.0f, 2440.0f },  // A
            { 530.0f, 1840.0f, 2480.0f },  // E
            { 270.0f, 2290.0f, 3010.0f },  // I
            { 570.0f,  840.0f, 2410.0f },  // O
            { 440.0f, 1020.0f, 2240.0f }   // U
        };

        // Map vowel 0-1 to index with interpolation
        float pos = vowel * 4.0f;
        int idx1 = juce::jlimit(0, 3, static_cast<int>(pos));
        int idx2 = juce::jlimit(0, 4, idx1 + 1);
        float frac = pos - idx1;

        f1 = formants[idx1][0] * (1.0f - frac) + formants[idx2][0] * frac;
        f2 = formants[idx1][1] * (1.0f - frac) + formants[idx2][1] * frac;
        f3 = formants[idx1][2] * (1.0f - frac) + formants[idx2][2] * frac;
    }

    // ★ Phase 82: Extended formant table (5 formants for VocalFilter)
    static void getExtendedFormantFrequencies(float vowel, float& f1, float& f2, float& f3, float& f4, float& f5) {
        // Extended formant table with F4 and F5
        const float formants[5][5] = {
            { 730.0f, 1090.0f, 2440.0f, 3400.0f, 4500.0f },  // A
            { 530.0f, 1840.0f, 2480.0f, 3600.0f, 4700.0f },  // E
            { 270.0f, 2290.0f, 3010.0f, 3500.0f, 4500.0f },  // I
            { 570.0f,  840.0f, 2410.0f, 3400.0f, 4600.0f },  // O
            { 440.0f, 1020.0f, 2240.0f, 3200.0f, 4400.0f }   // U
        };

        float pos = vowel * 4.0f;
        int idx1 = juce::jlimit(0, 3, static_cast<int>(pos));
        int idx2 = juce::jlimit(0, 4, idx1 + 1);
        float frac = pos - idx1;

        f1 = formants[idx1][0] * (1.0f - frac) + formants[idx2][0] * frac;
        f2 = formants[idx1][1] * (1.0f - frac) + formants[idx2][1] * frac;
        f3 = formants[idx1][2] * (1.0f - frac) + formants[idx2][2] * frac;
        f4 = formants[idx1][3] * (1.0f - frac) + formants[idx2][3] * frac;
        f5 = formants[idx1][4] * (1.0f - frac) + formants[idx2][4] * frac;
    }

    static float getFormantPeakMagnitude(float freq, float centerFreq, float Q) {
        if (centerFreq <= 0.0f) return 0.0f;
        float r = freq / centerFreq;
        float denom = std::sqrt(std::pow(1.0f - r * r, 2.0f) + std::pow(r / Q, 2.0f));
        return (r / Q) / std::max(0.001f, denom);
    }

    // ======================================================================
    // SVF Processing
    // ======================================================================
    float processSVF(float x, Mode type, float cutoff, float res)
    {
        cutoff = juce::jmin(cutoff, (float)(sampleRate * 0.49));
        float g = fastTan(juce::MathConstants<float>::pi * cutoff / (float)sampleRate);
        float r = 1.0f / (0.1f + res * 2.0f);
        float k = 1.0f / (1.0f + 2.0f * r * g + g * g);

        float hp = (x - (2.0f * r + g) * s1 - s2) * k;
        float bp = g * hp + s1;
        float lp = g * bp + s2;

        s1 = g * hp + bp;
        s2 = g * bp + lp;

        if (std::abs(s1) < 1.0e-20f) s1 = 0.0f;
        if (std::abs(s2) < 1.0e-20f) s2 = 0.0f;

        switch (type) {
        case LowPass: return lp;
        case HighPass: return hp;
        case BandPass: return bp;
        case Notch: return x - bp;
        default: return lp;
        }
    }

    // ======================================================================
    // Moog Ladder Processing
    // ======================================================================
    float processMoog(float x, float cutoff, float res, float drive, float p1, float p2, float p3)
    {
        float smoothAlpha = 0.004f;
        smoothedCutoff += (cutoff - smoothedCutoff) * smoothAlpha;
        smoothedRes += (res - smoothedRes) * smoothAlpha;
        smoothedP1 += (p1 - smoothedP1) * smoothAlpha;
        smoothedP2 += (p2 - smoothedP2) * smoothAlpha;
        smoothedP3 += (p3 - smoothedP3) * smoothAlpha;

        if (random.nextFloat() < 0.002f) {
            driftTarget = (random.nextFloat() * 2.0f - 1.0f) * smoothedP3 * 0.10f;
        }
        driftCurrent += (driftTarget - driftCurrent) * 0.01f;

        float driftFactor = 1.0f + driftCurrent;
        float safeCutoff = smoothedCutoff * driftFactor;
        float maxFreq = (float)sampleRate * 0.49f;
        safeCutoff = juce::jlimit(20.0f, maxFreq, safeCutoff);

        float wc = juce::MathConstants<float>::pi * safeCutoff / (float)sampleRate;
        float g = fastTan(wc);
        g = juce::jlimit(0.0f, 20.0f, g);

        float G = g / (1.0f + g);
        float oneMinusG = 1.0f - G;

        float k = (smoothedRes / 10.0f) * 3.98f;

        if (safeCutoff > (float)sampleRate * 0.40f) {
            float damp = 1.0f - ((safeCutoff - (float)sampleRate * 0.40f) / ((float)sampleRate * 0.09f));
            k *= juce::jlimit(0.0f, 1.0f, damp);
        }

        float makeupGain = 1.0f + (k * 0.5f * smoothedP2);
        float input = x * makeupGain;

        float S0 = z[0] * oneMinusG;
        float S1 = z[1] * oneMinusG;
        float S2 = z[2] * oneMinusG;
        float S3 = z[3] * oneMinusG;

        float G2 = G * G;
        float G3 = G2 * G;
        float G4 = G3 * G;

        float Sigma = G3 * S0 + G2 * S1 + G * S2 + S3;
        float u = (input - k * Sigma) / (1.0f + k * G4);

        float y0 = G * u + S0;
        z[0] = 2.0f * y0 - z[0];

        float y1 = G * y0 + S1;
        z[1] = 2.0f * y1 - z[1];

        float y2 = G * y1 + S2;
        z[2] = 2.0f * y2 - z[2];

        float y3 = G * y2 + S3;
        z[3] = 2.0f * y3 - z[3];

        for (int i = 0; i < 4; ++i) {
            if (std::abs(z[i]) < 1.0e-20f) z[i] = 0.0f;
        }

        return y3 * 0.707f;
    }

    // ======================================================================
    // Peak Filter (Parametric EQ) - Phase 79.1: Gain corrected
    // ======================================================================
    float processPeak(float x, float cutoff, float res, float p1, float p2)
    {
        cutoff = juce::jlimit(20.0f, (float)(sampleRate * 0.49f), cutoff);

        // P1 = Gain (-12dB to +12dB instead of 0 to +24dB)
        float gainDb = (p1 - 0.5f) * 24.0f;  // -12 to +12 dB
        float bandwidth = 0.1f + p2 * 3.9f;

        float Q = 1.41f / bandwidth;
        Q = juce::jlimit(0.1f, 20.0f, Q);

        float omega = 2.0f * juce::MathConstants<float>::pi * cutoff / (float)sampleRate;
        float cosOmega = std::cos(omega);
        float sinOmega = std::sin(omega);
        float A = std::pow(10.0f, gainDb / 40.0f);
        float alpha = sinOmega / (2.0f * Q);

        float b0 = 1.0f + alpha * A;
        float b1 = -2.0f * cosOmega;
        float b2 = 1.0f - alpha * A;
        float a0 = 1.0f + alpha / A;
        float a1 = -2.0f * cosOmega;
        float a2 = 1.0f - alpha / A;

        b0 /= a0; b1 /= a0; b2 /= a0;
        a1 /= a0; a2 /= a0;

        float output = b0 * x + peakState1;
        peakState1 = b1 * x - a1 * output + peakState2;
        peakState2 = b2 * x - a2 * output;

        if (std::abs(peakState1) < 1.0e-20f) peakState1 = 0.0f;
        if (std::abs(peakState2) < 1.0e-20f) peakState2 = 0.0f;

        // Removed resonance boost - Peak filter shouldn't have additional boost
        return output;
    }

    // ======================================================================
    // Comb Filter
    // ======================================================================
    float processComb(float x, float cutoff, float res, float p1, float p2, float p3)
    {
        if (combBuffer.empty()) return x;

        float baseDelayMs = 1000.0f / juce::jlimit(20.0f, 10000.0f, cutoff);
        float delayMultiplier = 0.5f + p1 * 1.5f;
        float delayMs = baseDelayMs * delayMultiplier;
        delayMs = juce::jlimit(0.1f, 50.0f, delayMs);

        float delaySamples = delayMs * (float)sampleRate / 1000.0f;
        int delayInt = static_cast<int>(delaySamples);
        float frac = delaySamples - (float)delayInt;

        delayInt = juce::jlimit(1, (int)combBuffer.size() - 2, delayInt);

        int bufSize = (int)combBuffer.size();
        int readPos1 = (combWritePos - delayInt + bufSize) % bufSize;
        int readPos2 = (combWritePos - delayInt - 1 + bufSize) % bufSize;

        float delayed = combBuffer[readPos1] * (1.0f - frac) + combBuffer[readPos2] * frac;

        float feedback = (p2 * 2.0f - 1.0f) * 0.95f;
        feedback *= (res / 5.0f);
        feedback = juce::jlimit(-0.98f, 0.98f, feedback);

        float polarity = (p3 > 0.5f) ? -1.0f : 1.0f;

        float output = x + feedback * polarity * delayed;

        combBuffer[combWritePos] = fastTanh(output * 0.5f) * 2.0f;
        combWritePos = (combWritePos + 1) % bufSize;

        return output;
    }

    // ======================================================================
    // Oberheim SEM Filter
    // ======================================================================
    float processOberheim(float x, float cutoff, float res, float p1, float p2)
    {
        cutoff = juce::jlimit(20.0f, (float)(sampleRate * 0.49f), cutoff);

        float warmInput = x;
        if (p2 > 0.01f) {
            float warmDrive = 1.0f + p2 * 2.0f;
            warmInput = fastTanh(x * warmDrive) / warmDrive;
        }

        float g = fastTan(juce::MathConstants<float>::pi * cutoff / (float)sampleRate);
        float R = 1.0f / (2.0f * juce::jlimit(0.5f, 10.0f, res));
        float d = 1.0f / (1.0f + 2.0f * R * g + g * g);

        float hp = (warmInput - (2.0f * R + g) * obState1 - obState2) * d;
        float v1 = g * hp;
        float bp = v1 + obState1;
        obState1 = bp + v1;

        float v2 = g * bp;
        float lp = v2 + obState2;
        obState2 = lp + v2;

        float notch = lp + hp;

        float output;
        float mode = p1;

        if (mode < 0.5f) {
            float blend = mode * 2.0f;
            output = lp * (1.0f - blend) + notch * blend;
        }
        else {
            float blend = (mode - 0.5f) * 2.0f;
            output = notch * (1.0f - blend) + hp * blend;
        }

        if (std::abs(obState1) < 1.0e-20f) obState1 = 0.0f;
        if (std::abs(obState2) < 1.0e-20f) obState2 = 0.0f;

        if (p2 > 0.5f) {
            float postWarm = (p2 - 0.5f) * 2.0f;
            output = output * (1.0f - postWarm * 0.1f) + fastTanh(output * 1.5f) * postWarm * 0.1f;
        }

        return output;
    }

    // ======================================================================
    // Flanger
    // ======================================================================
    float processFlanger(float x, float cutoff, float res, float p1, float p2, float p3)
    {
        if (flangerBuffer.empty()) return x;

        float lfoRate = 0.01f + p1 * p1 * 9.99f;

        float lfoIncrement = lfoRate / (float)sampleRate;
        flangerLfoPhase += lfoIncrement;
        if (flangerLfoPhase >= 1.0f) flangerLfoPhase -= 1.0f;

        float lfoValue = 0.5f + 0.5f * std::sin(2.0f * juce::MathConstants<float>::pi * flangerLfoPhase);

        float baseDelayMs = 1.0f + (1.0f - cutoff / 20000.0f) * 9.0f;
        baseDelayMs = juce::jlimit(0.5f, 15.0f, baseDelayMs);

        float depth = p2;
        float modulatedDelayMs = baseDelayMs * (1.0f - depth * 0.8f + depth * 0.8f * lfoValue);
        modulatedDelayMs = juce::jlimit(0.1f, 20.0f, modulatedDelayMs);

        float delaySamples = modulatedDelayMs * (float)sampleRate / 1000.0f;
        int delayInt = static_cast<int>(delaySamples);
        float frac = delaySamples - (float)delayInt;

        delayInt = juce::jlimit(1, (int)flangerBuffer.size() - 2, delayInt);

        int bufSize = (int)flangerBuffer.size();
        int readPos1 = (flangerWritePos - delayInt + bufSize) % bufSize;
        int readPos2 = (flangerWritePos - delayInt - 1 + bufSize) % bufSize;
        float delayed = flangerBuffer[readPos1] * (1.0f - frac) + flangerBuffer[readPos2] * frac;

        float feedback = (p3 * 2.0f - 1.0f) * 0.9f;
        feedback *= (res / 5.0f);
        feedback = juce::jlimit(-0.95f, 0.95f, feedback);

        float output = x + depth * delayed;

        flangerBuffer[flangerWritePos] = fastTanh((x + feedback * delayed) * 0.7f) / 0.7f;
        flangerWritePos = (flangerWritePos + 1) % bufSize;

        // ★ Phase 79.1: Reduced wet gain to match other filters
        return x * (1.0f - depth * 0.3f) + output * (0.3f + depth * 0.4f) * 0.85f;
    }

    // ======================================================================
    // Diffuser (Allpass Cascade with Delay)
    // ★ Phase 85: Added short delay for more audible effect
    // ======================================================================
    float processDiffuser(float x, float cutoff, float res, float p1, float p2, float p3)
    {
        // P1 = Diffusion Amount (0-1): How much smearing/blur
        // P2 = Stages (0-1): 2-8 allpass stages
        // P3 = Modulation (0-1): LFO modulation of allpass coefficients

        float diffusion = p1;
        int numStages = 2 + static_cast<int>(p2 * 6.0f);  // 2-8 stages
        float modulation = p3;

        // ★ Phase 85: Add short delay (5-30ms based on cutoff)
        // Lower cutoff = longer delay = more "room" feel
        // Higher cutoff = shorter delay = tighter sound
        float delayMs = 5.0f + (1.0f - cutoff / 20000.0f) * 25.0f;  // 5-30ms
        delayMs *= (0.5f + diffusion * 0.5f);  // Scale by diffusion amount
        int delaySamples = static_cast<int>(delayMs * 0.001f * sampleRate);

        if (!diffuserDelayBuffer.empty()) {
            delaySamples = juce::jlimit(1, (int)diffuserDelayBuffer.size() - 1, delaySamples);
        }
        else {
            delaySamples = 1;
        }

        // Read from delay buffer
        float delayed = 0.0f;
        if (!diffuserDelayBuffer.empty()) {
            int readPos = (diffuserDelayWritePos - delaySamples + (int)diffuserDelayBuffer.size())
                % (int)diffuserDelayBuffer.size();
            delayed = diffuserDelayBuffer[readPos];
        }

        // ★ Phase 83.4: Improved coefficient calculation
        float normalizedCutoff = cutoff / 20000.0f;
        float baseCoeff = 0.2f + 0.7f * normalizedCutoff;
        baseCoeff *= (0.3f + diffusion * 0.7f);
        baseCoeff = juce::jlimit(0.1f, 0.95f, baseCoeff);

        // Resonance affects feedback intensity between stages
        float spread = res / 10.0f;
        float feedbackBoost = 1.0f + spread * 0.5f;

        // ★ Phase 85: Mix input with delayed signal before allpass cascade
        float delayMix = 0.3f + diffusion * 0.4f;  // 30-70% delay mix
        float output = x * (1.0f - delayMix) + delayed * delayMix;

        for (int i = 0; i < numStages && i < 8; ++i) {
            float stageOffset = (float)i / 7.0f;
            float stageCoeff = baseCoeff * (0.6f + 0.4f * stageOffset);

            // Alternating coefficient signs for better diffusion
            if (i % 2 == 1) stageCoeff *= -1.0f;

            // Spread creates variation between stages
            stageCoeff *= feedbackBoost * (0.8f + spread * 0.4f * ((i % 3) / 2.0f));

            // LFO modulation for movement
            if (modulation > 0.01f) {
                float modPhase = flangerLfoPhase + stageOffset * 0.5f;
                if (modPhase >= 1.0f) modPhase -= 1.0f;
                float modValue = std::sin(2.0f * juce::MathConstants<float>::pi * modPhase);
                stageCoeff *= (1.0f + modulation * 0.25f * modValue);
            }

            stageCoeff = juce::jlimit(-0.97f, 0.97f, stageCoeff);

            // Allpass filter: y = coeff * x + state; state = x - coeff * y
            float y = stageCoeff * output + diffuserState[i];
            diffuserState[i] = output - stageCoeff * y;

            if (std::abs(diffuserState[i]) < 1.0e-20f) diffuserState[i] = 0.0f;

            output = y;
        }

        // ★ Phase 85: Write to delay buffer (after allpass for feedback character)
        if (!diffuserDelayBuffer.empty()) {
            // Add some feedback from output back into delay
            float feedbackAmount = res / 20.0f;  // 0-0.5 based on resonance
            feedbackAmount = juce::jlimit(0.0f, 0.6f, feedbackAmount);
            diffuserDelayBuffer[diffuserDelayWritePos] = x + output * feedbackAmount;
            diffuserDelayWritePos = (diffuserDelayWritePos + 1) % (int)diffuserDelayBuffer.size();
        }

        // ★ Phase 85: Enhanced wet/dry mix
        float wetAmount = 0.4f + diffusion * 0.5f;  // 0.4 to 0.9
        return x * (1.0f - wetAmount) + output * wetAmount;
    }

    // ======================================================================
    // Steiner-Parker Filter
    // ======================================================================
    float processSteinerParker(float x, float cutoff, float res, float p1, float p2, float drive)
    {
        cutoff = juce::jlimit(20.0f, (float)(sampleRate * 0.49f), cutoff);

        float aggression = p2;
        float inputSignal = x;
        if (aggression > 0.01f || drive > 0.01f) {
            float totalDrive = 1.0f + aggression * 3.0f + drive * 2.0f;
            inputSignal = fastTanh(x * totalDrive) / totalDrive;
            inputSignal += aggression * 0.1f * fastTanh(x * 5.0f) * 0.2f;
        }

        float g = fastTan(juce::MathConstants<float>::pi * cutoff / (float)sampleRate);
        float Q = 0.5f + res * 2.5f;
        float R = 1.0f / (2.0f * Q);
        float d = 1.0f / (1.0f + 2.0f * R * g + g * g);

        float hp = (inputSignal - (2.0f * R + g) * spState1 - spState2) * d;
        float v1 = g * hp;
        float bp = v1 + spState1;
        spState1 = bp + v1;

        float v2 = g * bp;
        float lp = v2 + spState2;
        spState2 = lp + v2;

        if (std::abs(spState1) < 1.0e-20f) spState1 = 0.0f;
        if (std::abs(spState2) < 1.0e-20f) spState2 = 0.0f;

        float mode = p1;
        float output;

        if (mode < 0.5f) {
            float blend = mode * 2.0f;
            output = lp * (1.0f - blend) + bp * blend;
        }
        else {
            float blend = (mode - 0.5f) * 2.0f;
            output = bp * (1.0f - blend) + hp * blend;
        }

        if (aggression > 0.3f) {
            float postAgg = (aggression - 0.3f) / 0.7f;
            output = output * (1.0f - postAgg * 0.2f) + fastTanh(output * 2.0f) * postAgg * 0.2f;
        }

        if (res > 3.0f) {
            output *= 1.0f + (res - 3.0f) * 0.05f;
        }

        return output;
    }

    // ======================================================================
    // ★ Phase 73: Diode Ladder Filter (TB-303 / EMS VCS3 style)
    // ======================================================================
    float processDiodeLadder(float x, float cutoff, float res, float p1, float p2, float p3, float drive)
    {
        // P1 = Saturation (0-1): Diode clipping intensity
        // P2 = Asymmetry (0-1): Even/odd harmonic balance
        // P3 = Feedback Character (0-1): 0=soft/warm, 1=hard/aggressive

        cutoff = juce::jlimit(20.0f, (float)(sampleRate * 0.49f), cutoff);

        float saturation = p1;
        float asymmetry = p2;
        float fbChar = p3;  // Renamed for clarity

        // Pre-filter saturation (diode clipping simulation)
        float inputSignal = x;
        if (saturation > 0.01f || drive > 0.01f) {
            float totalSat = 1.0f + saturation * 4.0f + drive * 2.0f;

            // Asymmetric clipping (more even harmonics)
            if (asymmetry > 0.01f) {
                float posClip = fastTanh(inputSignal * totalSat * (1.0f + asymmetry * 0.5f));
                float negClip = fastTanh(inputSignal * totalSat * (1.0f - asymmetry * 0.3f));
                inputSignal = (inputSignal > 0.0f) ? posClip : negClip;
                inputSignal /= totalSat;
            }
            else {
                inputSignal = fastTanh(inputSignal * totalSat) / totalSat;
            }
        }

        // Diode ladder coefficients (slightly different from Moog)
        float wc = juce::MathConstants<float>::pi * cutoff / (float)sampleRate;
        float g = fastTan(wc);
        g = juce::jlimit(0.0f, 15.0f, g);  // Tighter limit than Moog

        float G = g / (1.0f + g);
        float oneMinusG = 1.0f - G;

        // ★ Phase 83.4: Continuous feedback character (was step-based)
        // fbChar 0 = soft (0.6x resonance), fbChar 1 = aggressive (1.4x resonance)
        float fbMultiplier = 0.6f + fbChar * 0.8f;  // Range: 0.6 to 1.4
        float k = (res / 10.0f) * 3.5f * fbMultiplier;
        k = juce::jlimit(0.0f, 3.95f, k);

        // Nyquist damping
        if (cutoff > (float)sampleRate * 0.40f) {
            float damp = 1.0f - ((cutoff - (float)sampleRate * 0.40f) / ((float)sampleRate * 0.09f));
            k *= juce::jlimit(0.0f, 1.0f, damp);
        }

        // 4-stage ladder with diode-style nonlinearity
        float S0 = diodeZ[0] * oneMinusG;
        float S1 = diodeZ[1] * oneMinusG;
        float S2 = diodeZ[2] * oneMinusG;
        float S3 = diodeZ[3] * oneMinusG;

        float G2 = G * G;
        float G3 = G2 * G;
        float G4 = G3 * G;

        float Sigma = G3 * S0 + G2 * S1 + G * S2 + S3;

        // ★ Phase 83.4: Continuous feedback waveshaping blend
        // fbChar 0 = sine-like (warm), fbChar 1 = tanh (aggressive)
        float softFb = std::sin(Sigma * 1.5f) / 1.5f;
        float hardFb = fastTanh(Sigma * (1.5f + fbChar)) / (1.5f + fbChar);
        float fbSignal = softFb * (1.0f - fbChar) + hardFb * fbChar;

        float u = (inputSignal - k * fbSignal) / (1.0f + k * G4);

        // Process through 4 stages with per-stage diode saturation
        float y0 = G * u + S0;
        if (saturation > 0.3f) y0 = fastTanh(y0 * 1.5f) / 1.5f;
        diodeZ[0] = 2.0f * y0 - diodeZ[0];

        float y1 = G * y0 + S1;
        if (saturation > 0.5f) y1 = fastTanh(y1 * 1.3f) / 1.3f;
        diodeZ[1] = 2.0f * y1 - diodeZ[1];

        float y2 = G * y1 + S2;
        if (saturation > 0.7f) y2 = fastTanh(y2 * 1.2f) / 1.2f;
        diodeZ[2] = 2.0f * y2 - diodeZ[2];

        float y3 = G * y2 + S3;
        diodeZ[3] = 2.0f * y3 - diodeZ[3];

        // Denormal protection
        for (int i = 0; i < 4; ++i) {
            if (std::abs(diodeZ[i]) < 1.0e-20f) diodeZ[i] = 0.0f;
        }

        // ★ Phase 83.3: Output gain
        float output = y3 * 1.69f;
        if (asymmetry > 0.5f) {
            output += (y2 - y3) * (asymmetry - 0.5f) * 0.34f;
        }

        return output;
    }

    // ======================================================================
    // ★ Phase 73: Formant Filter (Vowel A-E-I-O-U)
    // ======================================================================
    float processFormant(float x, float cutoff, float res, float p1, float p2, float p3)
    {
        // P1 = Vowel (0-1): A(0) - E(0.25) - I(0.5) - O(0.75) - U(1.0)
        // P2 = Gender (0-1): 0=male, 0.5=neutral, 1=female (frequency shift)
        // P3 = Sharpness (0-1): Formant resonance/Q

        float vowel = p1;
        float genderShift = 0.7f + p2 * 0.6f;  // 0.7x (male) to 1.3x (female)
        float sharpness = 2.0f + p3 * 8.0f;  // Q = 2 to 10

        // Get interpolated formant frequencies
        float f1Base, f2Base, f3Base;
        getFormantFrequencies(vowel, f1Base, f2Base, f3Base);

        // Apply cutoff scaling and gender shift
        float freqScale = (cutoff / 1000.0f) * genderShift;
        float f1 = f1Base * freqScale;
        float f2 = f2Base * freqScale;
        float f3 = f3Base * freqScale;

        // Clamp to valid range
        float maxFreq = (float)sampleRate * 0.49f;
        f1 = juce::jlimit(50.0f, maxFreq, f1);
        f2 = juce::jlimit(100.0f, maxFreq, f2);
        f3 = juce::jlimit(200.0f, maxFreq, f3);

        // Process 3 parallel bandpass filters
        float out1 = processBandpass(x, f1, sharpness, 0);
        float out2 = processBandpass(x, f2, sharpness * 0.8f, 1);
        float out3 = processBandpass(x, f3, sharpness * 0.6f, 2);

        // ★ Phase 79.1: Reduced gains significantly (was way too loud)
        // Bandpass outputs are already boosted at resonance, so use lower coefficients
        float output = out1 * 0.4f + out2 * 0.25f + out3 * 0.15f;

        // Add some of the original signal for body
        float dryMix = 0.2f;
        output = output * (1.0f - dryMix) + x * dryMix;

        // Remove resonance boost - formant already has enough gain
        return output;
    }

    // Helper: Process a single bandpass for formant
    float processBandpass(float x, float freq, float Q, int stageIndex)
    {
        float g = fastTan(juce::MathConstants<float>::pi * freq / (float)sampleRate);
        float R = 1.0f / (2.0f * Q);
        float d = 1.0f / (1.0f + 2.0f * R * g + g * g);

        float hp = (x - (2.0f * R + g) * formantS1[stageIndex] - formantS2[stageIndex]) * d;
        float v1 = g * hp;
        float bp = v1 + formantS1[stageIndex];
        formantS1[stageIndex] = bp + v1;

        float v2 = g * bp;
        float lp = v2 + formantS2[stageIndex];
        formantS2[stageIndex] = lp + v2;

        // Denormal protection
        if (std::abs(formantS1[stageIndex]) < 1.0e-20f) formantS1[stageIndex] = 0.0f;
        if (std::abs(formantS2[stageIndex]) < 1.0e-20f) formantS2[stageIndex] = 0.0f;

        return bp;  // Bandpass output
    }

    // ======================================================================
    // ★ Phase 75: Phaser (6-stage Allpass with LFO) - Stabilized
    // ======================================================================
    float processPhaser(float x, float cutoff, float res, float p1, float p2, float p3)
    {
        // P1 = LFO Rate (0.1Hz ~ 10Hz)
        // P2 = Depth (modulation depth)
        // P3 = Feedback (0 ~ 0.85)

        // Input safety
        if (std::isnan(x) || std::isinf(x)) x = 0.0f;
        x = juce::jlimit(-10.0f, 10.0f, x);

        // LFO Rate: 0.1Hz to 10Hz
        float lfoRate = 0.1f + p1 * 9.9f;
        float depth = p2;
        float feedback = p3 * 0.85f;  // Reduced max feedback for stability

        // Update LFO phase
        phaserLfoPhase += lfoRate / (float)sampleRate;
        if (phaserLfoPhase >= 1.0f) phaserLfoPhase -= 1.0f;

        // LFO value (sine wave, 0 to 1)
        float lfoValue = 0.5f + 0.5f * std::sin(2.0f * juce::MathConstants<float>::pi * phaserLfoPhase);

        // Calculate modulated cutoff frequency
        // Sweep range based on depth (narrower range for stability)
        float minFreq = cutoff * 0.5f;
        float maxFreq = juce::jmin(cutoff * 3.0f, (float)(sampleRate * 0.4f));
        float modFreq = minFreq + (maxFreq - minFreq) * lfoValue * depth;
        modFreq = juce::jlimit(100.0f, (float)(sampleRate * 0.4f), modFreq);

        // Input with feedback (apply feedback limiting)
        float fbSignal = phaserFeedback;
        if (std::isnan(fbSignal) || std::isinf(fbSignal)) fbSignal = 0.0f;
        fbSignal = juce::jlimit(-5.0f, 5.0f, fbSignal);

        float input = x + fbSignal * feedback;
        input = juce::jlimit(-10.0f, 10.0f, input);

        // Process through 6 allpass stages using stable 1st-order allpass
        float allpassOut = input;

        // Each stage has slightly different frequency for richer sound
        const float stageMultipliers[6] = { 0.7f, 0.85f, 1.0f, 1.15f, 1.3f, 1.5f };

        for (int i = 0; i < 6; ++i) {
            // Calculate per-stage frequency
            float stageFreq = modFreq * stageMultipliers[i];
            stageFreq = juce::jlimit(50.0f, (float)(sampleRate * 0.45f), stageFreq);

            // 1st-order allpass coefficient: a = (1 - wc) / (1 + wc)
            // where wc = tan(pi * fc / fs)
            float wc = fastTan(juce::MathConstants<float>::pi * stageFreq / (float)sampleRate);
            float a = (1.0f - wc) / (1.0f + wc);
            a = juce::jlimit(-0.98f, 0.98f, a);  // Ensure stability

            // 1st-order allpass: y[n] = a * x[n] + x[n-1] - a * y[n-1]
            // Simplified: y = a * (x - state) + state; state = y
            // Actually using: y = -a * x + state; state = x + a * y (Direct Form II)
            float temp = allpassOut - a * phaserState[i];
            float y = a * temp + phaserState[i];
            phaserState[i] = temp;

            // Safety check per stage
            if (std::isnan(phaserState[i]) || std::isinf(phaserState[i])) {
                phaserState[i] = 0.0f;
                y = allpassOut;  // Pass through on error
            }

            // Denormal protection
            if (std::abs(phaserState[i]) < 1.0e-15f) phaserState[i] = 0.0f;

            allpassOut = y;
        }

        // Store feedback with safety
        phaserFeedback = allpassOut;
        if (std::isnan(phaserFeedback) || std::isinf(phaserFeedback)) {
            phaserFeedback = 0.0f;
        }
        phaserFeedback = juce::jlimit(-5.0f, 5.0f, phaserFeedback);
        if (std::abs(phaserFeedback) < 1.0e-15f) phaserFeedback = 0.0f;

        // Mix dry and wet (50/50 for classic phaser sound)
        float output = (x + allpassOut) * 0.5f;

        // Note: Resonance-based gain is handled by global AGC in process()

        // Final safety clamp
        output = juce::jlimit(-10.0f, 10.0f, output);

        if (std::isnan(output) || std::isinf(output)) {
            // Reset all states on catastrophic failure
            for (int i = 0; i < 6; ++i) phaserState[i] = 0.0f;
            phaserFeedback = 0.0f;
            return x * 0.5f;  // Return attenuated dry signal
        }

        return output;
    }

    // ======================================================================
    // ★ Phase 76: Wasp Filter (EDP Wasp style 12dB with CMOS distortion)
    // ======================================================================
    float processWasp(float x, float cutoff, float res, float p1, float p2, float drive)
    {
        // P1 = LP/HP Mix (0=LP, 0.5=Notch-ish, 1=HP)
        // P2 = Grit (CMOS distortion amount)

        cutoff = juce::jlimit(20.0f, (float)(sampleRate * 0.45f), cutoff);

        float morphLP_HP = p1;
        float grit = p2;

        // CMOS-style input distortion (asymmetric soft clip)
        float inputSignal = x;
        if (grit > 0.01f || drive > 0.01f) {
            float totalGrit = 1.0f + grit * 3.0f + drive * 2.0f;
            // Asymmetric clipping characteristic of CMOS
            float shaped = fastTanh(inputSignal * totalGrit);
            // Add slight asymmetry
            if (inputSignal > 0.0f) {
                shaped *= 1.0f + grit * 0.1f;
            }
            inputSignal = shaped / totalGrit;
        }

        // 2-pole SVF (Wasp doesn't self-oscillate, limit resonance)
        float g = fastTan(juce::MathConstants<float>::pi * cutoff / (float)sampleRate);
        float k = 2.0f - juce::jmin(res * 0.18f, 1.8f);  // Limited resonance

        float hp = (inputSignal - (k + g) * waspState1 - waspState2) / (1.0f + k * g + g * g);
        float v1 = g * hp;
        float bp = v1 + waspState1;
        waspState1 = bp + v1;

        float v2 = g * bp;
        float lp = v2 + waspState2;
        waspState2 = lp + v2;

        // Denormal protection
        if (std::abs(waspState1) < 1.0e-15f) waspState1 = 0.0f;
        if (std::abs(waspState2) < 1.0e-15f) waspState2 = 0.0f;

        // Output distortion (subtle CMOS character)
        if (grit > 0.3f) {
            lp = fastTanh(lp * 1.2f) / 1.2f;
            hp = fastTanh(hp * 1.2f) / 1.2f;
        }

        // LP/HP morphing
        float output = lp * (1.0f - morphLP_HP) + hp * morphLP_HP;

        return output;
    }

    // ======================================================================
    // ★ Phase 77: Korg35 Filter - Phase 79.2: Completely rewritten for stability
    // ======================================================================
    float processKorg35(float x, float cutoff, float res, float p1, float p2, float drive)
    {
        // P1 = Mode (0-0.5=LP, 0.5-1=HP)
        // P2 = Saturation

        // Input safety
        if (std::isnan(x) || std::isinf(x)) x = 0.0f;
        x = juce::jlimit(-5.0f, 5.0f, x);

        cutoff = juce::jlimit(20.0f, (float)(sampleRate * 0.40f), cutoff);

        bool isHighpass = (p1 > 0.5f);
        float saturation = p2;

        // Pre-saturation (internal saturation, different from global drive)
        float inputSignal = x;
        if (saturation > 0.01f) {
            float satGain = 1.0f + saturation * 2.5f;
            inputSignal = fastTanh(inputSignal * satGain) / satGain;
        }

        // Simple 2-pole SVF implementation (much more stable than original Korg35)
        float g = fastTan(juce::MathConstants<float>::pi * cutoff / (float)sampleRate);

        // ★ Phase 79.2: Resonance that doesn't explode at res=0
        // Q = 0.5 (no resonance) to Q = 20 (high resonance)
        float Q = 0.5f + res * 1.5f;  // res 0-10 -> Q 0.5-15.5
        Q = juce::jlimit(0.5f, 15.0f, Q);
        float k = 1.0f / Q;

        // State variable filter
        float hp = (inputSignal - (2.0f * k + g) * korg35Lpf1 - korg35Lpf2) / (1.0f + 2.0f * k * g + g * g);
        float bp = g * hp + korg35Lpf1;
        korg35Lpf1 = g * hp + bp;
        float lp = g * bp + korg35Lpf2;
        korg35Lpf2 = g * bp + lp;

        // Denormal protection
        if (std::abs(korg35Lpf1) < 1.0e-15f) korg35Lpf1 = 0.0f;
        if (std::abs(korg35Lpf2) < 1.0e-15f) korg35Lpf2 = 0.0f;
        if (std::abs(korg35Lpf1) > 50.0f) korg35Lpf1 = 0.0f;
        if (std::abs(korg35Lpf2) > 50.0f) korg35Lpf2 = 0.0f;

        float output = isHighpass ? hp : lp;

        // Post saturation for MS-20 character
        if (saturation > 0.3f) {
            output = fastTanh(output * 1.5f) / 1.5f;
        }

        // Safety
        if (std::isnan(output) || std::isinf(output)) {
            korg35Lpf1 = korg35Lpf2 = 0.0f;
            return 0.0f;
        }

        return output;
    }

    // ======================================================================
    // ★ Phase 78: Lowpass Gate (Buchla 292 style) - Phase 79.1: Fixed
    // ======================================================================
    float processLowpassGate(float x, float cutoff, float res, float p1, float p2, float p3)
    {
        // P1 = Mode (0=VCA, 0.5=LPF, 1=Both)
        // P2 = Decay Time (Vactrol release speed) - now more dramatic range
        // P3 = Trig (0=envelope follower, 0.5+=manual open amount)

        // Mode selection
        int mode = static_cast<int>(p1 * 2.99f);  // 0, 1, or 2

        // ★ Phase 79.1: More dramatic decay range (20ms to 2000ms)
        float decayMs = 20.0f + p2 * p2 * 1980.0f;  // Quadratic for better control
        float attackMs = 5.0f;  // Fixed fast attack

        float attackCoeff = 1.0f - std::exp(-1.0f / (attackMs * 0.001f * (float)sampleRate));
        float releaseCoeff = std::exp(-1.0f / (decayMs * 0.001f * (float)sampleRate));

        // ★ Phase 79.1: Trig behavior fixed
        // Trig = 0: Pure envelope follower from input
        // Trig > 0: Manual control (higher = more open)
        float cv;
        if (p3 < 0.02f) {
            // Envelope follower mode - tracks input amplitude
            float inputLevel = std::abs(x) * 2.0f;  // Boost sensitivity
            inputLevel = juce::jmin(inputLevel, 1.0f);
            cv = inputLevel;
        }
        else {
            // Manual mode - Trig directly controls gate amount
            cv = p3;
        }

        // Process Vactrol (asymmetric slew)
        if (cv > lpgVactrolState) {
            // Attack: fast rise
            lpgVactrolState += attackCoeff * (cv - lpgVactrolState);
        }
        else {
            // Release: slow decay based on Decay knob
            lpgVactrolState = lpgVactrolState * releaseCoeff + cv * (1.0f - releaseCoeff) * 0.1f;
        }
        lpgVactrolState = juce::jlimit(0.0f, 1.0f, lpgVactrolState);

        // Control voltage with minimum floor to prevent complete silence
        float control = lpgVactrolState;
        float minLevel = 0.001f;  // Very small minimum to allow sound through

        // VCA processing - control affects amplitude
        float vcaGain = minLevel + control * (1.0f - minLevel);
        float vcaOut = x * vcaGain;

        // LPF processing (cutoff controlled by vactrol)
        float lpfCutoff = 20.0f + control * (cutoff - 20.0f);
        lpfCutoff = juce::jlimit(20.0f, (float)(sampleRate * 0.45f), lpfCutoff);

        float g = fastTan(juce::MathConstants<float>::pi * lpfCutoff / (float)sampleRate);
        float k = 2.0f - 2.0f * juce::jmin(res * 0.1f, 0.9f);

        float hp = (x - (k + g) * lpgState1 - lpgState2) / (1.0f + k * g + g * g);
        float bp = g * hp + lpgState1;
        lpgState1 = bp + g * hp;
        float lp = g * bp + lpgState2;
        lpgState2 = lp + g * bp;

        // Denormal protection
        if (std::abs(lpgState1) < 1.0e-15f) lpgState1 = 0.0f;
        if (std::abs(lpgState2) < 1.0e-15f) lpgState2 = 0.0f;
        if (std::abs(lpgVactrolState) < 1.0e-15f) lpgVactrolState = 0.0f;

        // Output based on mode
        float output;
        if (mode == 0) {
            output = vcaOut;  // VCA only - amplitude control
        }
        else if (mode == 1) {
            output = lp;  // LPF only - filter tracks but no VCA
        }
        else {
            output = lp * vcaGain;  // Both - filter + VCA
        }

        return output;
    }

    // ======================================================================
    // ★ Phase 79: Resonator (Karplus-Strong string resonance)
    // ======================================================================
    float processResonator(float x, float cutoff, float res, float p1, float p2, float p3)
    {
        // Cutoff = Resonant frequency (pitch)
        // P1 = Damping (0=short decay, 1=long decay)
        // P2 = Brightness (loop filter cutoff)
        // P3 = Mix (dry/wet)

        if (resonatorBuffer.empty()) return x;

        float freq = juce::jlimit(20.0f, 5000.0f, cutoff);
        float damping = 0.5f + p1 * 0.499f;  // 0.5 to 0.999
        float brightness = p2;
        float mix = p3;

        // Calculate delay length for target frequency
        float delaySamples = (float)sampleRate / freq;
        int delayInt = static_cast<int>(delaySamples);
        float delayFrac = delaySamples - delayInt;

        // Ensure delay fits in buffer
        delayInt = juce::jlimit(1, (int)resonatorBuffer.size() - 2, delayInt);

        // Read from delay line with linear interpolation
        int readPos1 = (resonatorWritePos - delayInt + (int)resonatorBuffer.size()) % (int)resonatorBuffer.size();
        int readPos2 = (readPos1 - 1 + (int)resonatorBuffer.size()) % (int)resonatorBuffer.size();

        float delayed1 = resonatorBuffer[readPos1];
        float delayed2 = resonatorBuffer[readPos2];
        float delayed = delayed1 * (1.0f - delayFrac) + delayed2 * delayFrac;

        // Loop filter (simple 1-pole lowpass for string damping)
        float lpfCoeff = 0.2f + brightness * 0.75f;  // Higher = brighter
        resonatorLpfState = resonatorLpfState + lpfCoeff * (delayed - resonatorLpfState);

        // Feedback with damping
        float feedback = resonatorLpfState * damping;

        // Add input and write to delay line
        float toDelay = x * 0.5f + feedback;
        toDelay = juce::jlimit(-10.0f, 10.0f, toDelay);  // Safety
        resonatorBuffer[resonatorWritePos] = toDelay;

        // Advance write position
        resonatorWritePos = (resonatorWritePos + 1) % (int)resonatorBuffer.size();

        // Denormal protection
        if (std::abs(resonatorLpfState) < 1.0e-15f) resonatorLpfState = 0.0f;

        // Mix dry and wet
        float output = x * (1.0f - mix) + feedback * mix;

        // Resonance adds gain at fundamental
        if (res > 1.0f) {
            output *= (1.0f + (res - 1.0f) * 0.05f);
        }

        return output;
    }

    // ======================================================================
    // ★ Phase 80: Ring Modulator
    // ======================================================================
    float processRingMod(float x, float cutoff, float res, float p1, float p2, float p3)
    {
        // Cutoff influences carrier frequency range
        // P1 = Carrier Frequency (relative to cutoff)
        // P2 = Mix (dry/wet)
        // P3 = Rectify amount (0=bipolar, 1=full rectify)

        // Carrier frequency: 20Hz to cutoff*2
        float carrierFreq = 20.0f + p1 * cutoff * 2.0f;
        carrierFreq = juce::jlimit(20.0f, 10000.0f, carrierFreq);

        float mix = p2;
        float rectify = p3;

        // Update carrier oscillator phase
        ringModPhase += carrierFreq / (float)sampleRate;
        if (ringModPhase >= 1.0f) ringModPhase -= 1.0f;

        // Carrier waveform (sine)
        float carrier = std::sin(2.0f * juce::MathConstants<float>::pi * ringModPhase);

        // Optional rectification (adds even harmonics)
        if (rectify > 0.01f) {
            float rectCarrier = std::abs(carrier);
            carrier = carrier * (1.0f - rectify) + rectCarrier * rectify;
        }

        // Ring modulation (multiplication)
        float ringMod = x * carrier;

        // Mix dry and wet
        float output = x * (1.0f - mix) + ringMod * mix;

        // Resonance can add subtle feedback-like effect
        if (res > 1.0f) {
            output *= (1.0f + (res - 1.0f) * 0.05f);
        }

        return output;
    }

    // ======================================================================
    // ★ Phase 81: Bitcrusher
    // ======================================================================
    float processBitcrusher(float x, float cutoff, float res, float p1, float p2, float p3)
    {
        // Cutoff affects pre-filter (smoothing before crush)
        // P1 = Bit Depth (1-16 bits)
        // P2 = Downsample Rate (1-50x)
        // P3 = Mix (dry/wet)

        float bits = 1.0f + p1 * 15.0f;  // 1 to 16 bits
        float downsampleRate = 1.0f + p2 * 49.0f;  // 1x to 50x
        float mix = p3;

        // Optional pre-filter (cutoff acts as anti-aliasing)
        float filtered = x;
        if (cutoff < 10000.0f) {
            // Simple 1-pole lowpass
            float coeff = 1.0f - std::exp(-2.0f * juce::MathConstants<float>::pi * cutoff / (float)sampleRate);
            bitcrushHeldSample = bitcrushHeldSample + coeff * (x - bitcrushHeldSample);
            filtered = bitcrushHeldSample;
        }

        // Sample rate reduction (sample & hold)
        bitcrushSampleCounter++;
        int downsampleInt = static_cast<int>(downsampleRate);
        if (bitcrushSampleCounter >= downsampleInt) {
            bitcrushSampleCounter = 0;
            bitcrushHeldSample = filtered;
        }
        float crushed = bitcrushHeldSample;

        // Bit depth reduction (quantization)
        float levels = std::pow(2.0f, bits);
        crushed = std::floor(crushed * levels + 0.5f) / levels;

        // Mix dry and wet
        float output = x * (1.0f - mix) + crushed * mix;

        // Resonance adds gain
        if (res > 1.0f) {
            output *= (1.0f + (res - 1.0f) * 0.08f);
        }

        return output;
    }

    // ======================================================================
    // ★ Phase 82: Vocal Filter (Advanced 5-formant)
    // ======================================================================
    float processVocalFilter(float x, float cutoff, float res, float p1, float p2, float p3)
    {
        // Cutoff scales all formant frequencies
        // P1 = Vowel Morph (A-E-I-O-U)
        // P2 = Character (0=male, 0.5=neutral, 1=female)
        // P3 = Sharpness (formant Q)

        float vowelMorph = p1;
        float character = 0.7f + p2 * 0.6f;  // 0.7x (male) to 1.3x (female)
        float sharpness = 2.0f + p3 * 10.0f;  // Q = 2 to 12

        // Get 5 formant frequencies
        float f1, f2, f3, f4, f5;
        getExtendedFormantFrequencies(vowelMorph, f1, f2, f3, f4, f5);

        // Apply cutoff scaling and character shift
        float scale = (cutoff / 1000.0f) * character;
        f1 *= scale; f2 *= scale; f3 *= scale; f4 *= scale; f5 *= scale;

        // Clamp to valid range
        float maxFreq = (float)sampleRate * 0.45f;
        f1 = juce::jlimit(50.0f, maxFreq, f1);
        f2 = juce::jlimit(100.0f, maxFreq, f2);
        f3 = juce::jlimit(200.0f, maxFreq, f3);
        f4 = juce::jlimit(300.0f, maxFreq, f4);
        f5 = juce::jlimit(400.0f, maxFreq, f5);

        // Process 5 parallel bandpass filters
        float out1 = processVocalBandpass(x, f1, sharpness, 0);
        float out2 = processVocalBandpass(x, f2, sharpness * 0.8f, 1);
        float out3 = processVocalBandpass(x, f3, sharpness * 0.6f, 2);
        float out4 = processVocalBandpass(x, f4, sharpness * 0.4f, 3);
        float out5 = processVocalBandpass(x, f5, sharpness * 0.3f, 4);

        // Mix with decreasing gains
        float output = out1 * 0.35f + out2 * 0.25f + out3 * 0.15f + out4 * 0.10f + out5 * 0.05f;

        // Add dry signal for body
        output = output * 0.85f + x * 0.15f;

        return output;
    }

    // Helper: Process a single bandpass for VocalFilter
    float processVocalBandpass(float x, float freq, float Q, int stageIndex)
    {
        float g = fastTan(juce::MathConstants<float>::pi * freq / (float)sampleRate);
        float R = 1.0f / (2.0f * Q);
        float d = 1.0f / (1.0f + 2.0f * R * g + g * g);

        float hp = (x - (2.0f * R + g) * vocalS1[stageIndex] - vocalS2[stageIndex]) * d;
        float v1 = g * hp;
        float bp = v1 + vocalS1[stageIndex];
        vocalS1[stageIndex] = bp + v1;

        float v2 = g * bp;
        float lp = v2 + vocalS2[stageIndex];
        vocalS2[stageIndex] = lp + v2;

        // Denormal protection
        if (std::abs(vocalS1[stageIndex]) < 1.0e-15f) vocalS1[stageIndex] = 0.0f;
        if (std::abs(vocalS2[stageIndex]) < 1.0e-15f) vocalS2[stageIndex] = 0.0f;

        return bp;
    }
};