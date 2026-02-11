/*
  ==============================================================================
    ChimeraVoice.h
    Phase 85: Color Mode - User Filter Choice Respected

    Changes:
    - Removed forced BandPass in Color Mode
    - User's filter selection is now used (creative flexibility)
    - Resonance boost (4x) still applied for ringing effect

    Previous: Phase 69.3 - Filter Soft Reset for Click Prevention
  ==============================================================================
*/

#pragma once
#include <JuceHeader.h>
#include "HentaiFilter.h"

struct VoiceFilterSettings {
    float cutoff, res, drive;
    int type;
    float p1 = 0.0f, p2 = 0.0f, p3 = 0.0f;
};

class ChimeraVoice
{
public:
    ChimeraVoice() { reset(); }

    void prepare(double sampleRate) {
        currentSampleRate = sampleRate;
        for (int i = 0; i < 4; ++i) {
            filtersL[i].prepare(sampleRate);
            filtersR[i].prepare(sampleRate);
        }
        currentGainL = 1.0f;
        currentGainR = 1.0f;

        // ★ Phase 69.1: Initialize envelope
        envGain = 0.0f;
        inRelease = false;
        updateEnvelopeCoeffs(1.0f, 100.0f); // Default: 1ms attack, 100ms release
    }

    void reset() {
        for (int i = 0; i < 4; ++i) {
            filtersL[i].reset();
            filtersR[i].reset();
            smoothedCutoffL[i] = 1000.0f;
            smoothedCutoffR[i] = 1000.0f;
        }
        isActive = false;
        isAlwaysActive = false;
        currentNote = -1;
        baseFrequency = 0.0f;
        currentGainL = 1.0f;
        currentGainR = 1.0f;

        // ★ Phase 69.1: Reset envelope
        envGain = 0.0f;
        inRelease = false;
    }

    // ★ Phase 69.2: Update envelope coefficients from ms values
    // Added minimum attack/release times to prevent clicks
    void updateEnvelopeCoeffs(float attackMs, float releaseMs) {
        // ★ Phase 69.2: Enforce minimum times to prevent clicks
        // Minimum 2ms attack, 5ms release for click-free operation
        const float minAttackMs = 2.0f;
        const float minReleaseMs = 5.0f;

        attackMs = juce::jmax(minAttackMs, attackMs);
        releaseMs = juce::jmax(minReleaseMs, releaseMs);

        float attackSamples = (attackMs / 1000.0f) * (float)currentSampleRate;
        float releaseSamples = (releaseMs / 1000.0f) * (float)currentSampleRate;

        // Using simple one-pole smoothing approach
        attackCoeff = 1.0f - std::exp(-2.2f / attackSamples);
        releaseCoeff = std::exp(-2.2f / releaseSamples);
    }

    // ★ Phase 58: Always Active Logic
    void setAlwaysActive(bool active) {
        isAlwaysActive = active;
        if (active) {
            isActive = true;
            baseFrequency = 0.0f;
            currentGainL = 1.0f;
            currentGainR = 1.0f;
            envGain = 1.0f;  // Normal mode: full gain immediately
            inRelease = false;
        }
        else {
            if (currentNote == -1) isActive = false;
        }
    }

    void setHarmonicRatio(float ratio) { harmonicRatio = ratio; }
    void setSemitoneOffset(int semitones) { semitoneOffset = semitones; }

    void noteOn(int noteNumber, float velocity) {
        currentNote = noteNumber;
        isActive = true;
        baseFrequency = 440.0f * std::pow(2.0f, (float)(noteNumber - 69) / 12.0f);

        // ★ Phase 69.3: Soft reset filters to prevent clicks on note transitions
        // This decays the filter state gradually instead of hard resetting
        for (int i = 0; i < 4; ++i) {
            filtersL[i].softReset(0.95f);  // 50% decay - balance between click prevention and responsiveness
            filtersR[i].softReset(0.95f);
        }

        // ★ Phase 69.2: Don't reset smoothed cutoffs - preserve filter state to prevent clicks
        // Only reset if voice was completely inactive (envGain near zero)
        if (envGain < 0.01f) {
            for (int i = 0; i < 4; ++i) {
                smoothedCutoffL[i] = baseFrequency;
                smoothedCutoffR[i] = baseFrequency;
            }
        }

        // ★ Phase 69.2: Start attack phase but DON'T reset envGain
        // This allows legato-style retriggering without clicks
        inRelease = false;
        // envGain continues from current value, providing smooth retrigger
    }

    void noteOff() {
        if (!isAlwaysActive) {
            // ★ Phase 69.1: Enter release phase instead of immediately stopping
            inRelease = true;
            currentNote = -1;
            // isActive stays true until envelope reaches zero
        }
    }

    bool isVoiceActive() const { return isActive; }

    void process(float inL, float inR, float& outL, float& outR,
        float morphPos, const std::array<VoiceFilterSettings, 4>& settings, bool isColorMode)
    {
        if (!isActive) { outL = 0.0f; outR = 0.0f; return; }

        // ★ Phase 69.1: Process AR Envelope (Color Mode only)
        if (isColorMode) {
            if (inRelease) {
                // Release phase: decay toward zero
                envGain *= releaseCoeff;

                // Check if envelope has finished
                if (envGain < 0.0001f) {
                    envGain = 0.0f;
                    isActive = false;
                    outL = 0.0f;
                    outR = 0.0f;
                    return;
                }
            }
            else {
                // Attack phase: rise toward 1.0
                envGain += attackCoeff * (1.0f - envGain);
                envGain = juce::jmin(1.0f, envGain);
            }
        }
        else {
            // Normal Mode: no envelope, full gain
            envGain = 1.0f;
        }

        // Morph Weights Calculation
        float scaledMorph = morphPos * 3.0f;
        int index = (int)scaledMorph;
        if (index > 2) index = 2;
        if (morphPos >= 1.0f) index = 2;
        if (morphPos <= 0.0f) index = 0;

        float weights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        if (scaledMorph < 1.0f) { weights[1] = scaledMorph; weights[0] = 1.0f - weights[1]; }
        else if (scaledMorph < 2.0f) { weights[2] = scaledMorph - 1.0f; weights[1] = 1.0f - weights[2]; }
        else { weights[3] = scaledMorph - 2.0f; weights[2] = 1.0f - weights[3]; }

        // Edge cases
        if (scaledMorph >= 3.0f) { weights[3] = 1.0f; weights[2] = 0.0f; }
        if (scaledMorph <= 0.0f) { weights[0] = 1.0f; weights[1] = 0.0f; }

        float tempL = 0.0f;
        float tempR = 0.0f;
        float currentTargetFreq = 0.0f;

        // Color Mode Pitch Tracking
        if (isColorMode && baseFrequency > 20.0f) {
            float intervalMulti = std::pow(2.0f, (float)semitoneOffset / 12.0f);
            currentTargetFreq = baseFrequency * intervalMulti * harmonicRatio;
        }

        for (int i = 0; i < 4; ++i) {
            if (weights[i] < 0.001f) continue;

            const auto& s = settings[i];
            float targetCutoff = s.cutoff;
            float targetRes = s.res;
            int targetType = s.type;

            if (isColorMode && baseFrequency > 20.0f) {
                targetCutoff = currentTargetFreq;
                // Boost resonance in Color Mode for ringing
                targetRes = s.res * 4.0f;

                // ★ Phase 85: REMOVED forced BandPass - user choice is now respected
                // Previously: targetType = HentaiFilter::Mode::BandPass;
                // Now the user's selected filter type is used for creative flexibility
                // Note: Some filter types work better than others in Color Mode:
                // - BandPass, Peak, Resonator, Formant, VocalFilter: Excellent
                // - Moog, DiodeLadder, Korg35: Good with high resonance
                // - LowpassGate, Comb: Interesting experimental results
                // - LP, HP, Notch: May be quieter, but can be creative
            }

            // ★ Phase 69.2: Enhanced smoothing to prevent clicks
            // Slower smoothing in Color Mode for click-free frequency transitions
            float smoothAlpha = isColorMode ? 0.02f : 0.1f;
            smoothedCutoffL[i] += (targetCutoff - smoothedCutoffL[i]) * smoothAlpha;
            smoothedCutoffR[i] += (targetCutoff - smoothedCutoffR[i]) * smoothAlpha;

            // ★ Phase 69.2: Denormal protection for filter states
            if (std::abs(smoothedCutoffL[i]) < 1.0e-10f) smoothedCutoffL[i] = 20.0f;
            if (std::abs(smoothedCutoffR[i]) < 1.0e-10f) smoothedCutoffR[i] = 20.0f;

            tempL += filtersL[i].process(inL, targetType, smoothedCutoffL[i], targetRes, s.drive, s.p1, s.p2, s.p3) * weights[i];
            tempR += filtersR[i].process(inR, targetType, smoothedCutoffR[i], targetRes, s.drive, s.p1, s.p2, s.p3) * weights[i];
        }

        // ★ Phase 69.1: Apply envelope gain
        outL = tempL * envGain;
        outR = tempR * envGain;
    }

private:
    double currentSampleRate = 44100.0;
    std::array<HentaiFilter, 4> filtersL;
    std::array<HentaiFilter, 4> filtersR;

    bool isActive = false;
    bool isAlwaysActive = false;
    int currentNote = -1;
    float baseFrequency = 0.0f;
    float harmonicRatio = 1.0f;
    int semitoneOffset = 0;
    float currentGainL = 1.0f;
    float currentGainR = 1.0f;

    float smoothedCutoffL[4] = { 1000.0f, 1000.0f, 1000.0f, 1000.0f };
    float smoothedCutoffR[4] = { 1000.0f, 1000.0f, 1000.0f, 1000.0f };

    // ★ Phase 69.1: AR Envelope variables
    float envGain = 0.0f;           // Current envelope level (0.0 ~ 1.0)
    float attackCoeff = 0.01f;      // Attack coefficient
    float releaseCoeff = 0.9999f;   // Release coefficient  
    bool inRelease = false;         // True when in release phase
};