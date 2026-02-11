/*
  ==============================================================================
    ChimeraADSR.h
    (ADSR Envelope Engine)

    Phase 45: Fixed duplicate #pragma once
  ==============================================================================
*/

#pragma once
#include <JuceHeader.h>

class ChimeraADSR
{
public:
    ChimeraADSR() {}

    void prepare(double newSampleRate)
    {
        adsr.setSampleRate(newSampleRate);
    }

    void setParameters(float attack, float decay, float sustain, float release, float curve)
    {
        juce::ADSR::Parameters params;
        params.attack = attack;
        params.decay = decay;
        params.sustain = sustain;
        params.release = release;
        adsr.setParameters(params);
        // Curve parameter is unused in standard JUCE ADSR but kept for interface compatibility
    }

    void noteOn() { adsr.noteOn(); }
    void noteOff() { adsr.noteOff(); }

    float getNextSample() { return adsr.getNextSample(); }

private:
    juce::ADSR adsr;
};