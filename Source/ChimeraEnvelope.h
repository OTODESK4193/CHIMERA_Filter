/*
  ==============================================================================
    ChimeraEnvelope.h
    (Sensitivity Extreme)
  ==============================================================================
*/

#pragma once
#include <JuceHeader.h>
#include <cmath>

class ChimeraEnvelope
{
public:
    ChimeraEnvelope() { reset(); }

    void prepare(double newSampleRate)
    {
        sampleRate = newSampleRate;
        reset();
    }

    void reset() { currentLevel = 0.0f; }

    void setParameters(float attackMs, float releaseMs)
    {
        float attSec = juce::jmax(0.1f, attackMs) / 1000.0f;
        float relSec = juce::jmax(0.1f, releaseMs) / 1000.0f;
        attackCoeff = std::exp(-1.0f / (attSec * sampleRate));
        releaseCoeff = std::exp(-1.0f / (relSec * sampleRate));
    }

    float process(float input)
    {
        // ★感度調整: 入力を8倍にブースト
        // これで微小な入力でもMAXまで振り切れるようになります
        float absInput = std::abs(input) * 8.0f;

        if (absInput > currentLevel)
            currentLevel = attackCoeff * currentLevel + (1.0f - attackCoeff) * absInput;
        else
            currentLevel = releaseCoeff * currentLevel + (1.0f - releaseCoeff) * absInput;

        // 1.0を超えないようにクランプ
        return juce::jmin(currentLevel, 1.0f);
    }

private:
    double sampleRate = 44100.0;
    float currentLevel = 0.0f;
    float attackCoeff = 0.0f;
    float releaseCoeff = 0.0f;
};