/*
  ==============================================================================
    PluginProcessor.h
    Phase 83.2: Oversampling Fix (2x/4x separate instances)

    Changes:
    - Separate oversampler2x and oversampler4x instances
    - Panic function for emergency reset
  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include "HentaiFilter.h"
#include "MorphPhysics.h"
#include "ChimeraVoice.h"

class CHIMERA_FilterAudioProcessor : public juce::AudioProcessor
{
public:
    CHIMERA_FilterAudioProcessor();
    ~CHIMERA_FilterAudioProcessor() override;

    void prepareToPlay(double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

#ifndef JucePlugin_PreferredChannelConfigurations
    bool isBusesLayoutSupported(const BusesLayout& layouts) const override;
#endif

    void processBlock(juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    const juce::String getName() const override;
    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram(int index) override;
    const juce::String getProgramName(int index) override;
    void changeProgramName(int index, const juce::String& newName) override;

    void getStateInformation(juce::MemoryBlock& destData) override;
    void setStateInformation(const void* data, int sizeInBytes) override;

    // ★ Phase 83.1: Panic function - reset all voices and filters
    void panic();

    juce::UndoManager undoManager;
    juce::AudioProcessorValueTreeState apvts;

    // Visual Feedback (atomic)
    std::atomic<float> currentVisualMorph{ 0.0f };
    std::atomic<float> currentVisualMix{ 1.0f };
    std::atomic<float> currentVisualMaster{ 1.0f };

    std::atomic<float> currentVisualCutoff0{ 1000.0f };
    std::atomic<float> currentVisualCutoff1{ 1000.0f };
    std::atomic<float> currentVisualCutoff2{ 1000.0f };
    std::atomic<float> currentVisualCutoff3{ 1000.0f };

    std::atomic<float> currentVisualRes0{ 0.707f };
    std::atomic<float> currentVisualRes1{ 0.707f };
    std::atomic<float> currentVisualRes2{ 0.707f };
    std::atomic<float> currentVisualRes3{ 0.707f };

    std::atomic<float> currentVisualDrive0{ 0.0f };
    std::atomic<float> currentVisualDrive1{ 0.0f };
    std::atomic<float> currentVisualDrive2{ 0.0f };
    std::atomic<float> currentVisualDrive3{ 0.0f };

    std::atomic<float>* getVisualCutoffPtr(int i) {
        std::atomic<float>* ptrs[] = { &currentVisualCutoff0, &currentVisualCutoff1, &currentVisualCutoff2, &currentVisualCutoff3 };
        return (i >= 0 && i < 4) ? ptrs[i] : nullptr;
    }
    std::atomic<float>* getVisualResPtr(int i) {
        std::atomic<float>* ptrs[] = { &currentVisualRes0, &currentVisualRes1, &currentVisualRes2, &currentVisualRes3 };
        return (i >= 0 && i < 4) ? ptrs[i] : nullptr;
    }
    std::atomic<float>* getVisualDrivePtr(int i) {
        std::atomic<float>* ptrs[] = { &currentVisualDrive0, &currentVisualDrive1, &currentVisualDrive2, &currentVisualDrive3 };
        return (i >= 0 && i < 4) ? ptrs[i] : nullptr;
    }

    std::array<int, 6> activeNoteSlots = { -1, -1, -1, -1, -1, -1 };

private:
    juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();
    void updateParameterCache();
    void handleMidi(const juce::MidiBuffer& midiMessages);

    std::array<VoiceFilterSettings, 4> settingsCache;
    static constexpr int maxVoices = 18;
    std::array<ChimeraVoice, maxVoices> voices;
    MorphPhysics myPhysics;

    float smoothedMorph = 0.0f;

    // Smoothers
    juce::LinearSmoothedValue<float> smoothMix;
    juce::LinearSmoothedValue<float> smoothMaster;

    // ★ Phase 69: New Smoothers for Input/Output
    juce::LinearSmoothedValue<float> smoothInputGain;
    juce::LinearSmoothedValue<float> smoothWetHighShelf;

    // ★ Phase 69: Input HPF State (1st order, stereo)
    float inputHpfStateL = 0.0f;
    float inputHpfStateR = 0.0f;

    // ★ Phase 69.4: Input Smoother for Color Mode (prevents clicks from synth input)
    float colorInputStateL = 0.0f;
    float colorInputStateR = 0.0f;

    // ★ Phase 69: WET Envelope State (AR envelope, stereo)
    float wetEnvStateL = 0.0f;
    float wetEnvStateR = 0.0f;

    // ★ Phase 69: High Shelf Filter State (1st order, stereo)
    float highShelfStateL = 0.0f;
    float highShelfStateR = 0.0f;

    // ★ Phase 69: Limiter State
    float limiterGainL = 1.0f;
    float limiterGainR = 1.0f;

    // Parameter Pointers
    std::atomic<float>* morphPosPtr = nullptr;
    std::atomic<float>* morphModePtr = nullptr;
    std::atomic<float>* physicsDepthPtr = nullptr;
    std::atomic<float>* colorModePtr = nullptr;
    std::atomic<float>* snapModePtr = nullptr;
    std::atomic<float>* mixPtr = nullptr;
    std::atomic<float>* masterVolPtr = nullptr;

    // Physics Parameters (A is reused smoothing_time, B is new)
    std::atomic<float>* smoothingTimePtr = nullptr; // Param A (Primary)
    std::atomic<float>* physicsParamBPtr = nullptr; // Param B (Secondary)

    std::atomic<float>* harmRatioPtr = nullptr;
    std::atomic<float>* intervalSemiPtr = nullptr;
    std::atomic<float>* volRootPtr = nullptr;
    std::atomic<float>* volHarmPtr = nullptr;
    std::atomic<float>* volIntPtr = nullptr;

    // ★ Phase 69: New Parameter Pointers
    std::atomic<float>* inputGainPtr = nullptr;
    std::atomic<float>* inputHpfPtr = nullptr;
    std::atomic<float>* wetAttackPtr = nullptr;
    std::atomic<float>* wetReleasePtr = nullptr;
    std::atomic<float>* wetHighShelfPtr = nullptr;
    std::atomic<float>* limiterOnPtr = nullptr;

    std::atomic<float>* cutPtrs[4] = { nullptr };
    std::atomic<float>* resPtrs[4] = { nullptr };
    std::atomic<float>* drivePtrs[4] = { nullptr };
    std::atomic<float>* typePtrs[4] = { nullptr };
    std::atomic<float>* param1Ptrs[4] = { nullptr };
    std::atomic<float>* param2Ptrs[4] = { nullptr };
    std::atomic<float>* param3Ptrs[4] = { nullptr };

    // ★ Phase 83.2: Oversampling (separate instances for 2x and 4x)
    std::atomic<float>* oversamplingPtr = nullptr;
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampler2x;  // order 1 = 2x
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampler4x;  // order 2 = 4x
    int currentOversamplingFactor = 1;
    int cachedBlockSize = 512;

    // Cached sample rate for DSP
    double cachedSampleRate = 44100.0;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(CHIMERA_FilterAudioProcessor)
};