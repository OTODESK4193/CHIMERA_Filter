/*
  ==============================================================================
    PluginProcessor.cpp
    Phase 83.2: Oversampling Fix & Improvements

    Changes:
    - Fixed 2x oversampling (was using 4x internally) - now uses separate 2x/4x instances
    - Panic function for emergency voice/filter reset
    - Latency reporting to DAW

    All Filter Types (21 total):
    0-3: SVF (LP, HP, BP, Notch)
    4: Moog Ladder
    5-12: Peak, Comb, Oberheim, Flanger, Diffuser, Steiner, Diode, Formant
    13-20: Phaser, Wasp, Korg35, LPGate, Resonator, RingMod, Bitcrush, Vocal

    Previous: Phase 83.1 - Oversampling 4x, Panic Function
  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

// ==============================================================================
// CONSTRUCTOR
// ==============================================================================
CHIMERA_FilterAudioProcessor::CHIMERA_FilterAudioProcessor()
#ifndef JucePlugin_PreferredChannelConfigurations
    : AudioProcessor(BusesProperties()
#if ! JucePlugin_IsMidiEffect
#if ! JucePlugin_IsSynth
        .withInput("Input", juce::AudioChannelSet::stereo(), true)
#endif
        .withOutput("Output", juce::AudioChannelSet::stereo(), true)
#endif
    ),
    apvts(*this, &undoManager, "Parameters", createParameterLayout())
#endif
{
    updateParameterCache();
}

CHIMERA_FilterAudioProcessor::~CHIMERA_FilterAudioProcessor()
{
    morphPosPtr = nullptr; morphModePtr = nullptr; physicsDepthPtr = nullptr;
    colorModePtr = nullptr; snapModePtr = nullptr; mixPtr = nullptr;
    masterVolPtr = nullptr;
    smoothingTimePtr = nullptr; physicsParamBPtr = nullptr;

    // ★ Phase 69: Reset new pointers
    inputGainPtr = nullptr;
    inputHpfPtr = nullptr;
    wetAttackPtr = nullptr;
    wetReleasePtr = nullptr;
    wetHighShelfPtr = nullptr;
    limiterOnPtr = nullptr;

    harmRatioPtr = nullptr; intervalSemiPtr = nullptr; volRootPtr = nullptr;
    volHarmPtr = nullptr; volIntPtr = nullptr;
    for (int i = 0; i < 4; ++i) {
        cutPtrs[i] = nullptr; resPtrs[i] = nullptr; drivePtrs[i] = nullptr;
        typePtrs[i] = nullptr; param1Ptrs[i] = nullptr; param2Ptrs[i] = nullptr; param3Ptrs[i] = nullptr;
    }
}

// ==============================================================================
// PARAMETER LAYOUT
// ==============================================================================
juce::AudioProcessorValueTreeState::ParameterLayout CHIMERA_FilterAudioProcessor::createParameterLayout()
{
    juce::AudioProcessorValueTreeState::ParameterLayout layout;
    juce::String suffixes[] = { "A", "B", "C", "D" };
    // ★ Phase 84: All 21 Filter Types (must match HentaiFilter.h enum)
    juce::StringArray typeNames;
    typeNames.add("LowPass (SVF)");   // 0
    typeNames.add("HighPass (SVF)");  // 1
    typeNames.add("BandPass (SVF)");  // 2
    typeNames.add("Notch (SVF)");     // 3
    typeNames.add("Moog Ladder");     // 4
    typeNames.add("Peak");            // 5
    typeNames.add("Comb");            // 6
    typeNames.add("Oberheim");        // 7
    typeNames.add("Flanger");         // 8
    typeNames.add("Diffuser");        // 9
    typeNames.add("Steiner");         // 10
    typeNames.add("Diode");           // 11
    typeNames.add("Formant");         // 12
    typeNames.add("Phaser");          // 13
    typeNames.add("Wasp");            // 14
    typeNames.add("Korg35");          // 15
    typeNames.add("LPG");             // 16
    typeNames.add("Resonator");       // 17
    typeNames.add("RingMod");         // 18
    typeNames.add("Bitcrusher");      // 19
    typeNames.add("Vocal");           // 20

    auto cutoffRange = juce::NormalisableRange<float>(20.0f, 20000.0f, 1.0f, 0.25f);
    auto addFilterParams = [&](juce::String suffix, float defCut, float defRes, float defDrv, int defType) {
        layout.add(std::make_unique<juce::AudioParameterFloat>("cutoff_" + suffix, "Cutoff " + suffix, cutoffRange, defCut));
        layout.add(std::make_unique<juce::AudioParameterFloat>("res_" + suffix, "Res " + suffix, 0.1f, 10.0f, defRes));
        layout.add(std::make_unique<juce::AudioParameterFloat>("drive_" + suffix, "Drive " + suffix, 0.0f, 1.0f, defDrv));
        layout.add(std::make_unique<juce::AudioParameterChoice>("type_" + suffix, "Type " + suffix, typeNames, defType));
        layout.add(std::make_unique<juce::AudioParameterFloat>("p1_" + suffix, "P1 " + suffix, 0.0f, 1.0f, 0.0f));
        layout.add(std::make_unique<juce::AudioParameterFloat>("p2_" + suffix, "P2 " + suffix, 0.0f, 1.0f, 0.0f));
        layout.add(std::make_unique<juce::AudioParameterFloat>("p3_" + suffix, "P3 " + suffix, 0.0f, 1.0f, 0.0f));
        };
    addFilterParams("A", 100.0f, 0.707f, 0.0f, 0);
    addFilterParams("B", 1000.0f, 2.0f, 0.2f, 1);
    addFilterParams("C", 5000.0f, 5.0f, 0.5f, 2);
    addFilterParams("D", 15000.0f, 0.707f, 0.8f, 3);

    layout.add(std::make_unique<juce::AudioParameterFloat>("morph_pos", "Morph", 0.0f, 1.0f, 0.0f));

    juce::StringArray morphModes;
    morphModes.add("Linear"); morphModes.add("Stepped"); morphModes.add("Elastic");
    morphModes.add("Sloth"); morphModes.add("Chaos"); morphModes.add("Bounce");
    layout.add(std::make_unique<juce::AudioParameterChoice>("morph_mode", "Physics", morphModes, 0));
    layout.add(std::make_unique<juce::AudioParameterFloat>("physics_depth", "Depth", 0.0f, 1.0f, 0.5f));

    layout.add(std::make_unique<juce::AudioParameterBool>("snap_mode", "PITCH SNAP", false));
    layout.add(std::make_unique<juce::AudioParameterBool>("color_mode", "COLOR MODE", false));

    // Physics Param A (Primary) - reused "smoothing_time"
    auto smoothRange = juce::NormalisableRange<float>(0.1f, 50.0f, 0.1f, 0.3f);
    layout.add(std::make_unique<juce::AudioParameterFloat>("smoothing_time", "Smooth", smoothRange, 5.0f));

    // Physics Param B (Secondary)
    layout.add(std::make_unique<juce::AudioParameterFloat>("physics_param_b", "Physics B", 0.0f, 1.0f, 0.5f));

    layout.add(std::make_unique<juce::AudioParameterFloat>("harm_ratio", "Harm Ratio", 1.0f, 8.0f, 2.0f));
    layout.add(std::make_unique<juce::AudioParameterInt>("interval_semi", "Interval", -24, 24, 7));
    layout.add(std::make_unique<juce::AudioParameterFloat>("vol_root", "Vol Root", 0.0f, 1.0f, 1.0f));
    layout.add(std::make_unique<juce::AudioParameterFloat>("vol_harm", "Vol Harm", 0.0f, 1.0f, 0.8f));
    layout.add(std::make_unique<juce::AudioParameterFloat>("vol_int", "Vol Int", 0.0f, 1.0f, 0.8f));

    layout.add(std::make_unique<juce::AudioParameterFloat>("mix", "Dry/Wet", 0.0f, 1.0f, 1.0f));
    layout.add(std::make_unique<juce::AudioParameterFloat>("master_vol", "Master Vol", 0.0f, 2.0f, 1.0f));
    layout.add(std::make_unique<juce::AudioParameterInt>("preset", "Preset", 0, 10, 0));

    // ★ Phase 69: New Parameters - Input Section
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        "input_gain", "Input Gain", 0.0f, 2.0f, 1.0f));

    auto hpfRange = juce::NormalisableRange<float>(20.0f, 500.0f, 1.0f, 0.5f);
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        "input_hpf", "Input HPF", hpfRange, 20.0f));

    // ★ Phase 69: New Parameters - WET Section
    auto attackRange = juce::NormalisableRange<float>(0.1f, 100.0f, 0.1f, 0.4f);
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        "wet_attack", "WET Attack", attackRange, 1.0f));

    auto releaseRange = juce::NormalisableRange<float>(10.0f, 1000.0f, 1.0f, 0.4f);
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        "wet_release", "WET Release", releaseRange, 100.0f));

    layout.add(std::make_unique<juce::AudioParameterFloat>(
        "wet_highshelf", "WET High", -12.0f, 12.0f, 0.0f));

    // ★ Phase 69: New Parameters - Output Section
    layout.add(std::make_unique<juce::AudioParameterBool>(
        "limiter_on", "Limiter", true));

    // ★ Phase 83: Oversampling Parameter (0=1x, 1=2x, 2=4x)
    layout.add(std::make_unique<juce::AudioParameterInt>(
        "oversampling", "Oversampling", 0, 2, 0));

    return layout;
}

void CHIMERA_FilterAudioProcessor::updateParameterCache()
{
    juce::String suffixes[] = { "A", "B", "C", "D" };
    morphPosPtr = apvts.getRawParameterValue("morph_pos");
    morphModePtr = apvts.getRawParameterValue("morph_mode");
    physicsDepthPtr = apvts.getRawParameterValue("physics_depth");
    colorModePtr = apvts.getRawParameterValue("color_mode");
    snapModePtr = apvts.getRawParameterValue("snap_mode");
    mixPtr = apvts.getRawParameterValue("mix");
    masterVolPtr = apvts.getRawParameterValue("master_vol");

    smoothingTimePtr = apvts.getRawParameterValue("smoothing_time");
    physicsParamBPtr = apvts.getRawParameterValue("physics_param_b");

    harmRatioPtr = apvts.getRawParameterValue("harm_ratio");
    intervalSemiPtr = apvts.getRawParameterValue("interval_semi");
    volRootPtr = apvts.getRawParameterValue("vol_root");
    volHarmPtr = apvts.getRawParameterValue("vol_harm");
    volIntPtr = apvts.getRawParameterValue("vol_int");

    // ★ Phase 69: Cache new parameter pointers
    inputGainPtr = apvts.getRawParameterValue("input_gain");
    inputHpfPtr = apvts.getRawParameterValue("input_hpf");
    wetAttackPtr = apvts.getRawParameterValue("wet_attack");
    wetReleasePtr = apvts.getRawParameterValue("wet_release");
    wetHighShelfPtr = apvts.getRawParameterValue("wet_highshelf");
    limiterOnPtr = apvts.getRawParameterValue("limiter_on");

    // ★ Phase 83: Oversampling pointer
    oversamplingPtr = apvts.getRawParameterValue("oversampling");

    for (int i = 0; i < 4; ++i) {
        cutPtrs[i] = apvts.getRawParameterValue("cutoff_" + suffixes[i]);
        resPtrs[i] = apvts.getRawParameterValue("res_" + suffixes[i]);
        drivePtrs[i] = apvts.getRawParameterValue("drive_" + suffixes[i]);
        typePtrs[i] = apvts.getRawParameterValue("type_" + suffixes[i]);
        param1Ptrs[i] = apvts.getRawParameterValue("p1_" + suffixes[i]);
        param2Ptrs[i] = apvts.getRawParameterValue("p2_" + suffixes[i]);
        param3Ptrs[i] = apvts.getRawParameterValue("p3_" + suffixes[i]);
    }
}

// ==============================================================================
// ★ Phase 70: Soft Limiter Function (moved outside loop for optimization)
// ==============================================================================
namespace {
    inline float softLimit(float x, float threshold, float knee) noexcept
    {
        float absX = std::abs(x);
        if (absX <= threshold - knee) {
            return x;
        }
        else if (absX >= threshold + knee) {
            return (x > 0.0f) ? threshold : -threshold;
        }
        else {
            // Soft knee region
            float t = (absX - (threshold - knee)) / (2.0f * knee);
            float gain = 1.0f - t * t * 0.5f;
            return x * gain;
        }
    }
}

// ==============================================================================
// PROCESS BLOCK
// ==============================================================================
void CHIMERA_FilterAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();
    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i) buffer.clear(i, 0, buffer.getNumSamples());

    int numSamples = buffer.getNumSamples();
    double sampleRate = getSampleRate();
    if (sampleRate < 8000.0) sampleRate = 44100.0;
    cachedSampleRate = sampleRate;

    // ★ Phase 70: Block-level parameter loading (instead of per-sample)
    float targetMorph = (morphPosPtr) ? morphPosPtr->load() : 0.0f;
    float targetMix = (mixPtr) ? mixPtr->load() : 1.0f;
    float targetMaster = (masterVolPtr) ? masterVolPtr->load() : 1.0f;
    int morphModeInt = (morphModePtr) ? (int)morphModePtr->load() : 0;
    MorphPhysics::Mode physMode = (MorphPhysics::Mode)juce::jlimit(0, 5, morphModeInt);

    // Physics Params
    float paramA = (smoothingTimePtr) ? smoothingTimePtr->load() : 5.0f;
    float paramB = (physicsParamBPtr) ? physicsParamBPtr->load() : 0.5f;

    float physicsDepthVal = (physicsDepthPtr) ? physicsDepthPtr->load() : 0.5f;
    bool isColorMode = (colorModePtr) ? (*colorModePtr > 0.5f) : false;
    bool snapOn = (snapModePtr) ? (*snapModePtr > 0.5f) : false;

    // ★ Phase 69: Load new parameters
    float inputGain = (inputGainPtr) ? inputGainPtr->load() : 1.0f;
    float inputHpfFreq = (inputHpfPtr) ? inputHpfPtr->load() : 20.0f;
    float wetAttackMs = (wetAttackPtr) ? wetAttackPtr->load() : 1.0f;
    float wetReleaseMs = (wetReleasePtr) ? wetReleasePtr->load() : 100.0f;
    float wetHighShelfDb = (wetHighShelfPtr) ? wetHighShelfPtr->load() : 0.0f;
    bool limiterOn = (limiterOnPtr) ? (limiterOnPtr->load() > 0.5f) : true;

    // ★ Phase 70: Pre-calculate DSP coefficients (block-level)
    // Input HPF coefficient (1st order highpass)
    float hpfCutoff = juce::jlimit(20.0f, 500.0f, inputHpfFreq);
    float hpfCoeff = 1.0f - std::exp(-2.0f * juce::MathConstants<float>::pi * hpfCutoff / (float)sampleRate);

    // High Shelf coefficient (simple 1st order shelf at 4kHz)
    float shelfFreq = 4000.0f;
    float shelfGain = std::pow(10.0f, wetHighShelfDb / 20.0f);
    float shelfCoeff = 1.0f - std::exp(-2.0f * juce::MathConstants<float>::pi * shelfFreq / (float)sampleRate);

    // ★ Phase 70: Limiter constants (pre-computed)
    const float limiterThreshold = 0.95f;
    const float limiterKnee = 0.1f;

    // ★ Phase 70: Check if high shelf is needed
    bool applyHighShelf = std::abs(wetHighShelfDb) > 0.1f;

    handleMidi(midiMessages);

    if (!isColorMode) {
        voices[0].setAlwaysActive(true);
        for (int v = 1; v < maxVoices; ++v) voices[v].setAlwaysActive(false);
    }
    else {
        for (int v = 0; v < maxVoices; ++v) voices[v].setAlwaysActive(false);
    }

    // --- Settings Cache & Visualizer Update ---
    for (int i = 0; i < 4; ++i) {
        if (cutPtrs[i]) {
            float baseCutoff = cutPtrs[i]->load();
            if (snapOn) {
                float semitones = 12.0f * std::log2(baseCutoff / 440.0f);
                float rounded = std::round(semitones);
                baseCutoff = 440.0f * std::pow(2.0f, rounded / 12.0f);
            }
            float physicsMod = (physicsDepthVal * 2.0f - 1.0f) * 0.2f;
            float modFactor = std::pow(2.0f, physicsMod * 2.0f);
            float finalCutoff = juce::jlimit(20.0f, 20000.0f, baseCutoff * modFactor);
            settingsCache[i].cutoff = finalCutoff;

            if (i == 0) currentVisualCutoff0.store(finalCutoff);
            else if (i == 1) currentVisualCutoff1.store(finalCutoff);
            else if (i == 2) currentVisualCutoff2.store(finalCutoff);
            else if (i == 3) currentVisualCutoff3.store(finalCutoff);
        }

        if (resPtrs[i]) {
            float resVal = resPtrs[i]->load();
            settingsCache[i].res = resVal;
            if (i == 0) currentVisualRes0.store(resVal);
            else if (i == 1) currentVisualRes1.store(resVal);
            else if (i == 2) currentVisualRes2.store(resVal);
            else if (i == 3) currentVisualRes3.store(resVal);
        }

        if (drivePtrs[i]) {
            float driveVal = drivePtrs[i]->load();
            settingsCache[i].drive = driveVal;
            if (i == 0) currentVisualDrive0.store(driveVal);
            else if (i == 1) currentVisualDrive1.store(driveVal);
            else if (i == 2) currentVisualDrive2.store(driveVal);
            else if (i == 3) currentVisualDrive3.store(driveVal);
        }

        if (typePtrs[i]) settingsCache[i].type = (int)typePtrs[i]->load();
        if (param1Ptrs[i]) settingsCache[i].p1 = param1Ptrs[i]->load();
        if (param2Ptrs[i]) settingsCache[i].p2 = param2Ptrs[i]->load();
        if (param3Ptrs[i]) settingsCache[i].p3 = param3Ptrs[i]->load();
    }

    smoothMix.setTargetValue(targetMix);
    smoothMaster.setTargetValue(targetMaster);
    smoothInputGain.setTargetValue(inputGain);

    float hRatio = (harmRatioPtr) ? harmRatioPtr->load() : 2.0f;
    int interval = (intervalSemiPtr) ? (int)intervalSemiPtr->load() : 7;
    float volR = (volRootPtr) ? volRootPtr->load() : 1.0f;
    float volH = (volHarmPtr) ? volHarmPtr->load() : 0.8f;
    float volI = (volIntPtr) ? volIntPtr->load() : 0.8f;

    if (isColorMode) {
        for (int i = 0; i < 6; ++i) {
            int base = i * 3;
            if (base < maxVoices) { voices[base].setHarmonicRatio(1.0f); voices[base].setSemitoneOffset(0); }
            if (base + 1 < maxVoices) { voices[base + 1].setHarmonicRatio(hRatio); voices[base + 1].setSemitoneOffset(interval); }
            if (base + 2 < maxVoices) { voices[base + 2].setHarmonicRatio(1.0f); voices[base + 2].setSemitoneOffset(interval); }
        }

        // ★ Phase 69.1: Update AR envelope coefficients for all voices
        for (int v = 0; v < maxVoices; ++v) {
            voices[v].updateEnvelopeCoeffs(wetAttackMs, wetReleaseMs);
        }
    }

    // ★ Phase 70: Cache active voice indices (avoid checking all 18 every sample)
    int activeVoiceIndices[maxVoices];
    int numActiveVoices = 0;
    for (int v = 0; v < maxVoices; ++v) {
        if (voices[v].isVoiceActive()) {
            activeVoiceIndices[numActiveVoices++] = v;
        }
    }

    // ★ Phase 70: Pre-compute voice levels for Color Mode
    float voiceLevels[3] = { volR, volH, volI };

    // ★ Phase 83.2: Get oversampling factor (0=1x, 1=2x, 2=4x)
    int osChoice = (oversamplingPtr) ? static_cast<int>(oversamplingPtr->load()) : 0;
    int osFactor = 1;
    juce::dsp::Oversampling<float>* activeOversampler = nullptr;
    if (osChoice == 1) {
        osFactor = 2;
        activeOversampler = oversampler2x.get();
    }
    else if (osChoice == 2) {
        osFactor = 4;
        activeOversampler = oversampler4x.get();
    }
    bool useOversampling = (osFactor > 1) && (activeOversampler != nullptr);

    // ★ Phase 83: Update voices' sample rate if oversampling factor changed
    if (osFactor != currentOversamplingFactor) {
        currentOversamplingFactor = osFactor;
        double effectiveRate = cachedSampleRate * static_cast<double>(osFactor);
        for (auto& voice : voices) {
            voice.prepare(effectiveRate);
            // Note: Don't call reset() to preserve filter state
        }
        // Report latency to DAW
        if (useOversampling) {
            setLatencySamples(static_cast<int>(activeOversampler->getLatencyInSamples()));
        }
        else {
            setLatencySamples(0);
        }
    }

    auto* channelL = buffer.getWritePointer(0);
    auto* channelR = (totalNumInputChannels > 1) ? buffer.getWritePointer(1) : nullptr;

    // ★ Phase 83.2: Create AudioBlock for oversampling
    juce::dsp::AudioBlock<float> block(buffer);
    juce::dsp::AudioBlock<float> workBlock = block;
    int workNumSamples = numSamples;

    // ★ Phase 83.2: Upsample if enabled (use selected oversampler)
    if (useOversampling) {
        workBlock = activeOversampler->processSamplesUp(block);
        workNumSamples = static_cast<int>(workBlock.getNumSamples());
    }

    // Get work pointers (may be oversampled buffer or original)
    float* workL = workBlock.getChannelPointer(0);
    float* workR = (workBlock.getNumChannels() > 1) ? workBlock.getChannelPointer(1) : nullptr;

    // ★ Phase 83: Process at (possibly higher) sample rate
    for (int sample = 0; sample < workNumSamples; ++sample)
    {
        // Scale smoothing for oversampled rate
        if (useOversampling && (sample % osFactor == 0)) {
            smoothedMorph = myPhysics.process(targetMorph, physMode, paramA, paramB);
        }
        else if (!useOversampling) {
            smoothedMorph = myPhysics.process(targetMorph, physMode, paramA, paramB);
        }

        float currentInputGain = smoothInputGain.getNextValue();

        // ★ Phase 69: Read input and apply Input Gain
        float inL = workL[sample] * currentInputGain;
        float inR = (workR) ? workR[sample] * currentInputGain : inL;

        // ★ Phase 69: Apply Input HPF (1st order highpass)
        // Only apply if not in Color Mode (Normal Mode only)
        if (!isColorMode) {
            // HPF: out = in - lowpassed(in)
            inputHpfStateL += hpfCoeff * (inL - inputHpfStateL);
            inputHpfStateR += hpfCoeff * (inR - inputHpfStateR);
            inL = inL - inputHpfStateL;
            inR = inR - inputHpfStateR;
        }
        else {
            // ★ Phase 69.4: Color Mode - Apply input smoothing to prevent clicks from synth transients
            const float colorSmoothCoeff = 0.3f;
            colorInputStateL += colorSmoothCoeff * (inL - colorInputStateL);
            colorInputStateR += colorSmoothCoeff * (inR - colorInputStateR);
            inL = colorInputStateL;
            inR = colorInputStateR;
        }

        float wetL = 0.0f;
        float wetR = 0.0f;

        // ★ Phase 70: Only iterate over active voices
        for (int i = 0; i < numActiveVoices; ++i) {
            int v = activeVoiceIndices[i];
            float vOutL = 0.0f, vOutR = 0.0f;
            voices[v].process(inL, inR, vOutL, vOutR, smoothedMorph, settingsCache, isColorMode);

            float level = 1.0f;
            if (isColorMode) {
                level = voiceLevels[v % 3];
            }
            wetL += vOutL * level;
            wetR += vOutR * level;
        }

        // ★ Phase 69: Apply WET High Shelf Filter
        if (applyHighShelf) {
            highShelfStateL += shelfCoeff * (wetL - highShelfStateL);
            highShelfStateR += shelfCoeff * (wetR - highShelfStateR);

            float highContentL = wetL - highShelfStateL;
            float highContentR = wetR - highShelfStateR;

            wetL = highShelfStateL + highContentL * shelfGain;
            wetR = highShelfStateR + highContentR * shelfGain;
        }

        float currentMix = smoothMix.getNextValue();
        float currentMaster = smoothMaster.getNextValue();

        float dryGain = 1.0f - currentMix;
        float wetGain = currentMix;

        // ★ Phase 82.1: Color Mode WET gain reduction (prevents too loud output)
        if (isColorMode) {
            wetGain *= 0.85f;
        }

        // ★ Phase 83: Use work buffer for dry signal reference
        float dryL = workL[sample] * currentInputGain;
        float dryR = (workR) ? workR[sample] * currentInputGain : dryL;

        float finalL = (dryL * dryGain) + (wetL * wetGain);
        float finalR = (dryR * dryGain) + (wetR * wetGain);

        finalL *= currentMaster;
        finalR *= currentMaster;

        // ★ Phase 70: Apply Limiter (using pre-defined function)
        if (limiterOn) {
            finalL = softLimit(finalL, limiterThreshold, limiterKnee);
            finalR = softLimit(finalR, limiterThreshold, limiterKnee);
        }
        else {
            // Hard clip safety even without limiter
            finalL = juce::jlimit(-2.0f, 2.0f, finalL);
            finalR = juce::jlimit(-2.0f, 2.0f, finalR);
        }

        // ★ Phase 83: Write to work buffer (may be oversampled)
        workL[sample] = finalL;
        if (workR) workR[sample] = finalR;
    }

    // ★ Phase 83.2: Downsample if oversampling was used
    if (useOversampling) {
        activeOversampler->processSamplesDown(block);
    }

    // ★ Phase 70: Unified denormal protection (end of block)
    // Only apply if states are very small
    if (std::abs(inputHpfStateL) < 1.0e-15f) inputHpfStateL = 0.0f;
    if (std::abs(inputHpfStateR) < 1.0e-15f) inputHpfStateR = 0.0f;
    if (std::abs(colorInputStateL) < 1.0e-15f) colorInputStateL = 0.0f;
    if (std::abs(colorInputStateR) < 1.0e-15f) colorInputStateR = 0.0f;
    if (std::abs(highShelfStateL) < 1.0e-15f) highShelfStateL = 0.0f;
    if (std::abs(highShelfStateR) < 1.0e-15f) highShelfStateR = 0.0f;

    currentVisualMorph.store(smoothedMorph);
    currentVisualMix.store(targetMix);
    currentVisualMaster.store(targetMaster);
}

// ==============================================================================
// BOILERPLATE
// ==============================================================================
const juce::String CHIMERA_FilterAudioProcessor::getName() const { return JucePlugin_Name; }
bool CHIMERA_FilterAudioProcessor::acceptsMidi() const { return true; }
bool CHIMERA_FilterAudioProcessor::producesMidi() const { return false; }
bool CHIMERA_FilterAudioProcessor::isMidiEffect() const { return false; }
double CHIMERA_FilterAudioProcessor::getTailLengthSeconds() const { return 0.0; }
int CHIMERA_FilterAudioProcessor::getNumPrograms() { return 1; }
int CHIMERA_FilterAudioProcessor::getCurrentProgram() { return 0; }
void CHIMERA_FilterAudioProcessor::setCurrentProgram(int) {}
const juce::String CHIMERA_FilterAudioProcessor::getProgramName(int) { return {}; }
void CHIMERA_FilterAudioProcessor::changeProgramName(int, const juce::String&) {}

void CHIMERA_FilterAudioProcessor::prepareToPlay(double sampleRate, int samplesPerBlock) {
    updateParameterCache();
    cachedSampleRate = sampleRate;
    cachedBlockSize = samplesPerBlock;

    // ★ Phase 83.2: Initialize TWO Oversamplers (one for 2x, one for 4x)
    // 2x oversampler (order 1 = 2^1 = 2x)
    oversampler2x = std::make_unique<juce::dsp::Oversampling<float>>(
        2,  // numChannels (stereo)
        1,  // oversampling order (2^1 = 2x)
        juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR,
        true  // isMaxQuality
    );
    oversampler2x->initProcessing(static_cast<size_t>(samplesPerBlock));

    // 4x oversampler (order 2 = 2^2 = 4x)
    oversampler4x = std::make_unique<juce::dsp::Oversampling<float>>(
        2,  // numChannels (stereo)
        2,  // oversampling order (2^2 = 4x)
        juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR,
        true  // isMaxQuality
    );
    oversampler4x->initProcessing(static_cast<size_t>(samplesPerBlock));

    currentOversamplingFactor = 1;  // Start with 1x (OFF)

    // ★ Phase 83: Voices always use BASE sample rate
    // The oversampler handles rate conversion, filters don't need to know about it
    for (auto& voice : voices) { voice.prepare(sampleRate); voice.reset(); }
    std::fill(activeNoteSlots.begin(), activeNoteSlots.end(), -1);
    myPhysics.prepare(sampleRate);
    smoothedMorph = (morphPosPtr) ? morphPosPtr->load() : 0.0f;

    // Initialize smoothers
    smoothMix.reset(sampleRate, 0.05);
    smoothMaster.reset(sampleRate, 0.05);
    smoothInputGain.reset(sampleRate, 0.05);
    smoothWetHighShelf.reset(sampleRate, 0.05);

    smoothMix.setCurrentAndTargetValue((mixPtr) ? mixPtr->load() : 1.0f);
    smoothMaster.setCurrentAndTargetValue((masterVolPtr) ? masterVolPtr->load() : 1.0f);
    smoothInputGain.setCurrentAndTargetValue((inputGainPtr) ? inputGainPtr->load() : 1.0f);
    smoothWetHighShelf.setCurrentAndTargetValue(0.0f);

    // ★ Phase 69: Reset filter states
    inputHpfStateL = 0.0f;
    inputHpfStateR = 0.0f;
    colorInputStateL = 0.0f;
    colorInputStateR = 0.0f;
    wetEnvStateL = 0.0f;
    wetEnvStateR = 0.0f;
    highShelfStateL = 0.0f;
    highShelfStateR = 0.0f;
    limiterGainL = 1.0f;
    limiterGainR = 1.0f;
}

// ★ Phase 70: Enhanced releaseResources for Ableton compatibility
void CHIMERA_FilterAudioProcessor::releaseResources()
{
    // Reset all voice states safely
    for (auto& voice : voices) {
        voice.reset();
    }

    // Reset physics engine
    myPhysics.reset();

    // Clear MIDI note slots
    std::fill(activeNoteSlots.begin(), activeNoteSlots.end(), -1);

    // Reset filter states
    inputHpfStateL = 0.0f;
    inputHpfStateR = 0.0f;
    colorInputStateL = 0.0f;
    colorInputStateR = 0.0f;
    wetEnvStateL = 0.0f;
    wetEnvStateR = 0.0f;
    highShelfStateL = 0.0f;
    highShelfStateR = 0.0f;
    limiterGainL = 1.0f;
    limiterGainR = 1.0f;
}

// ★ Phase 83.1: Panic function - emergency reset for howling/clicks
void CHIMERA_FilterAudioProcessor::panic()
{
    // Reset all voices
    for (auto& voice : voices) {
        voice.reset();
    }

    // Clear MIDI note slots
    std::fill(activeNoteSlots.begin(), activeNoteSlots.end(), -1);

    // Reset all filter states
    inputHpfStateL = 0.0f;
    inputHpfStateR = 0.0f;
    colorInputStateL = 0.0f;
    colorInputStateR = 0.0f;
    wetEnvStateL = 0.0f;
    wetEnvStateR = 0.0f;
    highShelfStateL = 0.0f;
    highShelfStateR = 0.0f;
    limiterGainL = 1.0f;
    limiterGainR = 1.0f;

    // Reset physics
    myPhysics.reset();
    smoothedMorph = (morphPosPtr) ? morphPosPtr->load() : 0.0f;
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool CHIMERA_FilterAudioProcessor::isBusesLayoutSupported(const BusesLayout& layouts) const {
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::mono() && layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo()) return false;
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet()) return false; return true;
}
#endif

void CHIMERA_FilterAudioProcessor::handleMidi(const juce::MidiBuffer& midiMessages) {
    if (colorModePtr == nullptr || colorModePtr->load() < 0.5f) return;
    for (const auto metadata : midiMessages) {
        auto msg = metadata.getMessage();
        if (msg.isNoteOn()) {
            int note = msg.getNoteNumber(); float vel = msg.getFloatVelocity();
            int slot = -1; for (int i = 0; i < 6; ++i) if (activeNoteSlots[i] == -1) { slot = i; break; }
            if (slot == -1) slot = 0; activeNoteSlots[slot] = note;
            int base = slot * 3;
            if (base < maxVoices) voices[base].noteOn(note, vel);
            if (base + 1 < maxVoices) voices[base + 1].noteOn(note, vel);
            if (base + 2 < maxVoices) voices[base + 2].noteOn(note, vel);
        }
        else if (msg.isNoteOff()) {
            int note = msg.getNoteNumber();
            for (int i = 0; i < 6; ++i) {
                if (activeNoteSlots[i] == note) {
                    activeNoteSlots[i] = -1; int base = i * 3;
                    if (base < maxVoices) voices[base].noteOff();
                    if (base + 1 < maxVoices) voices[base + 1].noteOff();
                    if (base + 2 < maxVoices) voices[base + 2].noteOff(); break;
                }
            }
        }
    }
}

juce::AudioProcessorEditor* CHIMERA_FilterAudioProcessor::createEditor() { return new CHIMERA_FilterAudioProcessorEditor(*this); }
bool CHIMERA_FilterAudioProcessor::hasEditor() const { return true; }

void CHIMERA_FilterAudioProcessor::getStateInformation(juce::MemoryBlock& destData) {
    auto state = apvts.copyState(); std::unique_ptr<juce::XmlElement> xml(state.createXml()); if (xml) copyXmlToBinary(*xml, destData);
}

void CHIMERA_FilterAudioProcessor::setStateInformation(const void* data, int sizeInBytes) {
    std::unique_ptr<juce::XmlElement> xmlState(getXmlFromBinary(data, sizeInBytes)); if (xmlState && xmlState->hasTagName(apvts.state.getType())) apvts.replaceState(juce::ValueTree::fromXml(*xmlState));
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter() { return new CHIMERA_FilterAudioProcessor(); }