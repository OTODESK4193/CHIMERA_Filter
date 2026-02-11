/*
  ==============================================================================
    PluginEditor.cpp
    Phase 87: Mouse-Hover-Only InfoBar Update (Anti-flicker Solution)

    Changes:
    - InfoBar updates ONLY when mouse is over the slider
    - Returns to 1-line display (28px height)
    - GUI height: 758 -> 740px (restored)

    Previous: Phase 86 - Two-line InfoBar attempt
  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

// ==============================================================================
// Helper Logic (Static)
// ==============================================================================
juce::String CHIMERA_FilterAudioProcessorEditor::getNoteNameFromFreq(float freq)
{
    if (freq <= 0) return "";
    float noteNum = 69.0f + 12.0f * std::log2(freq / 440.0f);
    int noteInt = (int)(noteNum + 0.5f);
    return juce::MidiMessage::getMidiNoteName(noteInt, true, true, 3);
}

juce::String CHIMERA_FilterAudioProcessorEditor::getMidiNoteName(int noteNumber)
{
    return juce::MidiMessage::getMidiNoteName(noteNumber, true, true, 3);
}

// ==============================================================================
// Phase 84: Japanese Description Functions
// ==============================================================================
juce::String CHIMERA_FilterAudioProcessorEditor::getFilterDescription(int typeIndex)
{
    switch (typeIndex)
    {
    case 0:  return juce::String::fromUTF8((const char*)u8"12dB/oct ローパス。高域カットで温かい音に");
    case 1:  return juce::String::fromUTF8((const char*)u8"12dB/oct ハイパス。低域カットで軽やかな音に");
    case 2:  return juce::String::fromUTF8((const char*)u8"12dB/oct バンドパス。Cutoff周辺のみ通過");
    case 3:  return juce::String::fromUTF8((const char*)u8"ノッチ。Cutoff周波数をカット");
    case 4:  return juce::String::fromUTF8((const char*)u8"Moog 24dB/oct ラダー。太く温かいアナログサウンド");
    case 5:  return juce::String::fromUTF8((const char*)u8"パラメトリックEQ。Cutoff周波数をブースト");
    case 6:  return juce::String::fromUTF8((const char*)u8"コムフィルター。金属的な響き");
    case 7:  return juce::String::fromUTF8((const char*)u8"Oberheim SEM風。滑らかで上品な音");
    case 8:  return juce::String::fromUTF8((const char*)u8"フランジャー。ジェット音効果");
    case 9:  return juce::String::fromUTF8((const char*)u8"ディフューザー。空間的な広がり");
    case 10: return juce::String::fromUTF8((const char*)u8"Steiner-Parker。アグレッシブな12dB");
    case 11: return juce::String::fromUTF8((const char*)u8"TB-303風ダイオードラダー。アシッドサウンド");
    case 12: return juce::String::fromUTF8((const char*)u8"フォルマント。人声的な母音フィルター");
    case 13: return juce::String::fromUTF8((const char*)u8"フェイザー。うねり感のある効果");
    case 14: return juce::String::fromUTF8((const char*)u8"EDP Wasp風。荒々しいキャラクター");
    case 15: return juce::String::fromUTF8((const char*)u8"MS-20風 Korg35。鋭い自己発振");
    case 16: return juce::String::fromUTF8((const char*)u8"Buchla風ローパスゲート。VCA+VCF");
    case 17: return juce::String::fromUTF8((const char*)u8"レゾネーター。弦共鳴シミュレーション");
    case 18: return juce::String::fromUTF8((const char*)u8"リングモジュレーター。ベル/メタリック");
    case 19: return juce::String::fromUTF8((const char*)u8"ビットクラッシャー。ローファイ効果");
    case 20: return juce::String::fromUTF8((const char*)u8"ボーカルフィルター。5母音モーフィング");
    default: return "";
    }
}

juce::String CHIMERA_FilterAudioProcessorEditor::getPhysicsDescription(int modeIndex)
{
    switch (modeIndex)
    {
    case 0: return juce::String::fromUTF8((const char*)u8"Linear: 等速移動+慣性。DJフィルター向き");
    case 1: return juce::String::fromUTF8((const char*)u8"Stepped: 階段状の変化。アルペジエーター的");
    case 2: return juce::String::fromUTF8((const char*)u8"Elastic: バネ振動。オーバーシュートする動き");
    case 3: return juce::String::fromUTF8((const char*)u8"Sloth: 可変カーブ遅延。ゆったり変化（最大4秒）");
    case 4: return juce::String::fromUTF8((const char*)u8"Chaos: ローレンツアトラクタ。予測不能な動き");
    case 5: return juce::String::fromUTF8((const char*)u8"Bounce: 重力+バウンド。跳ねる動き");
    default: return "";
    }
}

// ==============================================================================
// Phase 86: P1/P2/P3 Parameter Descriptions
// ==============================================================================
void CHIMERA_FilterAudioProcessorEditor::getFilterParamDescriptions(int typeIndex,
    juce::String& p1Desc, juce::String& p2Desc, juce::String& p3Desc)
{
    switch (typeIndex)
    {
    case 0: case 1: case 2: case 3:  // SVF types (LP/HP/BP/Notch) - no P1/P2/P3
        p1Desc = ""; p2Desc = ""; p3Desc = "";
        break;

    case 4:  // MoogLadder
        p1Desc = juce::String::fromUTF8((const char*)u8"Character: フィルターの性格（明るい〜暗い）");
        p2Desc = juce::String::fromUTF8((const char*)u8"Comp: 高レゾナンス時の音量補正");
        p3Desc = juce::String::fromUTF8((const char*)u8"Drift: アナログ風の微妙な揺らぎ");
        break;

    case 5:  // Peak
        p1Desc = juce::String::fromUTF8((const char*)u8"Gain: ブースト量（0〜24dB）");
        p2Desc = juce::String::fromUTF8((const char*)u8"BW: 帯域幅（狭い〜広い）");
        p3Desc = "";
        break;

    case 6:  // Comb
        p1Desc = juce::String::fromUTF8((const char*)u8"Time: ディレイ時間（ピッチに影響）");
        p2Desc = juce::String::fromUTF8((const char*)u8"FB: フィードバック量（金属感）");
        p3Desc = juce::String::fromUTF8((const char*)u8"Polar: 極性（+/-で音色変化）");
        break;

    case 7:  // OberheimSEM
        p1Desc = juce::String::fromUTF8((const char*)u8"Mode: LP〜BP〜HP連続モーフィング");
        p2Desc = juce::String::fromUTF8((const char*)u8"Warm: 温かみ（サチュレーション）");
        p3Desc = "";
        break;

    case 8:  // Flanger
        p1Desc = juce::String::fromUTF8((const char*)u8"Rate: LFO速度（うねりの速さ）");
        p2Desc = juce::String::fromUTF8((const char*)u8"Depth: 変調深さ（効果の強さ）");
        p3Desc = juce::String::fromUTF8((const char*)u8"FB: フィードバック（ジェット感）");
        break;

    case 9:  // Diffuser
        p1Desc = juce::String::fromUTF8((const char*)u8"Diff: 拡散量（ぼかし具合）");
        p2Desc = juce::String::fromUTF8((const char*)u8"Stage: オールパス段数（2〜8）");
        p3Desc = juce::String::fromUTF8((const char*)u8"Mod: LFO変調（動きを加える）");
        break;

    case 10:  // SteinerParker
        p1Desc = juce::String::fromUTF8((const char*)u8"Mode: LP〜BP〜HP連続変化");
        p2Desc = juce::String::fromUTF8((const char*)u8"Aggr: アグレッシブさ（歪み）");
        p3Desc = "";
        break;

    case 11:  // DiodeLadder
        p1Desc = juce::String::fromUTF8((const char*)u8"Sat: ダイオードサチュレーション");
        p2Desc = juce::String::fromUTF8((const char*)u8"Asym: 非対称歪み（偶数倍音）");
        p3Desc = juce::String::fromUTF8((const char*)u8"FB: フィードバック特性");
        break;

    case 12:  // Formant
        p1Desc = juce::String::fromUTF8((const char*)u8"Vowel: 母音（A-E-I-O-U）");
        p2Desc = juce::String::fromUTF8((const char*)u8"Gender: 声質（男性〜女性）");
        p3Desc = juce::String::fromUTF8((const char*)u8"Sharp: フォルマントの鋭さ");
        break;

    case 13:  // Phaser
        p1Desc = juce::String::fromUTF8((const char*)u8"Rate: LFO速度");
        p2Desc = juce::String::fromUTF8((const char*)u8"Depth: 変調深さ");
        p3Desc = juce::String::fromUTF8((const char*)u8"FB: フィードバック（深み）");
        break;

    case 14:  // Wasp
        p1Desc = juce::String::fromUTF8((const char*)u8"Mode: LP〜HPモーフィング");
        p2Desc = juce::String::fromUTF8((const char*)u8"Grit: CMOS風の荒さ");
        p3Desc = "";
        break;

    case 15:  // Korg35
        p1Desc = juce::String::fromUTF8((const char*)u8"Mode: LP / HP 切替");
        p2Desc = juce::String::fromUTF8((const char*)u8"Sat: サチュレーション量");
        p3Desc = juce::String::fromUTF8((const char*)u8"HP/LP: HPFミックス量");
        break;

    case 16:  // LowpassGate
        p1Desc = juce::String::fromUTF8((const char*)u8"Gate: VCA/LPF/両方の切替");
        p2Desc = juce::String::fromUTF8((const char*)u8"Resp: Vactrol応答速度");
        p3Desc = juce::String::fromUTF8((const char*)u8"Color: フィルター色付け");
        break;

    case 17:  // Resonator
        p1Desc = juce::String::fromUTF8((const char*)u8"Decay: 減衰時間（残響長）");
        p2Desc = juce::String::fromUTF8((const char*)u8"Bright: 明るさ（高域成分）");
        p3Desc = juce::String::fromUTF8((const char*)u8"Mix: Dry/Wet バランス");
        break;

    case 18:  // RingMod
        p1Desc = juce::String::fromUTF8((const char*)u8"Freq: キャリア周波数");
        p2Desc = juce::String::fromUTF8((const char*)u8"Mix: Dry/Wet バランス");
        p3Desc = juce::String::fromUTF8((const char*)u8"Rect: 整流量（偶数倍音）");
        break;

    case 19:  // Bitcrusher
        p1Desc = juce::String::fromUTF8((const char*)u8"Bits: ビット深度（1〜16bit）");
        p2Desc = juce::String::fromUTF8((const char*)u8"Down: ダウンサンプル率");
        p3Desc = juce::String::fromUTF8((const char*)u8"Mix: Dry/Wet バランス");
        break;

    case 20:  // VocalFilter
        p1Desc = juce::String::fromUTF8((const char*)u8"Vowel: 母音モーフィング");
        p2Desc = juce::String::fromUTF8((const char*)u8"Char: 声質キャラクター");
        p3Desc = juce::String::fromUTF8((const char*)u8"Sharp: フォルマント鋭さ");
        break;

    default:
        p1Desc = ""; p2Desc = ""; p3Desc = "";
        break;
    }
}

// ==============================================================================
// CurveVisualizer Implementation
// ==============================================================================
CurveVisualizer::CurveVisualizer(CHIMERA_FilterAudioProcessor& p) : processor(p)
{
    setOpaque(true);
    setInterceptsMouseClicks(false, false);
    startTimerHz(20);
}

void CurveVisualizer::setActiveHead(int headIndex)
{
    if (activeHeadIndex != headIndex) {
        activeHeadIndex = headIndex;
        repaint();
    }
}

void CurveVisualizer::paint(juce::Graphics& g)
{
    int w = getWidth();
    int h = getHeight();
    if (w <= 0 || h <= 0) return;

    g.fillAll(juce::Colour(20, 20, 20));

    g.setColour(juce::Colours::white.withAlpha(0.06f));
    float freqs[] = { 100.0f, 1000.0f, 10000.0f };
    for (float f : freqs) {
        float x = getXForFreq(f, w);
        if (x > 0) g.drawVerticalLine((int)x, 0.0f, (float)h);
    }
    float dbMarks[] = { 0.0f };
    for (float db : dbMarks) {
        float y = getYForDb(db, h);
        g.drawHorizontalLine((int)y, 0.0f, (float)w);
    }

    float physMorph = processor.currentVisualMorph.load();
    if (!std::isfinite(physMorph)) physMorph = 0.0f;
    physMorph = juce::jlimit(0.0f, 1.0f, physMorph);

    auto* colorModeParam = processor.apvts.getRawParameterValue("color_mode");
    bool isColorMode = (colorModeParam != nullptr) ? (*colorModeParam > 0.5f) : false;

    FilterData d[4];
    for (int i = 0; i < 4; ++i) d[i] = getFilterData(i);

    juce::Colour filterColors[4] = {
        juce::Colour(150, 220, 230),
        juce::Colour(180, 230, 180),
        juce::Colour(240, 200, 150),
        juce::Colour(220, 170, 220)
    };

    for (int i = 0; i < 4; ++i) {
        bool isActive = (activeHeadIndex == i);
        float alpha = isActive ? 0.8f : 0.3f;
        juce::Colour col = isColorMode
            ? (i == 0 ? juce::Colours::cyan.brighter(0.3f) :
                i == 1 ? juce::Colours::yellow :
                i == 2 ? juce::Colours::magenta.brighter(0.2f) : juce::Colours::white)
            : filterColors[i];
        drawSingleCurve(g, (float)w, (float)h, d[i], col.withAlpha(alpha));
    }

    float scaled = physMorph * 3.0f;
    int idx1 = (int)scaled;
    if (idx1 > 2) idx1 = 2;
    int idx2 = idx1 + 1;
    float blend = scaled - (float)idx1;

    juce::Colour blendCurveColor = filterColors[idx1]
        .interpolatedWith(filterColors[idx2], blend)
        .brighter(0.4f);

    drawBlendedCurve(g, (float)w, (float)h, d[idx1], d[idx2], blend, blendCurveColor);

    float centerFreq = d[idx1].cutoff * (1.0f - blend) + d[idx2].cutoff * blend;
    float gx = getXForFreq(centerFreq, w);
    juce::Colour ghostCol = isColorMode
        ? juce::Colours::yellow.withAlpha(0.8f)
        : filterColors[idx1].interpolatedWith(filterColors[idx2], blend).withAlpha(0.8f);

    g.setColour(ghostCol);
    g.drawVerticalLine((int)gx, 0.0f, (float)h);
    g.setColour(juce::Colours::white.withAlpha(0.3f));
    g.drawRect(getLocalBounds(), 1);
}

CurveVisualizer::FilterData CurveVisualizer::getFilterData(int idx)
{
    FilterData d;
    auto* pc = processor.getVisualCutoffPtr(idx);
    auto* pr = processor.getVisualResPtr(idx);
    auto* pd = processor.getVisualDrivePtr(idx);

    juce::String suffixes[] = { "A", "B", "C", "D" };
    auto* pt = processor.apvts.getRawParameterValue("type_" + suffixes[idx]);
    auto* pp1 = processor.apvts.getRawParameterValue("p1_" + suffixes[idx]);
    auto* pp2 = processor.apvts.getRawParameterValue("p2_" + suffixes[idx]);
    auto* pp3 = processor.apvts.getRawParameterValue("p3_" + suffixes[idx]);

    if (pc) d.cutoff = pc->load();
    if (pr) d.res = pr->load();
    if (pd) d.drive = pd->load();
    if (pt) d.type = (int)pt->load();
    if (pp1) d.p1 = pp1->load();
    if (pp2) d.p2 = pp2->load();
    if (pp3) d.p3 = pp3->load();

    d.cutoff = juce::jlimit(20.0f, 20000.0f, d.cutoff);
    d.res = juce::jlimit(0.1f, 10.0f, d.res);
    return d;
}

void CurveVisualizer::drawSingleCurve(juce::Graphics& g, float w, float h, const FilterData& d, juce::Colour color)
{
    g.setColour(color);
    juce::Path p;
    bool first = true;
    for (float x = 0.0f; x < w; x += 5.0f) {
        float freq = getFreqForX(x, (int)w);
        float mag = HentaiFilter::getMagnitudeForFrequency(freq, d.cutoff, d.res, d.type, d.drive, d.p1, d.p2, d.p3);
        float db = 20.0f * std::log10(mag + 0.00001f);
        float y = getYForDb(db, (int)h);
        if (first) { p.startNewSubPath(x, y); first = false; }
        else { p.lineTo(x, y); }
    }
    g.strokePath(p, juce::PathStrokeType(2.0f));
}

void CurveVisualizer::drawBlendedCurve(juce::Graphics& g, float w, float h, const FilterData& d1, const FilterData& d2, float blend, juce::Colour color)
{
    g.setColour(color.withAlpha(1.0f));
    juce::Path p;
    bool first = true;
    for (float x = 0.0f; x < w; x += 5.0f) {
        float freq = getFreqForX(x, (int)w);
        float mag1 = HentaiFilter::getMagnitudeForFrequency(freq, d1.cutoff, d1.res, d1.type, d1.drive, d1.p1, d1.p2, d1.p3);
        float mag2 = HentaiFilter::getMagnitudeForFrequency(freq, d2.cutoff, d2.res, d2.type, d2.drive, d2.p1, d2.p2, d2.p3);
        float mag = mag1 * (1.0f - blend) + mag2 * blend;
        float db = 20.0f * std::log10(mag + 0.00001f);
        float y = getYForDb(db, (int)h);
        if (first) { p.startNewSubPath(x, y); first = false; }
        else { p.lineTo(x, y); }
    }
    g.strokePath(p, juce::PathStrokeType(3.0f));
}

float CurveVisualizer::getXForFreq(float f, int w)
{
    if (f < 20.0f) return 0.0f;
    return (std::log10(f / 20.0f) / 3.0f) * (float)w;
}

float CurveVisualizer::getFreqForX(float x, int w)
{
    float normX = x / (float)w;
    return 20.0f * std::pow(1000.0f, normX);
}

float CurveVisualizer::getYForDb(float db, int h)
{
    float normalized = (db - (-60.0f)) / 90.0f;
    normalized = std::tanh(normalized * 2.0f - 1.0f) * 0.5f + 0.5f;
    return (float)h * (1.0f - normalized);
}

// ==============================================================================
// EDITOR CONSTRUCTOR
// ==============================================================================
CHIMERA_FilterAudioProcessorEditor::CHIMERA_FilterAudioProcessorEditor(CHIMERA_FilterAudioProcessor& p)
    : AudioProcessorEditor(&p), audioProcessor(p)
{
    setLookAndFeel(&customStyle);
    logoImage = juce::ImageCache::getFromMemory(
        BinaryData::chimera_logo_png,
        BinaryData::chimera_logo_pngSize);

    addAndMakeVisible(infoBar);

    curveVisualizer = std::make_unique<CurveVisualizer>(audioProcessor);
    addAndMakeVisible(*curveVisualizer);

    addAndMakeVisible(morphLabel);
    morphLabel.setText("MORPH", juce::dontSendNotification);
    morphLabel.setJustificationType(juce::Justification::centred);
    morphLabel.setColour(juce::Label::textColourId, juce::Colour(50, 50, 50));

    morphSlider = std::make_unique<HoverSlider>();
    addAndMakeVisible(*morphSlider);
    morphSlider->setName("Morph");
    morphSlider->setSuffix("");
    morphSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"4つのフィルター間をブレンド"));
    morphSlider->setRange(0.0, 1.0, 0.001);
    morphSlider->linkToVisual(&audioProcessor.currentVisualMorph, &infoBar);
    morphAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "morph_pos", *morphSlider);

    juce::String suffixes[] = { "A", "B", "C", "D" };
    for (int i = 0; i < 4; ++i) {
        auto& col = columns[i];
        col.bgColor = filterBgColors[i];
        addAndMakeVisible(col);

        col.jumpBtn.setButtonText("Filter " + suffixes[i]);
        col.jumpBtn.setColour(juce::TextButton::buttonColourId, filterBgColors[i]);
        col.jumpBtn.setColour(juce::TextButton::textColourOffId, juce::Colours::black);
        col.jumpBtn.addMouseListener(this, false);
        col.jumpBtn.onClick = [this, i]() {
            if (curveVisualizer) curveVisualizer->setActiveHead(i);
            if (auto* morphParam = audioProcessor.apvts.getParameter("morph_pos"))
                morphParam->setValueNotifyingHost((float)i / 3.0f);
            };
        col.addAndMakeVisible(col.jumpBtn);

        // Phase 84: All 21 filter types (ComboBox IDs 1-21 map to Mode 0-20)
        col.type.addItem("LP12", 1);
        col.type.addItem("HP12", 2);
        col.type.addItem("BP12", 3);
        col.type.addItem("Notch", 4);
        col.type.addItem("Moog", 5);
        col.type.addItem("Peak", 6);
        col.type.addItem("Comb", 7);
        col.type.addItem("Oberheim", 8);
        col.type.addItem("Flanger", 9);
        col.type.addItem("Diffuser", 10);
        col.type.addItem("Steiner", 11);
        col.type.addItem("Diode", 12);
        col.type.addItem("Formant", 13);
        col.type.addItem("Phaser", 14);
        col.type.addItem("Wasp", 15);
        col.type.addItem("Korg35", 16);
        col.type.addItem("LPG", 17);
        col.type.addItem("Resonator", 18);
        col.type.addItem("RingMod", 19);
        col.type.addItem("Bitcrush", 20);
        col.type.addItem("Vocal", 21);

        col.type.setSelectedId(1);
        col.type.setColour(juce::ComboBox::textColourId, juce::Colours::black);
        col.type.setColour(juce::ComboBox::arrowColourId, juce::Colours::black);
        col.type.setColour(juce::ComboBox::backgroundColourId, filterBgColors[i].darker(0.05f));
        col.type.addMouseListener(this, false);
        col.addAndMakeVisible(col.type);

        col.cutLabel.setText("Cutoff", juce::dontSendNotification);
        col.cutLabel.setJustificationType(juce::Justification::centred);
        col.cutLabel.setColour(juce::Label::textColourId, juce::Colours::black);
        col.addAndMakeVisible(col.cutLabel);
        col.cutoff = std::make_unique<HoverSlider>();
        col.cutoff->setName("Cutoff " + suffixes[i]);
        col.cutoff->setSuffix(" Hz");
        col.cutoff->setDescriptionText(juce::String::fromUTF8((const char*)u8"カットオフ周波数"));
        col.cutoff->setRange(20.0, 20000.0, 1.0);
        col.cutoff->setSkewFactorFromMidPoint(1000.0);
        col.cutoff->linkToVisual(audioProcessor.getVisualCutoffPtr(i), &infoBar);
        col.cutoff->customValueString = [this](float val) -> juce::String {
            auto* snapParam = audioProcessor.apvts.getRawParameterValue("snap_mode");
            bool snap = (snapParam && *snapParam > 0.5f);
            juce::String text = juce::String(val, 1) + " Hz";
            if (snap) text += " (" + getNoteNameFromFreq(val) + ")";
            return text;
            };
        col.cutoff->onValueChange = [this, i]() { if (curveVisualizer) curveVisualizer->setActiveHead(i); };
        col.addAndMakeVisible(*col.cutoff);
        col.cutoffAtt = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
            audioProcessor.apvts, "cutoff_" + suffixes[i], *col.cutoff);

        col.resLabel.setText("Res", juce::dontSendNotification);
        col.resLabel.setJustificationType(juce::Justification::centred);
        col.resLabel.setColour(juce::Label::textColourId, juce::Colours::black);
        col.addAndMakeVisible(col.resLabel);
        col.res = std::make_unique<HoverSlider>();
        col.res->setName("Res " + suffixes[i]);
        col.res->setSuffix("");
        col.res->setDescriptionText(juce::String::fromUTF8((const char*)u8"レゾナンス/Q値"));
        col.res->setRange(0.1, 10.0, 0.01);
        col.res->linkToVisual(audioProcessor.getVisualResPtr(i), &infoBar);
        col.res->onValueChange = [this, i]() { if (curveVisualizer) curveVisualizer->setActiveHead(i); };
        col.addAndMakeVisible(*col.res);
        col.resAtt = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
            audioProcessor.apvts, "res_" + suffixes[i], *col.res);

        col.driveLabel.setText("Drive", juce::dontSendNotification);
        col.driveLabel.setJustificationType(juce::Justification::centred);
        col.driveLabel.setColour(juce::Label::textColourId, juce::Colours::black);
        col.addAndMakeVisible(col.driveLabel);
        col.drive = std::make_unique<HoverSlider>();
        col.drive->setName("Drive " + suffixes[i]);
        col.drive->setSuffix("%");
        col.drive->setDescriptionText(juce::String::fromUTF8((const char*)u8"入力ゲイン/サチュレーション"));
        col.drive->setRange(0.0, 1.0, 0.01);
        col.drive->linkToVisual(audioProcessor.getVisualDrivePtr(i), &infoBar);
        col.drive->onValueChange = [this, i]() { if (curveVisualizer) curveVisualizer->setActiveHead(i); };
        col.addAndMakeVisible(*col.drive);
        col.driveAtt = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
            audioProcessor.apvts, "drive_" + suffixes[i], *col.drive);

        col.p1Label.setText("P1", juce::dontSendNotification);
        col.p1Label.setJustificationType(juce::Justification::centred);
        col.p1Label.setColour(juce::Label::textColourId, juce::Colours::black);
        col.addAndMakeVisible(col.p1Label);
        col.p1 = std::make_unique<HoverSlider>();
        col.p1->setName("P1 " + suffixes[i]);
        col.p1->setSuffix("");
        col.p1->setRange(0.0, 1.0, 0.01);
        col.p1->linkToVisual(nullptr, &infoBar);
        col.p1->onValueChange = [this, i]() { if (curveVisualizer) curveVisualizer->setActiveHead(i); };
        col.addAndMakeVisible(*col.p1);
        col.p1Att = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
            audioProcessor.apvts, "p1_" + suffixes[i], *col.p1);

        col.p2Label.setText("P2", juce::dontSendNotification);
        col.p2Label.setJustificationType(juce::Justification::centred);
        col.p2Label.setColour(juce::Label::textColourId, juce::Colours::black);
        col.addAndMakeVisible(col.p2Label);
        col.p2 = std::make_unique<HoverSlider>();
        col.p2->setName("P2 " + suffixes[i]);
        col.p2->setSuffix("");
        col.p2->setRange(0.0, 1.0, 0.01);
        col.p2->linkToVisual(nullptr, &infoBar);
        col.p2->onValueChange = [this, i]() { if (curveVisualizer) curveVisualizer->setActiveHead(i); };
        col.addAndMakeVisible(*col.p2);
        col.p2Att = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
            audioProcessor.apvts, "p2_" + suffixes[i], *col.p2);

        col.p3Label.setText("P3", juce::dontSendNotification);
        col.p3Label.setJustificationType(juce::Justification::centred);
        col.p3Label.setColour(juce::Label::textColourId, juce::Colours::black);
        col.addAndMakeVisible(col.p3Label);
        col.p3 = std::make_unique<HoverSlider>();
        col.p3->setName("P3 " + suffixes[i]);
        col.p3->setSuffix("");
        col.p3->setRange(0.0, 1.0, 0.01);
        col.p3->linkToVisual(nullptr, &infoBar);
        col.p3->onValueChange = [this, i]() { if (curveVisualizer) curveVisualizer->setActiveHead(i); };
        col.addAndMakeVisible(*col.p3);
        col.p3Att = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
            audioProcessor.apvts, "p3_" + suffixes[i], *col.p3);

        col.typeAtt = std::make_unique<juce::AudioProcessorValueTreeState::ComboBoxAttachment>(
            audioProcessor.apvts, "type_" + suffixes[i], col.type);
        col.type.addListener(this);
        updateFilterControls(i);
    }

    // Morph Physics Section
    modeCombo.addItem("Linear", 1);
    modeCombo.addItem("Stepped", 2);
    modeCombo.addItem("Elastic", 3);
    modeCombo.addItem("Sloth", 4);
    modeCombo.addItem("Chaos", 5);
    modeCombo.addItem("Bounce", 6);
    modeCombo.setSelectedId(1);
    modeCombo.setColour(juce::ComboBox::textColourId, juce::Colours::black);
    modeCombo.setColour(juce::ComboBox::arrowColourId, juce::Colours::black);
    modeCombo.setColour(juce::ComboBox::backgroundColourId, juce::Colours::white);
    modeCombo.setColour(juce::ComboBox::outlineColourId, juce::Colour(180, 180, 180));
    modeCombo.addMouseListener(this, false);
    modeCombo.onChange = [this] { updatePhysicsLabels(); };
    addAndMakeVisible(modeCombo);
    modeAttach = std::make_unique<juce::AudioProcessorValueTreeState::ComboBoxAttachment>(
        audioProcessor.apvts, "morph_mode", modeCombo);

    addAndMakeVisible(modeLabel);
    modeLabel.setJustificationType(juce::Justification::centredLeft);
    modeLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    modeLabel.setFont(13.0f);

    addAndMakeVisible(physALabel);
    physALabel.setJustificationType(juce::Justification::centred);
    physALabel.setColour(juce::Label::textColourId, juce::Colours::black);
    physALabel.setFont(13.0f);

    smoothSlider = std::make_unique<HoverSlider>();
    smoothSlider->setName("Param A");
    smoothSlider->setSuffix("");
    smoothSlider->setRange(0.1, 50.0, 0.1);
    smoothSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*smoothSlider);
    smoothAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "smoothing_time", *smoothSlider);

    addAndMakeVisible(physBLabel);
    physBLabel.setJustificationType(juce::Justification::centred);
    physBLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    physBLabel.setFont(13.0f);

    physicsParamBSlider = std::make_unique<HoverSlider>();
    physicsParamBSlider->setName("Param B");
    physicsParamBSlider->setSuffix("");
    physicsParamBSlider->setRange(0.0, 1.0, 0.01);
    physicsParamBSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*physicsParamBSlider);
    physicsParamBAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "physics_param_b", *physicsParamBSlider);

    addAndMakeVisible(logoLabel);
    logoLabel.setText("CHIMERA", juce::dontSendNotification);
    logoLabel.setJustificationType(juce::Justification::centred);
    logoLabel.setColour(juce::Label::textColourId, juce::Colour(80, 80, 80));
    logoLabel.setFont(juce::Font(24.0f, juce::Font::bold));

    // System Section
    addAndMakeVisible(systemLabel);
    systemLabel.setText("SYSTEM", juce::dontSendNotification);
    systemLabel.setJustificationType(juce::Justification::centred);
    systemLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    systemLabel.setFont(juce::Font(16.0f, juce::Font::bold));

    colorModeBtn.setButtonText("COLOR MODE");
    colorModeBtn.addMouseListener(this, false);
    addAndMakeVisible(colorModeBtn);
    colorModeAttach = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment>(
        audioProcessor.apvts, "color_mode", colorModeBtn);

    snapModeBtn.setButtonText("PITCH SNAP");
    snapModeBtn.addMouseListener(this, false);
    addAndMakeVisible(snapModeBtn);
    snapModeAttach = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment>(
        audioProcessor.apvts, "snap_mode", snapModeBtn);

    addAndMakeVisible(harmLabel);
    harmLabel.setText("Harmonics", juce::dontSendNotification);
    harmLabel.setJustificationType(juce::Justification::centred);
    harmLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    harmSlider = std::make_unique<HoverSlider>();
    harmSlider->setName("Harm Ratio");
    harmSlider->setSuffix("x");
    harmSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"倍音比"));
    harmSlider->setRange(1.0, 8.0, 0.1);
    harmSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*harmSlider);
    harmAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "harm_ratio", *harmSlider);

    addAndMakeVisible(intervalLabel);
    intervalLabel.setText("Interval", juce::dontSendNotification);
    intervalLabel.setJustificationType(juce::Justification::centred);
    intervalLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    intervalSlider = std::make_unique<HoverSlider>();
    intervalSlider->setName("Interval");
    intervalSlider->setSuffix(" st");
    intervalSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"半音オフセット"));
    intervalSlider->setRange(-24.0, 24.0, 1.0);
    intervalSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*intervalSlider);
    intervalAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "interval_semi", *intervalSlider);

    addAndMakeVisible(rootLabel);
    rootLabel.setText("Root", juce::dontSendNotification);
    rootLabel.setJustificationType(juce::Justification::centred);
    rootLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    rootSlider = std::make_unique<HoverSlider>();
    rootSlider->setName("Root Vol");
    rootSlider->setSuffix("");
    rootSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"基音音量"));
    rootSlider->setRange(0.0, 1.0, 0.01);
    rootSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*rootSlider);
    rootAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "vol_root", *rootSlider);

    addAndMakeVisible(harmVolLabel);
    harmVolLabel.setText("Harm", juce::dontSendNotification);
    harmVolLabel.setJustificationType(juce::Justification::centred);
    harmVolLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    harmVolSlider = std::make_unique<HoverSlider>();
    harmVolSlider->setName("Harm Vol");
    harmVolSlider->setSuffix("");
    harmVolSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"倍音音量"));
    harmVolSlider->setRange(0.0, 1.0, 0.01);
    harmVolSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*harmVolSlider);
    harmVolAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "vol_harm", *harmVolSlider);

    addAndMakeVisible(intVolLabel);
    intVolLabel.setText("Int", juce::dontSendNotification);
    intVolLabel.setJustificationType(juce::Justification::centred);
    intVolLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    intVolSlider = std::make_unique<HoverSlider>();
    intVolSlider->setName("Int Vol");
    intVolSlider->setSuffix("");
    intVolSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"インターバル音量"));
    intVolSlider->setRange(0.0, 1.0, 0.01);
    intVolSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*intVolSlider);
    intVolAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "vol_int", *intVolSlider);

    // Input Section
    addAndMakeVisible(inputGainLabel);
    inputGainLabel.setText("In Gain", juce::dontSendNotification);
    inputGainLabel.setJustificationType(juce::Justification::centred);
    inputGainLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    inputGainSlider = std::make_unique<HoverSlider>();
    inputGainSlider->setName("Input");
    inputGainSlider->setSuffix("");
    inputGainSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"入力ゲイン"));
    inputGainSlider->setRange(0.0, 2.0, 0.01);
    inputGainSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*inputGainSlider);
    inputGainAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "input_gain", *inputGainSlider);

    addAndMakeVisible(inputHpfLabel);
    inputHpfLabel.setText("In HPF", juce::dontSendNotification);
    inputHpfLabel.setJustificationType(juce::Justification::centred);
    inputHpfLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    inputHpfSlider = std::make_unique<HoverSlider>();
    inputHpfSlider->setName("HPF");
    inputHpfSlider->setSuffix(" Hz");
    inputHpfSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"ハイパスフィルター"));
    inputHpfSlider->setRange(20.0, 500.0, 1.0);
    inputHpfSlider->setSkewFactorFromMidPoint(100.0);
    inputHpfSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*inputHpfSlider);
    inputHpfAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "input_hpf", *inputHpfSlider);

    // WET Section
    addAndMakeVisible(wetAttackLabel);
    wetAttackLabel.setText("Attack", juce::dontSendNotification);
    wetAttackLabel.setJustificationType(juce::Justification::centred);
    wetAttackLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    wetAttackSlider = std::make_unique<HoverSlider>();
    wetAttackSlider->setName("Attack");
    wetAttackSlider->setSuffix(" ms");
    wetAttackSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"WETアタック時間"));
    wetAttackSlider->setRange(0.1, 100.0, 0.1);
    wetAttackSlider->setSkewFactorFromMidPoint(10.0);
    wetAttackSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*wetAttackSlider);
    wetAttackAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "wet_attack", *wetAttackSlider);

    addAndMakeVisible(wetReleaseLabel);
    wetReleaseLabel.setText("Release", juce::dontSendNotification);
    wetReleaseLabel.setJustificationType(juce::Justification::centred);
    wetReleaseLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    wetReleaseSlider = std::make_unique<HoverSlider>();
    wetReleaseSlider->setName("Release");
    wetReleaseSlider->setSuffix(" ms");
    wetReleaseSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"WETリリース時間"));
    wetReleaseSlider->setRange(10.0, 1000.0, 1.0);
    wetReleaseSlider->setSkewFactorFromMidPoint(100.0);
    wetReleaseSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*wetReleaseSlider);
    wetReleaseAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "wet_release", *wetReleaseSlider);

    addAndMakeVisible(wetHighShelfLabel);
    wetHighShelfLabel.setText("WET Hi", juce::dontSendNotification);
    wetHighShelfLabel.setJustificationType(juce::Justification::centred);
    wetHighShelfLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    wetHighShelfSlider = std::make_unique<HoverSlider>();
    wetHighShelfSlider->setName("Hi Shelf");
    wetHighShelfSlider->setSuffix(" dB");
    wetHighShelfSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"WET高域調整"));
    wetHighShelfSlider->setRange(-12.0, 12.0, 0.1);
    wetHighShelfSlider->linkToVisual(nullptr, &infoBar);
    addAndMakeVisible(*wetHighShelfSlider);
    wetHighShelfAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "wet_highshelf", *wetHighShelfSlider);

    // Output Section
    addAndMakeVisible(masterLabel);
    masterLabel.setText("Master", juce::dontSendNotification);
    masterLabel.setJustificationType(juce::Justification::centred);
    masterLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    masterSlider = std::make_unique<HoverSlider>();
    masterSlider->setName("Master");
    masterSlider->setSuffix("");
    masterSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"マスター音量"));
    masterSlider->setRange(0.0, 2.0, 0.01);
    masterSlider->linkToVisual(&audioProcessor.currentVisualMaster, &infoBar);
    addAndMakeVisible(*masterSlider);
    masterAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "master_vol", *masterSlider);

    addAndMakeVisible(mixLabel);
    mixLabel.setText("Dry/Wet", juce::dontSendNotification);
    mixLabel.setJustificationType(juce::Justification::centred);
    mixLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    mixSlider = std::make_unique<HoverSlider>();
    mixSlider->setName("Mix");
    mixSlider->setSuffix("");
    mixSlider->setDescriptionText(juce::String::fromUTF8((const char*)u8"ドライ/ウェット比"));
    mixSlider->setRange(0.0, 1.0, 0.01);
    mixSlider->linkToVisual(&audioProcessor.currentVisualMix, &infoBar);
    addAndMakeVisible(*mixSlider);
    mixAttach = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "mix", *mixSlider);

    limiterBtn.setButtonText("LIMITER");
    limiterBtn.addMouseListener(this, false);
    addAndMakeVisible(limiterBtn);
    limiterAttach = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment>(
        audioProcessor.apvts, "limiter_on", limiterBtn);

    qualityLabel.setText("Quality", juce::dontSendNotification);
    qualityLabel.setJustificationType(juce::Justification::centred);
    qualityLabel.setFont(juce::Font(14.0f, juce::Font::bold));
    qualityLabel.setColour(juce::Label::textColourId, juce::Colours::black);
    addAndMakeVisible(qualityLabel);

    oversamplingCombo.addItem("1x", 1);
    oversamplingCombo.addItem("2x", 2);
    oversamplingCombo.addItem("4x", 3);
    oversamplingCombo.setSelectedId(1);
    oversamplingCombo.addMouseListener(this, false);
    addAndMakeVisible(oversamplingCombo);
    oversamplingAttach = std::make_unique<juce::AudioProcessorValueTreeState::ComboBoxAttachment>(
        audioProcessor.apvts, "oversampling", oversamplingCombo);

    panicBtn.setButtonText("PANIC");
    panicBtn.setColour(juce::TextButton::buttonColourId, juce::Colour(180, 60, 60));
    panicBtn.setColour(juce::TextButton::textColourOffId, juce::Colours::white);
    panicBtn.addMouseListener(this, false);
    panicBtn.onClick = [this]() { audioProcessor.panic(); };
    addAndMakeVisible(panicBtn);

    updatePhysicsLabels();
    startTimerHz(20);

    // GUI Size: 740px (Phase 87: restored from 758px)
    setSize(1120, 740);
}

// ==============================================================================
// DESTRUCTOR
// ==============================================================================
CHIMERA_FilterAudioProcessorEditor::~CHIMERA_FilterAudioProcessorEditor()
{
    stopTimer();
    if (curveVisualizer) curveVisualizer->cleanup();

    colorModeBtn.removeMouseListener(this);
    snapModeBtn.removeMouseListener(this);
    limiterBtn.removeMouseListener(this);
    oversamplingCombo.removeMouseListener(this);
    panicBtn.removeMouseListener(this);
    modeCombo.removeMouseListener(this);
    for (auto& col : columns) {
        col.jumpBtn.removeMouseListener(this);
        col.type.removeMouseListener(this);
    }

    if (morphSlider) morphSlider->cleanup();
    if (smoothSlider) smoothSlider->cleanup();
    if (physicsParamBSlider) physicsParamBSlider->cleanup();

    for (auto& col : columns) {
        col.type.removeListener(this);
        if (col.cutoff) col.cutoff->cleanup();
        if (col.res) col.res->cleanup();
        if (col.drive) col.drive->cleanup();
        if (col.p1) col.p1->cleanup();
        if (col.p2) col.p2->cleanup();
        if (col.p3) col.p3->cleanup();
    }

    if (harmSlider) harmSlider->cleanup();
    if (intervalSlider) intervalSlider->cleanup();
    if (masterSlider) masterSlider->cleanup();
    if (mixSlider) mixSlider->cleanup();
    if (rootSlider) rootSlider->cleanup();
    if (harmVolSlider) harmVolSlider->cleanup();
    if (intVolSlider) intVolSlider->cleanup();
    if (inputGainSlider) inputGainSlider->cleanup();
    if (inputHpfSlider) inputHpfSlider->cleanup();
    if (wetAttackSlider) wetAttackSlider->cleanup();
    if (wetReleaseSlider) wetReleaseSlider->cleanup();
    if (wetHighShelfSlider) wetHighShelfSlider->cleanup();

    infoBar.clearText();

    morphAttach.reset(); modeAttach.reset(); smoothAttach.reset(); physicsParamBAttach.reset();
    colorModeAttach.reset(); snapModeAttach.reset(); mixAttach.reset(); masterAttach.reset();
    harmAttach.reset(); intervalAttach.reset(); rootAttach.reset(); harmVolAttach.reset();
    intVolAttach.reset(); inputGainAttach.reset(); inputHpfAttach.reset();
    wetAttackAttach.reset(); wetReleaseAttach.reset(); wetHighShelfAttach.reset();
    limiterAttach.reset(); oversamplingAttach.reset();

    for (auto& col : columns) {
        col.cutoffAtt.reset(); col.resAtt.reset(); col.driveAtt.reset();
        col.typeAtt.reset(); col.p1Att.reset(); col.p2Att.reset(); col.p3Att.reset();
    }

    curveVisualizer.reset(); morphSlider.reset(); smoothSlider.reset(); physicsParamBSlider.reset();

    for (auto& col : columns) {
        col.cutoff.reset(); col.res.reset(); col.drive.reset();
        col.p1.reset(); col.p2.reset(); col.p3.reset();
    }

    harmSlider.reset(); intervalSlider.reset(); masterSlider.reset(); mixSlider.reset();
    rootSlider.reset(); harmVolSlider.reset(); intVolSlider.reset();
    inputGainSlider.reset(); inputHpfSlider.reset();
    wetAttackSlider.reset(); wetReleaseSlider.reset(); wetHighShelfSlider.reset();

    removeAllChildren();
    setLookAndFeel(nullptr);
}

void CHIMERA_FilterAudioProcessorEditor::paint(juce::Graphics& g)
{
    g.fillAll(juce::Colour(235, 235, 235));

    int vizHeight = 200;
    int morphY = vizHeight + 5;
    int logoX = 20 + 120 + 80 + 80 + 80 + 20;
    int logoW = 750 - logoX - 5;
    int logoH = 85;

    if (logoImage.isValid())
    {
        float imgAspect = (float)logoImage.getWidth() / (float)logoImage.getHeight();
        float areaAspect = (float)logoW / (float)logoH;
        int drawW, drawH;
        if (imgAspect > areaAspect) { drawW = logoW; drawH = (int)(logoW / imgAspect); }
        else { drawH = logoH; drawW = (int)(logoH * imgAspect); }
        int drawX = logoX + (logoW - drawW) / 2;
        int drawY = morphY + (logoH - drawH) / 2;
        g.drawImage(logoImage, drawX, drawY, drawW, drawH, 0, 0, logoImage.getWidth(), logoImage.getHeight());
    }
    else
    {
        g.setColour(juce::Colours::black.withAlpha(0.15f));
        g.setFont(juce::Font("Impact", 32.0f, juce::Font::bold));
        g.drawText("CHIMERA", logoX, morphY + 20, 150, 40, juce::Justification::centredLeft);
        g.setFont(juce::Font(12.0f, juce::Font::plain));
        g.drawText("POLY MORPH FILTER", logoX + 2, morphY + 50, 150, 20, juce::Justification::centredLeft);
    }
}

void CHIMERA_FilterAudioProcessorEditor::resized()
{
    auto area = getLocalBounds();

    // InfoBar at bottom (28px for single-line display - Phase 87)
    auto infoBarArea = area.removeFromBottom(28);
    infoBar.setBounds(infoBarArea);

    auto leftArea = area.removeFromLeft(750);
    auto rightArea = area;

    int vizHeight = 200;
    auto vizArea = leftArea.removeFromTop(vizHeight);
    if (curveVisualizer) curveVisualizer->setBounds(vizArea.reduced(5));

    leftArea.removeFromTop(5);

    int morphSectionHeight = 90;
    auto morphSection = leftArea.removeFromTop(morphSectionHeight);
    {
        int modeW = 120;
        int knobW = 80;

        auto modeArea = morphSection.removeFromLeft(modeW).reduced(5);
        modeLabel.setBounds(modeArea.removeFromTop(18));
        modeLabel.setText("Physics:", juce::dontSendNotification);
        modeLabel.setJustificationType(juce::Justification::centredLeft);
        modeCombo.setBounds(modeArea.removeFromTop(24));

        auto morphArea = morphSection.removeFromLeft(knobW).reduced(2);
        morphLabel.setBounds(morphArea.removeFromTop(18));
        morphSlider->setBounds(morphArea);

        auto paramAArea = morphSection.removeFromLeft(knobW).reduced(2);
        physALabel.setBounds(paramAArea.removeFromTop(18));
        smoothSlider->setBounds(paramAArea);

        auto paramBArea = morphSection.removeFromLeft(knobW).reduced(2);
        physBLabel.setBounds(paramBArea.removeFromTop(18));
        physicsParamBSlider->setBounds(paramBArea);
    }

    leftArea.removeFromTop(5);
    int colW = leftArea.getWidth() / 4;
    for (int i = 0; i < 4; ++i)
        columns[i].setBounds(leftArea.removeFromLeft(colW).reduced(3));

    // Right Area
    rightArea.reduce(5, 0);
    systemLabel.setBounds(rightArea.removeFromTop(24));
    rightArea.removeFromTop(5);

    const int colW3 = rightArea.getWidth() / 3;
    const int sysKnobSize = 85;
    const int sysLabelH = 15;
    const int rowHeight = sysKnobSize + sysLabelH + 10;
    const int buttonHeight = 28;

    auto row1 = rightArea.removeFromTop(buttonHeight + 8);
    colorModeBtn.setBounds(row1.removeFromLeft(colW3).reduced(3, 2));
    snapModeBtn.setBounds(row1.removeFromLeft(colW3).reduced(3, 2));

    auto row2 = rightArea.removeFromTop(rowHeight);
    { auto col1 = row2.removeFromLeft(colW3).reduced(2); harmLabel.setBounds(col1.removeFromTop(sysLabelH)); harmSlider->setBounds(col1.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); auto col2 = row2.removeFromLeft(colW3).reduced(2); intervalLabel.setBounds(col2.removeFromTop(sysLabelH)); intervalSlider->setBounds(col2.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); }

    auto row3 = rightArea.removeFromTop(rowHeight);
    { auto col1 = row3.removeFromLeft(colW3).reduced(2); rootLabel.setBounds(col1.removeFromTop(sysLabelH)); rootSlider->setBounds(col1.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); auto col2 = row3.removeFromLeft(colW3).reduced(2); harmVolLabel.setBounds(col2.removeFromTop(sysLabelH)); harmVolSlider->setBounds(col2.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); auto col3 = row3.removeFromLeft(colW3).reduced(2); intVolLabel.setBounds(col3.removeFromTop(sysLabelH)); intVolSlider->setBounds(col3.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); }

    auto row4 = rightArea.removeFromTop(rowHeight);
    { auto col1 = row4.removeFromLeft(colW3).reduced(2); inputGainLabel.setBounds(col1.removeFromTop(sysLabelH)); inputGainSlider->setBounds(col1.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); auto col2 = row4.removeFromLeft(colW3).reduced(2); inputHpfLabel.setBounds(col2.removeFromTop(sysLabelH)); inputHpfSlider->setBounds(col2.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); }

    auto row5 = rightArea.removeFromTop(rowHeight);
    { auto col1 = row5.removeFromLeft(colW3).reduced(2); wetAttackLabel.setBounds(col1.removeFromTop(sysLabelH)); wetAttackSlider->setBounds(col1.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); auto col2 = row5.removeFromLeft(colW3).reduced(2); wetReleaseLabel.setBounds(col2.removeFromTop(sysLabelH)); wetReleaseSlider->setBounds(col2.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); auto col3 = row5.removeFromLeft(colW3).reduced(2); wetHighShelfLabel.setBounds(col3.removeFromTop(sysLabelH)); wetHighShelfSlider->setBounds(col3.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); }

    auto row6 = rightArea.removeFromTop(rowHeight);
    { auto col1 = row6.removeFromLeft(colW3).reduced(2); masterLabel.setBounds(col1.removeFromTop(sysLabelH)); masterSlider->setBounds(col1.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); auto col2 = row6.removeFromLeft(colW3).reduced(2); mixLabel.setBounds(col2.removeFromTop(sysLabelH)); mixSlider->setBounds(col2.withSizeKeepingCentre(sysKnobSize, sysKnobSize)); auto col3 = row6.removeFromLeft(colW3).reduced(2); col3.removeFromTop(sysLabelH); limiterBtn.setBounds(col3.removeFromTop(buttonHeight).reduced(2)); qualityLabel.setBounds(col3.removeFromTop(14)); oversamplingCombo.setBounds(col3.removeFromTop(buttonHeight).reduced(2)); }

    rightArea.removeFromTop(2);
    auto panicRow = rightArea.removeFromTop(buttonHeight);
    panicBtn.setBounds(panicRow.removeFromLeft(colW3 * 2).reduced(20, 0));
}

// ==============================================================================
// CALLBACKS
// ==============================================================================
void CHIMERA_FilterAudioProcessorEditor::updatePhysicsLabels()
{
    int mode = modeCombo.getSelectedId() - 1;
    if (mode < 0) mode = 0;

    juce::String txtA = "Smooth", txtB = "Param B", sfxA = "", sfxB = "";
    juce::String descA, descB;

    switch (mode) {
    case 0:
        txtA = "Speed"; txtB = "Inertia";
        descA = juce::String::fromUTF8((const char*)u8"移動速度");
        descB = juce::String::fromUTF8((const char*)u8"慣性");
        break;
    case 1:
        txtA = "Steps"; txtB = "Smooth";
        descA = juce::String::fromUTF8((const char*)u8"ステップ数");
        descB = juce::String::fromUTF8((const char*)u8"角の丸み");
        break;
    case 2:
        txtA = "Tension"; txtB = "Damping";
        descA = juce::String::fromUTF8((const char*)u8"バネ強度");
        descB = juce::String::fromUTF8((const char*)u8"減衰");
        break;
    case 3:
        txtA = "Time"; sfxA = " ms"; txtB = "Curve";
        descA = juce::String::fromUTF8((const char*)u8"遅延時間");
        descB = juce::String::fromUTF8((const char*)u8"カーブ形状");
        break;
    case 4:
        txtA = "Speed"; txtB = "Tether";
        descA = juce::String::fromUTF8((const char*)u8"カオス速度");
        descB = juce::String::fromUTF8((const char*)u8"拘束力");
        break;
    case 5:
        txtA = "Gravity"; txtB = "Bounce";
        descA = juce::String::fromUTF8((const char*)u8"重力");
        descB = juce::String::fromUTF8((const char*)u8"反発係数");
        break;
    }

    physALabel.setText(txtA, juce::dontSendNotification);
    physBLabel.setText(txtB, juce::dontSendNotification);

    if (smoothSlider) { smoothSlider->setName(txtA); smoothSlider->setSuffix(sfxA); smoothSlider->setDescriptionText(descA); }
    if (physicsParamBSlider) { physicsParamBSlider->setName(txtB); physicsParamBSlider->setSuffix(sfxB); physicsParamBSlider->setDescriptionText(descB); }
}

void CHIMERA_FilterAudioProcessorEditor::timerCallback()
{
    auto* colorParam = audioProcessor.apvts.getRawParameterValue("color_mode");
    if (colorParam && *colorParam > 0.5f) {
        juce::String midiText = "MIDI: ";
        bool found = false;
        for (int note : audioProcessor.activeNoteSlots) {
            if (note >= 0) { midiText += getMidiNoteName(note) + " "; found = true; }
        }
        juce::String current = infoBar.getText();
        if (current.isEmpty() || current.startsWith("MIDI")) {
            if (found) {
                // Phase 87: Single-line format
                infoBar.setText(midiText + "  ---  " +
                    juce::String::fromUTF8((const char*)u8"Color Mode: 再生中のノート"));
            }
            else if (current.startsWith("MIDI")) infoBar.clearText();
        }
    }
}

void CHIMERA_FilterAudioProcessorEditor::mouseEnter(const juce::MouseEvent& e)
{
    juce::Component* c = e.eventComponent;

    // Phase 87: Single-line format with " --- " separator
    if (c == &colorModeBtn) {
        infoBar.setText("Color Mode  ---  " +
            juce::String::fromUTF8((const char*)u8"ポリフォニックシンセモード。フィルターの自己発振で楽音を生成"));
    }
    else if (c == &snapModeBtn) {
        infoBar.setText("Pitch Snap  ---  " +
            juce::String::fromUTF8((const char*)u8"Cutoffを半音単位にスナップ"));
    }
    else if (c == &limiterBtn) {
        infoBar.setText("Limiter  ---  " +
            juce::String::fromUTF8((const char*)u8"出力保護リミッター"));
    }
    else if (c == &oversamplingCombo) {
        infoBar.setText("Quality  ---  " +
            juce::String::fromUTF8((const char*)u8"オーバーサンプリング (1x/2x/4x)。高いほど高音質・高負荷"));
    }
    else if (c == &panicBtn) {
        infoBar.setText("Panic  ---  " +
            juce::String::fromUTF8((const char*)u8"緊急リセット。音が止まらない時に使用"));
    }
    else if (c == &modeCombo) {
        int mode = modeCombo.getSelectedId() - 1;
        infoBar.setText("Physics: " + modeCombo.getText() + "  ---  " + getPhysicsDescription(mode));
    }
    else {
        for (int i = 0; i < 4; ++i) {
            auto& col = columns[i];
            if (c == &col.type) {
                int typeIdx = col.type.getSelectedId() - 1;
                infoBar.setText("Type: " + col.type.getText() + "  ---  " + getFilterDescription(typeIdx));
                return;
            }
            if (c == &col.jumpBtn) {
                infoBar.setText(juce::String::fromUTF8((const char*)u8"Filter ") + juce::String(char('A' + i)) +
                    "  ---  " + juce::String::fromUTF8((const char*)u8"このフィルターにMorphをジャンプ"));
                return;
            }
        }
    }
}

void CHIMERA_FilterAudioProcessorEditor::mouseMove(const juce::MouseEvent& e)
{
    auto logoArea = juce::Rectangle<int>(400, 205, 345, 85);
    if (logoArea.contains(e.getPosition()))
    {
        // Phase 87: Single-line format
        infoBar.setText("OTODESK GitHub  ---  " +
            juce::String::fromUTF8((const char*)u8"クリックでGitHubを開く"));
        setMouseCursor(juce::MouseCursor::PointingHandCursor);
    }
    else
    {
        setMouseCursor(juce::MouseCursor::NormalCursor);
    }
}

void CHIMERA_FilterAudioProcessorEditor::mouseExit(const juce::MouseEvent&) { infoBar.clearText(); }

void CHIMERA_FilterAudioProcessorEditor::mouseDown(const juce::MouseEvent& e)
{
    auto logoArea = juce::Rectangle<int>(400, 205, 345, 85);
    if (logoArea.contains(e.getPosition()))
        juce::URL("https://github.com/OTODESK4193").launchInDefaultBrowser();
}

void CHIMERA_FilterAudioProcessorEditor::comboBoxChanged(juce::ComboBox* box)
{
    for (int i = 0; i < 4; ++i) {
        if (box == &columns[i].type) {
            updateFilterControls(i);
            if (curveVisualizer) curveVisualizer->setActiveHead(i);
            int typeIdx = box->getSelectedId() - 1;
            // Phase 87: Single-line format
            infoBar.setText("Type: " + box->getText() + "  ---  " + getFilterDescription(typeIdx));
            break;
        }
    }
}

void CHIMERA_FilterAudioProcessorEditor::updateFilterControls(int colIndex)
{
    if (colIndex < 0 || colIndex >= 4) return;
    auto& col = columns[colIndex];
    int selectedType = col.type.getSelectedId() - 1;

    bool hasP1P2P3 = (selectedType >= 4);
    bool hasP3 = (selectedType == 4 || selectedType == 6 || selectedType == 8 ||
        selectedType == 9 || selectedType == 11 || selectedType == 12 ||
        selectedType == 13 || selectedType == 15 || selectedType == 16 ||
        selectedType == 17 || selectedType == 18 || selectedType == 19 ||
        selectedType == 20);

    col.p1->setEnabled(hasP1P2P3);
    col.p2->setEnabled(hasP1P2P3);
    col.p3->setEnabled(hasP3);

    juce::String suffixArr[] = { "A", "B", "C", "D" };
    juce::String p1Name = "P1", p2Name = "P2", p3Name = "P3";

    switch (selectedType) {
    case 4: p1Name = "Char"; p2Name = "Comp"; p3Name = "Drift"; break;
    case 5: p1Name = "Gain"; p2Name = "BW"; p3Name = "-"; break;
    case 6: p1Name = "Time"; p2Name = "FB"; p3Name = "Polar"; break;
    case 7: p1Name = "Mode"; p2Name = "Warm"; p3Name = "-"; break;
    case 8: p1Name = "Rate"; p2Name = "Depth"; p3Name = "FB"; break;
    case 9: p1Name = "Diff"; p2Name = "Stage"; p3Name = "Mod"; break;
    case 10: p1Name = "Mode"; p2Name = "Aggr"; p3Name = "-"; break;
    case 11: p1Name = "Sat"; p2Name = "Asym"; p3Name = "FB"; break;
    case 12: p1Name = "Vowel"; p2Name = "Gender"; p3Name = "Sharp"; break;
    case 13: p1Name = "Rate"; p2Name = "Depth"; p3Name = "FB"; break;
    case 14: p1Name = "Mode"; p2Name = "Grit"; p3Name = "-"; break;
    case 15: p1Name = "Mode"; p2Name = "Sat"; p3Name = "HP/LP"; break;
    case 16: p1Name = "Gate"; p2Name = "Resp"; p3Name = "Color"; break;
    case 17: p1Name = "Decay"; p2Name = "Bright"; p3Name = "Mix"; break;
    case 18: p1Name = "Freq"; p2Name = "Mix"; p3Name = "Rect"; break;
    case 19: p1Name = "Bits"; p2Name = "Down"; p3Name = "Mix"; break;
    case 20: p1Name = "Vowel"; p2Name = "Char"; p3Name = "Sharp"; break;
    default: p1Name = "P1"; p2Name = "P2"; p3Name = "P3"; break;
    }

    col.p1Label.setText(p1Name, juce::dontSendNotification);
    col.p2Label.setText(p2Name, juce::dontSendNotification);
    col.p3Label.setText(p3Name, juce::dontSendNotification);

    if (col.p1) col.p1->setName(p1Name + " " + suffixArr[colIndex]);
    if (col.p2) col.p2->setName(p2Name + " " + suffixArr[colIndex]);
    if (col.p3) col.p3->setName(p3Name + " " + suffixArr[colIndex]);

    // Phase 86: Set Japanese descriptions for P1/P2/P3
    juce::String p1Desc, p2Desc, p3Desc;
    getFilterParamDescriptions(selectedType, p1Desc, p2Desc, p3Desc);

    if (col.p1) col.p1->setDescriptionText(p1Desc);
    if (col.p2) col.p2->setDescriptionText(p2Desc);
    if (col.p3) col.p3->setDescriptionText(p3Desc);
}