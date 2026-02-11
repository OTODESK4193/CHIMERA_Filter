/*
  ==============================================================================
    PluginEditor.h
    Phase 87: Mouse-Hover-Only InfoBar Update (Anti-flicker Solution)

    Changes:
    - InfoBar updates ONLY when mouse is over the slider
    - Prevents flickering during automation
    - Returns to 1-line display (28px height)
    - GUI height: 758 -> 740px (restored)

    Previous: Phase 86 - Two-line InfoBar attempt
  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"
#include <functional>

// ======================================================================
// Forward Declarations
// ======================================================================
class CurveVisualizer;
class InfoBarComponent;
class HoverSlider;

// ======================================================================
// ChimeraStyle - Custom LookAndFeel
// ======================================================================
class ChimeraStyle : public juce::LookAndFeel_V4
{
public:
    ChimeraStyle()
    {
        setColour(juce::Slider::thumbColourId, juce::Colour(80, 80, 80));
        setColour(juce::Slider::rotarySliderFillColourId, juce::Colour(100, 160, 200));
        setColour(juce::Slider::rotarySliderOutlineColourId, juce::Colour(50, 50, 50));
        setColour(juce::Slider::trackColourId, juce::Colour(120, 120, 120));
        setColour(juce::ComboBox::backgroundColourId, juce::Colours::white);
        setColour(juce::ComboBox::outlineColourId, juce::Colour(150, 150, 150));
        setColour(juce::ComboBox::textColourId, juce::Colours::black);
        setColour(juce::TextButton::buttonColourId, juce::Colour(200, 200, 200));
        setColour(juce::TextButton::textColourOffId, juce::Colours::black);
    }

    void drawRotarySlider(juce::Graphics& g, int x, int y, int width, int height,
        float sliderPosProportional, float rotaryStartAngle,
        float rotaryEndAngle, juce::Slider& slider) override
    {
        auto bounds = juce::Rectangle<int>(x, y, width, height).toFloat().reduced(8.0f);
        auto radius = juce::jmin(bounds.getWidth(), bounds.getHeight()) / 2.0f;
        auto toAngle = rotaryStartAngle + sliderPosProportional * (rotaryEndAngle - rotaryStartAngle);
        auto lineW = juce::jmin(6.0f, radius * 0.4f);
        auto arcRadius = radius - lineW * 0.5f;

        juce::Path backgroundArc;
        backgroundArc.addCentredArc(bounds.getCentreX(), bounds.getCentreY(),
            arcRadius, arcRadius, 0.0f,
            rotaryStartAngle, rotaryEndAngle, true);
        g.setColour(slider.findColour(juce::Slider::rotarySliderOutlineColourId));
        g.strokePath(backgroundArc, juce::PathStrokeType(lineW, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));

        if (slider.isEnabled())
        {
            juce::Path valueArc;
            valueArc.addCentredArc(bounds.getCentreX(), bounds.getCentreY(),
                arcRadius, arcRadius, 0.0f,
                rotaryStartAngle, toAngle, true);
            g.setColour(slider.findColour(juce::Slider::rotarySliderFillColourId));
            g.strokePath(valueArc, juce::PathStrokeType(lineW, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
        }

        auto thumbWidth = lineW * 1.2f;
        juce::Point<float> thumbPoint(bounds.getCentreX() + arcRadius * std::cos(toAngle - juce::MathConstants<float>::halfPi),
            bounds.getCentreY() + arcRadius * std::sin(toAngle - juce::MathConstants<float>::halfPi));
        g.setColour(slider.findColour(juce::Slider::thumbColourId));
        g.fillEllipse(juce::Rectangle<float>(thumbWidth, thumbWidth).withCentre(thumbPoint));
    }

    void drawToggleButton(juce::Graphics& g, juce::ToggleButton& button,
        bool shouldDrawButtonAsHighlighted,
        bool shouldDrawButtonAsDown) override
    {
        auto bounds = button.getLocalBounds().toFloat().reduced(0.0f, 2.0f);
        auto checkArea = bounds.removeFromLeft(22.0f).reduced(2.0f);

        g.setColour(juce::Colours::white);
        g.fillRoundedRectangle(checkArea, 3.0f);
        g.setColour(juce::Colour(180, 180, 180));
        g.drawRoundedRectangle(checkArea, 3.0f, 1.5f);

        if (button.getToggleState())
        {
            g.setColour(juce::Colour(50, 150, 80));
            juce::Path tick;
            tick.startNewSubPath(checkArea.getX() + 5, checkArea.getCentreY());
            tick.lineTo(checkArea.getCentreX(), checkArea.getBottom() - 5);
            tick.lineTo(checkArea.getRight() - 5, checkArea.getY() + 5);
            g.strokePath(tick, juce::PathStrokeType(2.5f));
        }

        g.setColour(juce::Colours::black);
        g.setFont(juce::Font(14.0f, juce::Font::bold));
        g.drawText(button.getButtonText(), bounds.withTrimmedLeft(4.0f),
            juce::Justification::centredLeft, true);
    }
};

// ======================================================================
// InfoBarComponent - Phase 87: Single-line display (restored)
// Shows parameter info only when mouse hovers over slider
// ======================================================================
class InfoBarComponent : public juce::Component
{
public:
    InfoBarComponent()
    {
        textLabel.setColour(juce::Label::textColourId, juce::Colours::black);
        textLabel.setFont(juce::Font("Meiryo UI", 15.0f, juce::Font::bold));
        textLabel.setJustificationType(juce::Justification::centredLeft);
        addAndMakeVisible(textLabel);
    }

    void paint(juce::Graphics& g) override
    {
        g.fillAll(juce::Colour(235, 235, 235));
        g.setColour(juce::Colour(180, 180, 180));
        g.drawHorizontalLine(0, 0.0f, (float)getWidth());
    }

    void resized() override
    {
        textLabel.setBounds(getLocalBounds().reduced(10, 2));
    }

    void setText(const juce::String& text)
    {
        textLabel.setText(text, juce::dontSendNotification);
    }

    void clearText()
    {
        textLabel.setText("", juce::dontSendNotification);
    }

    juce::String getText() const { return textLabel.getText(); }

    // Legacy methods for compatibility (Phase 86 interface)
    void setValueText(const juce::String& text) { setText(text); }
    void setDescText(const juce::String&) { /* ignored in 1-line mode */ }

private:
    juce::Label textLabel;
};

// ======================================================================
// HoverSlider - Phase 87: Mouse-hover-only InfoBar update
// Key change: valueChanged() only updates InfoBar when isMouseOver is true
// ======================================================================
class HoverSlider : public juce::Slider, public juce::Timer
{
public:
    HoverSlider(juce::Slider::SliderStyle style = juce::Slider::RotaryHorizontalVerticalDrag)
        : juce::Slider(style, juce::Slider::TextBoxBelow)
    {
        setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 18);
        setColour(juce::Slider::textBoxTextColourId, juce::Colours::black);
        setColour(juce::Slider::textBoxBackgroundColourId, juce::Colours::white);
        setColour(juce::Slider::textBoxOutlineColourId, juce::Colour(200, 200, 200));
        setVelocityBasedMode(true);
        setVelocityModeParameters(0.7, 1, 0.09, false);
        setDoubleClickReturnValue(true, 0.5);
        setPopupDisplayEnabled(false, false, nullptr);
    }

    ~HoverSlider() override { stopTimer(); }

    void setSuffix(const juce::String& s) { suffix = s; }
    void setDescriptionText(const juce::String& desc) { descriptionText = desc; }

    std::function<juce::String(float)> customValueString;

    void cleanup()
    {
        stopTimer();
        visualSource = nullptr;
        targetInfoBar = nullptr;
    }

    void linkToVisual(std::atomic<float>* atomicSource, InfoBarComponent* infoBar)
    {
        visualSource = atomicSource;
        targetInfoBar = infoBar;
        if (visualSource) startTimerHz(20);
    }

    void timerCallback() override
    {
        if (visualSource != nullptr)
        {
            float val = visualSource->load();
            if (std::abs(val - lastVisualValue) > 0.001f)
            {
                lastVisualValue = val;
                repaint();
            }
        }
    }

    // Phase 87: Only update InfoBar if mouse is currently over this slider
    void valueChanged() override
    {
        // Key fix: Only update InfoBar when mouse is hovering
        // This prevents flickering during automation
        if (isMouseOver)
        {
            updateInfoBar();
        }
    }

    void mouseEnter(const juce::MouseEvent&) override
    {
        isMouseOver = true;
        updateInfoBar();
    }

    void mouseExit(const juce::MouseEvent&) override
    {
        isMouseOver = false;
        if (targetInfoBar)
            targetInfoBar->clearText();
    }

private:
    void updateInfoBar()
    {
        if (targetInfoBar)
        {
            juce::String text = getName() + ": ";
            if (customValueString)
                text += customValueString((float)getValue());
            else
                text += juce::String(getValue(), 2) + suffix;

            // Append description if available
            if (descriptionText.isNotEmpty())
                text += "  ---  " + descriptionText;

            targetInfoBar->setText(text);
        }
    }

    std::atomic<float>* visualSource = nullptr;
    InfoBarComponent* targetInfoBar = nullptr;
    float lastVisualValue = 0.0f;
    juce::String suffix;
    juce::String descriptionText;
    bool isMouseOver = false;  // Phase 87: Track mouse hover state
};

// ======================================================================
// CurveVisualizer
// ======================================================================
class CurveVisualizer : public juce::Component, public juce::Timer
{
public:
    CurveVisualizer(CHIMERA_FilterAudioProcessor& p);
    ~CurveVisualizer() override { stopTimer(); }

    void paint(juce::Graphics& g) override;
    void timerCallback() override { repaint(); }
    void setActiveHead(int headIndex);
    void cleanup() { stopTimer(); }

private:
    CHIMERA_FilterAudioProcessor& processor;
    int activeHeadIndex = -1;

    struct FilterData {
        float cutoff = 1000.0f;
        float res = 0.707f;
        float drive = 0.0f;
        int type = 0;
        float p1 = 0.0f, p2 = 0.0f, p3 = 0.0f;
    };

    FilterData getFilterData(int idx);
    void drawSingleCurve(juce::Graphics& g, float w, float h, const FilterData& d, juce::Colour color);
    void drawBlendedCurve(juce::Graphics& g, float w, float h, const FilterData& d1, const FilterData& d2, float blend, juce::Colour color);

    float getXForFreq(float f, int w);
    float getFreqForX(float x, int w);
    float getYForDb(float db, int h);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(CurveVisualizer)
};

// ======================================================================
// CHIMERA_FilterAudioProcessorEditor
// ======================================================================
class CHIMERA_FilterAudioProcessorEditor : public juce::AudioProcessorEditor,
    public juce::Timer,
    public juce::ComboBox::Listener,
    public juce::MouseListener
{
public:
    CHIMERA_FilterAudioProcessorEditor(CHIMERA_FilterAudioProcessor&);
    ~CHIMERA_FilterAudioProcessorEditor() override;

    void paint(juce::Graphics&) override;
    void resized() override;
    void timerCallback() override;
    void comboBoxChanged(juce::ComboBox* box) override;

    void mouseEnter(const juce::MouseEvent& e) override;
    void mouseExit(const juce::MouseEvent& e) override;
    void mouseDown(const juce::MouseEvent& e) override;
    void mouseMove(const juce::MouseEvent& e) override;

private:
    CHIMERA_FilterAudioProcessor& audioProcessor;

    ChimeraStyle customStyle;
    InfoBarComponent infoBar;

    juce::Colour filterBgColors[4] = {
        juce::Colour(200, 240, 245),
        juce::Colour(220, 245, 220),
        juce::Colour(255, 240, 220),
        juce::Colour(245, 220, 245)
    };

    juce::Colour filterColors[4] = {
        juce::Colour(150, 220, 230),
        juce::Colour(180, 230, 180),
        juce::Colour(240, 200, 150),
        juce::Colour(220, 170, 220)
    };

    struct FilterColumn : public juce::Component
    {
        juce::TextButton jumpBtn;
        juce::ComboBox type;
        juce::Label cutLabel, resLabel, driveLabel;
        std::unique_ptr<HoverSlider> cutoff, res, drive;
        juce::Label p1Label, p2Label, p3Label;
        std::unique_ptr<HoverSlider> p1, p2, p3;

        std::unique_ptr<juce::AudioProcessorValueTreeState::ComboBoxAttachment> typeAtt;
        std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> cutoffAtt;
        std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> resAtt;
        std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> driveAtt;
        std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> p1Att;
        std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> p2Att;
        std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> p3Att;

        juce::Colour bgColor;

        FilterColumn() = default;

        void paint(juce::Graphics& g) override
        {
            g.setColour(bgColor);
            g.fillRoundedRectangle(getLocalBounds().toFloat(), 8.0f);
        }

        void resized() override
        {
            auto area = getLocalBounds().reduced(5);
            jumpBtn.setBounds(area.removeFromTop(32).reduced(3, 2));
            type.setBounds(area.removeFromTop(28).reduced(2, 2));
            area.removeFromTop(5);
            auto leftCol = area.removeFromLeft(area.getWidth() / 2);
            cutLabel.setBounds(leftCol.removeFromTop(15));
            cutoff->setBounds(leftCol.removeFromTop(90).reduced(2));
            resLabel.setBounds(leftCol.removeFromTop(15));
            res->setBounds(leftCol.removeFromTop(90).reduced(2));
            driveLabel.setBounds(leftCol.removeFromTop(15));
            drive->setBounds(leftCol.removeFromTop(90).reduced(2));
            auto rightCol = area;
            p1Label.setBounds(rightCol.removeFromTop(15));
            p1->setBounds(rightCol.removeFromTop(90).reduced(2));
            p2Label.setBounds(rightCol.removeFromTop(15));
            p2->setBounds(rightCol.removeFromTop(90).reduced(2));
            p3Label.setBounds(rightCol.removeFromTop(15));
            p3->setBounds(rightCol.removeFromTop(90).reduced(2));
        }
    };

    std::array<FilterColumn, 4> columns;
    std::unique_ptr<CurveVisualizer> curveVisualizer;

    std::unique_ptr<HoverSlider> morphSlider;
    juce::Label morphLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> morphAttach;

    juce::ComboBox modeCombo;
    juce::Label modeLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ComboBoxAttachment> modeAttach;

    std::unique_ptr<HoverSlider> smoothSlider;
    juce::Label physALabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> smoothAttach;

    std::unique_ptr<HoverSlider> physicsParamBSlider;
    juce::Label physBLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> physicsParamBAttach;

    juce::Label logoLabel;
    juce::Image logoImage;

    juce::Label systemLabel;
    juce::ToggleButton colorModeBtn;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> colorModeAttach;

    juce::ToggleButton snapModeBtn;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> snapModeAttach;

    std::unique_ptr<HoverSlider> harmSlider;
    juce::Label harmLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> harmAttach;

    std::unique_ptr<HoverSlider> intervalSlider;
    juce::Label intervalLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> intervalAttach;

    std::unique_ptr<HoverSlider> masterSlider;
    juce::Label masterLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> masterAttach;

    std::unique_ptr<HoverSlider> mixSlider;
    juce::Label mixLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> mixAttach;

    std::unique_ptr<HoverSlider> rootSlider;
    juce::Label rootLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> rootAttach;

    std::unique_ptr<HoverSlider> harmVolSlider;
    juce::Label harmVolLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> harmVolAttach;

    std::unique_ptr<HoverSlider> intVolSlider;
    juce::Label intVolLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> intVolAttach;

    std::unique_ptr<HoverSlider> inputGainSlider;
    juce::Label inputGainLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> inputGainAttach;

    std::unique_ptr<HoverSlider> inputHpfSlider;
    juce::Label inputHpfLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> inputHpfAttach;

    std::unique_ptr<HoverSlider> wetAttackSlider;
    juce::Label wetAttackLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> wetAttackAttach;

    std::unique_ptr<HoverSlider> wetReleaseSlider;
    juce::Label wetReleaseLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> wetReleaseAttach;

    std::unique_ptr<HoverSlider> wetHighShelfSlider;
    juce::Label wetHighShelfLabel;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> wetHighShelfAttach;

    juce::ToggleButton limiterBtn;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> limiterAttach;

    juce::ComboBox oversamplingCombo;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ComboBoxAttachment> oversamplingAttach;
    juce::Label qualityLabel;

    juce::TextButton panicBtn;

    juce::String getNoteNameFromFreq(float freq);
    juce::String getMidiNoteName(int noteNumber);

    void updateFilterControls(int columnIndex);
    void updatePhysicsLabels();

    // Phase 84: Japanese description helpers
    juce::String getFilterDescription(int typeIndex);
    juce::String getPhysicsDescription(int modeIndex);

    // Phase 85: P1/P2/P3 parameter descriptions
    void getFilterParamDescriptions(int typeIndex,
        juce::String& p1Desc, juce::String& p2Desc, juce::String& p3Desc);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(CHIMERA_FilterAudioProcessorEditor)
};