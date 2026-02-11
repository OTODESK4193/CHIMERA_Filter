/*
  ==============================================================================
    MorphPhysics.h
    Phase 85: Sloth & Bounce Algorithm Improvements

    Changes:
    - Sloth: Time range expanded to 0.008-4.0 seconds (was 0.0001-0.05s)
    - Bounce: Gravity scaled down, clearer bounce behavior (A-style)

    Layout Mapping:
    [Mode]      [Param A (Primary)]       [Param B (Secondary)]
    ---------------------------------------------------------------
    Linear:     Speed (速度)              Inertia (慣性/加減速)
    Stepped:    Steps (段数)              Smooth (角の丸み)
    Elastic:    Tension (バネ強度)        Damping (摩擦/減衰)
    Sloth:      Time (遅延時間)           Curve (カーブ形状)
    Chaos:      Speed (カオス速度)        Tether (ターゲット拘束力)
    Bounce:     Gravity (重力)            Bounciness (反発係数)

    Previous: Phase 84 - Japanese InfoBar
  ==============================================================================
*/

#pragma once
#include <JuceHeader.h>
#include <cmath>

class MorphPhysics
{
public:
    enum Mode { Linear, Stepped, Elastic, Sloth, Chaos, Bounce };

    MorphPhysics() { reset(); }

    void prepare(double newSampleRate)
    {
        sampleRate = (newSampleRate > 0.0) ? newSampleRate : 44100.0;
        reset();
    }

    void reset()
    {
        currentPos = 0.0f;
        velocity = 0.0f;
        targetPos = 0.0f;

        // Linear Inertia State
        linearVelocity = 0.0f;

        // Chaos State (Lorenz)
        lorenzX = 0.1f;
        lorenzY = 0.0f;
        lorenzZ = 0.0f;

        // ★ Phase 85: Bounce state
        bounceInitialized = false;
    }

    // ======================================================================
    // PROCESS BLOCK
    // paramA: Previously "SmoothTime" (Range approx 0.1 to 50.0)
    // paramB: New Secondary Param (Range 0.0 to 1.0)
    // ======================================================================
    float process(float inputTarget, Mode mode, float paramA, float paramB)
    {
        // NaN Guard
        if (std::isnan(inputTarget)) inputTarget = targetPos;

        // ★ Phase 85: Detect target change for Bounce mode
        bool targetChanged = (std::abs(inputTarget - targetPos) > 0.001f);
        targetPos = inputTarget;

        float dt = 1.0f / (float)sampleRate;

        // Clamp ParamB to 0.0 - 1.0 just in case
        float pB = juce::jlimit(0.0f, 1.0f, paramB);

        switch (mode)
        {
            // ------------------------------------------------------------
            // 1. Linear (Constant Velocity with Inertia)
            // Param A: Speed (Time to traverse)
            // Param B: Inertia (Acceleration smoothing)
            // ------------------------------------------------------------
        case Linear:
        {
            // Base speed calculation
            float speed = 100.0f / juce::jmax(0.1f, paramA);

            float diff = targetPos - currentPos;
            float desiredVel = 0.0f;

            if (std::abs(diff) > 0.0001f) {
                desiredVel = (diff > 0) ? speed : -speed;
            }

            // Apply Inertia (Param B)
            float velAlpha = 1.0f - std::exp(-1.0f / (0.001f + pB * 0.5f));

            if (pB < 0.01f) {
                linearVelocity = desiredVel;
            }
            else {
                linearVelocity += (desiredVel - linearVelocity) * velAlpha * dt * 100.0f;
            }

            // Cap velocity to avoid overshoot if close
            float maxStep = std::abs(diff) / dt;
            if (std::abs(linearVelocity) > maxStep) {
                linearVelocity = (diff > 0) ? maxStep : -maxStep;
            }

            currentPos += linearVelocity * dt;

            // Snap if very close
            if (std::abs(targetPos - currentPos) < 0.001f && std::abs(linearVelocity) < 0.1f) {
                currentPos = targetPos;
                linearVelocity = 0.0f;
            }
            break;
        }

        // ------------------------------------------------------------
        // 2. Stepped (Quantized Steps with Smoothing)
        // Param A: Steps (2 to 24)
        // Param B: Smoothness (0.0 = Hard, 1.0 = Soft)
        // ------------------------------------------------------------
        case Stepped:
        {
            // Map A to Step Count
            float steps = juce::jlimit(2.0f, 24.0f, paramA * 0.5f);

            // Quantize Target
            float quantizedTarget = std::floor(targetPos * steps) / steps;

            // Param B controls the smoothing time to reach the step
            float smoothTimeMs = 1.0f + pB * 200.0f;
            float smoothFactor = 1.0f - std::exp(-1.0f / (smoothTimeMs * 0.001f * (float)sampleRate));

            currentPos += (quantizedTarget - currentPos) * smoothFactor;
            break;
        }

        // ------------------------------------------------------------
        // 3. Elastic (Mass-Spring-Damper)
        // Param A: Tension (Spring Stiffness)
        // Param B: Damping (Friction)
        // ------------------------------------------------------------
        case Elastic:
        {
            float k = 500.0f / juce::jmax(0.1f, paramA);

            float criticalDamp = 2.0f * std::sqrt(k);
            float dampingCoef = pB * criticalDamp;
            dampingCoef = juce::jmax(dampingCoef, 0.05f);

            float displacement = currentPos - targetPos;
            float acceleration = (-k * displacement) - (dampingCoef * velocity);

            velocity += acceleration * dt;
            velocity = juce::jlimit(-5000.0f, 5000.0f, velocity);

            currentPos += velocity * dt;
            break;
        }

        // ------------------------------------------------------------
        // 4. Sloth (Variable Curve Smoothing)
        // ★ Phase 85: Extended time range (0.008 - 4.0 seconds)
        // Param A: Time Constant
        // Param B: Curve (0.0 = Linear-ish, 1.0 = Logarithmic/S-Curve)
        // ------------------------------------------------------------
        case Sloth:
        {
            // ★ Phase 85: Changed from 0.001f to 0.08f for 4 second max
            // paramA range: 0.1 - 50.0
            // timeSec range: 0.008 - 4.0 seconds
            float timeSec = juce::jmax(0.1f, paramA) * 0.08f;

            // Basic exponential smooth
            float alpha = 1.0f - std::exp(-1.0f / (timeSec * (float)sampleRate));

            float diff = targetPos - currentPos;

            // Curve shaping (Param B)
            if (pB > 0.1f) {
                float dist = std::abs(diff);
                float boost = 1.0f + (dist * pB * 5.0f);
                alpha = juce::jlimit(0.0f, 1.0f, alpha * boost);
            }

            currentPos += diff * alpha;
            break;
        }

        // ------------------------------------------------------------
        // 5. Chaos (Lorenz Attractor with Tether)
        // Param A: Chaos Speed
        // Param B: Tether (0.0 = Loose/Wild, 1.0 = Tight to Target)
        // ------------------------------------------------------------
        case Chaos:
        {
            const float sigma = 10.0f;
            const float rho = 28.0f;
            const float beta = 8.0f / 3.0f;

            float chaosDt = dt * (100.0f / juce::jmax(1.0f, paramA));

            float dx = sigma * (lorenzY - lorenzX);
            float dy = lorenzX * (rho - lorenzZ) - lorenzY;
            float dz = lorenzX * lorenzY - beta * lorenzZ;

            lorenzX += dx * chaosDt;
            lorenzY += dy * chaosDt;
            lorenzZ += dz * chaosDt;

            float normalizedChaos = (lorenzX / 40.0f) + 0.5f;
            normalizedChaos = juce::jlimit(0.0f, 1.0f, normalizedChaos);

            float chaosOffset = normalizedChaos - 0.5f;
            float chaosAmp = 1.0f - (pB * 0.8f);

            currentPos = targetPos + (chaosOffset * chaosAmp);
            break;
        }

        // ------------------------------------------------------------
        // 6. Bounce (Gravity with Variable Elasticity)
        // ★ Phase 85: Improved algorithm (A-style)
        // - Current position falls toward target (floor)
        // - Bounces on contact with restitution
        // Param A: Gravity (gentler scaling)
        // Param B: Bounciness (Restitution)
        // ------------------------------------------------------------
        case Bounce:
        {
            // ★ Phase 85: Gentler gravity scaling (was paramA * 10.0f)
            // paramA range: 0.1 - 50.0 → gravity: 0.2 - 100.0
            float gravity = paramA * 2.0f;

            // Floor is the target position
            float floor = targetPos;

            // ★ Phase 85: When target changes, give initial velocity
            // This creates the "drop and bounce" effect
            if (targetChanged && !bounceInitialized) {
                // Start with some initial velocity toward the target
                velocity = 0.0f;
                bounceInitialized = true;
            }

            // Determine direction to floor
            float direction = (floor > currentPos) ? 1.0f : -1.0f;

            // Apply gravity toward floor
            velocity += direction * gravity * dt;

            // Velocity limiting for stability
            velocity = juce::jlimit(-200.0f, 200.0f, velocity);

            // Update position
            currentPos += velocity * dt;

            // ★ Phase 85: Restitution (bounce coefficient)
            // pB = 0: Dead bounce (lead ball)
            // pB = 1: High bounce (super ball) - capped at 0.9 to prevent infinite bounce
            float restitution = pB * 0.9f;

            // ★ Phase 85: Collision detection - crossed the floor?
            bool crossedFloor = false;
            if (direction > 0 && currentPos >= floor) {
                crossedFloor = true;
            }
            else if (direction < 0 && currentPos <= floor) {
                crossedFloor = true;
            }

            if (crossedFloor) {
                // Snap to floor
                currentPos = floor;

                // Reverse velocity with energy loss
                velocity = -velocity * restitution;

                // ★ Phase 85: Stop if energy is too low
                if (std::abs(velocity) < 0.5f) {
                    velocity = 0.0f;
                    currentPos = floor;
                    bounceInitialized = false;  // Ready for next target change
                }
            }

            // ★ Phase 85: Reset bounce state when settled at target
            if (std::abs(currentPos - floor) < 0.001f && std::abs(velocity) < 0.1f) {
                bounceInitialized = false;
            }

            break;
        }
        }

        // Safety
        if (std::isnan(currentPos) || std::isinf(currentPos)) {
            reset();
            currentPos = targetPos;
        }

        return juce::jlimit(0.0f, 1.0f, currentPos);
    }

private:
    double sampleRate = 44100.0;
    float currentPos = 0.0f;
    float velocity = 0.0f;
    float targetPos = 0.0f;

    // Linear specific
    float linearVelocity = 0.0f;

    // Chaos State
    float lorenzX = 0.1f;
    float lorenzY = 0.0f;
    float lorenzZ = 0.0f;

    // ★ Phase 85: Bounce state
    bool bounceInitialized = false;

    juce::Random random;
};