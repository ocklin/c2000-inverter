// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef RESOLVERCALIBRATION_H
#define RESOLVERCALIBRATION_H

#include <math.h>
#include <stdint.h>

namespace encoder {

//! Gain limits (percent) shared by the correction and the calibrator
static const int32_t ResolverMinGainPercent = 50;
static const int32_t ResolverMaxGainPercent = 200;

// Explicit limits: TI's stdint.h hides INTn_MAX in C++ without
// __STDC_LIMIT_MACROS
static const int32_t ResolverInt16Max = 32767;
static const int32_t ResolverInt16Min = -32768;

static const float ResolverTwoPi = 6.28318531f;

/** Centre a raw resolver ADC sample on its offset and scale it.
 * A gain outside [50, 200] % falls back to 100 % (fail safe against a corrupt
 * parameter). The result saturates to the int16_t range.
 */
inline int16_t CorrectResolverSample(uint16_t raw, int32_t offset, int32_t gainPercent)
{
    if (gainPercent < ResolverMinGainPercent || gainPercent > ResolverMaxGainPercent)
        gainPercent = 100;

    int32_t value = (((int32_t)raw - offset) * gainPercent) / 100;
    if (value > ResolverInt16Max)
        value = ResolverInt16Max;
    else if (value < ResolverInt16Min)
        value = ResolverInt16Min;
    return (int16_t)value;
}

/** Find per-channel offsets and the cosine gain while the shaft is turned.
 * Tracks the min/max of both raw channels, ignoring samples that jump more
 * than MaxJump counts from the previous one (ADC glitches). It also tracks the
 * unwrapped angle around the provisional centre, so a partial swing (rocking
 * the shaft) never qualifies. Once at least MinSeconds of samples are in, both
 * spreads reach MinSpread and the shaft has made a full electrical turn, it
 * finishes: Done if the cosine gain is within [50, 200] %, otherwise Failed.
 * Failed too after TimeoutSeconds.
 */
class ResolverCalibrator
{
public:
    //! Values match the rescalstat parameter
    enum State
    {
        Idle = 0,
        Running = 1,
        Done = 2,
        Failed = 3
    };

    static const int32_t  MinSpread = 1200;
    static const uint32_t MinSeconds = 2;
    static const uint32_t TimeoutSeconds = 30;
    //! Largest plausible change between samples; a hand turn moves ~1 count
    static const int32_t  MaxJump = 100;

    ResolverCalibrator()
        : state(Idle), minSamples(0), timeoutSamples(0), samples(0),
          sinMin(0), sinMax(0), cosMin(0), cosMax(0),
          sinOffset(0), cosOffset(0), cosGainPercent(100),
          prevSin(0), prevCos(0), havePrev(false),
          prevAngle(0.0f), haveAngle(false), turns(0.0f)
    {
    }

    void Start(uint32_t pwmFrequency)
    {
        minSamples = MinSeconds * pwmFrequency;
        timeoutSamples = TimeoutSeconds * pwmFrequency;
        samples = 0;
        // Raw samples are uint16_t, so these start outside any real value
        sinMin = cosMin = 65536;
        sinMax = cosMax = -1;
        havePrev = false;
        haveAngle = false;
        turns = 0.0f;
        // Without a sample rate there is no way to time the calibration
        state = pwmFrequency > 0 ? Running : Failed;
    }

    void Update(uint16_t rawSin, uint16_t rawCos)
    {
        if (state != Running)
            return;

        samples = samples + 1U;

        // A sample far from the previous one is a glitch (and so is the one
        // after it, which jumps back): skip both
        const bool plausible = havePrev &&
                               Abs((int32_t)rawSin - prevSin) <= MaxJump &&
                               Abs((int32_t)rawCos - prevCos) <= MaxJump;
        prevSin = rawSin;
        prevCos = rawCos;
        havePrev = true;

        if (plausible)
        {
            if (rawSin < sinMin) sinMin = rawSin;
            if (rawSin > sinMax) sinMax = rawSin;
            if (rawCos < cosMin) cosMin = rawCos;
            if (rawCos > cosMax) cosMax = rawCos;
            TrackTurns(rawSin, rawCos);
        }

        const int32_t sinSpread = sinMax - sinMin;
        const int32_t cosSpread = cosMax - cosMin;
        const bool    fullTurn = fabsf(turns) >= ResolverTwoPi;
        if (samples >= minSamples && sinSpread >= MinSpread &&
            cosSpread >= MinSpread && fullTurn)
        {
            sinOffset = (sinMax + sinMin) / 2;
            cosOffset = (cosMax + cosMin) / 2;
            cosGainPercent = (100 * sinSpread + cosSpread / 2) / cosSpread;
            state = cosGainPercent >= ResolverMinGainPercent &&
                            cosGainPercent <= ResolverMaxGainPercent
                        ? Done
                        : Failed;
        }
        else if (samples >= timeoutSamples)
        {
            state = Failed;
        }
    }

    //! Stop a running calibration without a result
    void Abort()
    {
        if (state == Running)
            state = Failed;
    }

    State   GetState() const { return state; }
    int32_t SinOffset() const { return sinOffset; }
    int32_t CosOffset() const { return cosOffset; }
    int32_t CosGainPercent() const { return cosGainPercent; }

private:
    static int32_t Abs(int32_t x) { return x < 0 ? -x : x; }

    //! Accumulate the unwrapped angle around the provisional centre
    void TrackTurns(uint16_t rawSin, uint16_t rawCos)
    {
        // Wait until the centre estimate means something
        if (sinMax - sinMin < MinSpread / 4 || cosMax - cosMin < MinSpread / 4)
            return;

        const float angle = atan2f((float)((int32_t)rawSin - (sinMax + sinMin) / 2),
                                   (float)((int32_t)rawCos - (cosMax + cosMin) / 2));
        if (haveAngle)
        {
            float diff = angle - prevAngle;
            if (diff > ResolverTwoPi / 2)
                diff -= ResolverTwoPi;
            else if (diff < -ResolverTwoPi / 2)
                diff += ResolverTwoPi;
            turns += diff;
        }
        prevAngle = angle;
        haveAngle = true;
    }

    State    state;
    uint32_t minSamples;
    uint32_t timeoutSamples;
    uint32_t samples;
    int32_t  sinMin, sinMax, cosMin, cosMax;
    int32_t  sinOffset, cosOffset, cosGainPercent;
    int32_t  prevSin, prevCos;
    bool     havePrev;
    float    prevAngle;
    bool     haveAngle;
    float    turns;
};

} // namespace encoder

#endif
