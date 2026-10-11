// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef RESOLVERCALIBRATION_H
#define RESOLVERCALIBRATION_H

#include <stdint.h>

namespace encoder {

//! Gain limits (percent) shared by the correction and the calibrator
static const int32_t ResolverMinGainPercent = 50;
static const int32_t ResolverMaxGainPercent = 200;

// Explicit limits: TI's stdint.h hides INTn_MAX in C++ without
// __STDC_LIMIT_MACROS
static const int32_t ResolverInt16Max = 32767;
static const int32_t ResolverInt16Min = -32768;

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
 * Tracks the min/max of both raw channels. Once at least MinSeconds of samples
 * are in and both spreads reach MinSpread, it finishes: DONE if the cosine gain
 * is within [50, 200] %, otherwise FAILED. FAILED too after TimeoutSeconds.
 */
class ResolverCalibrator
{
public:
    //! Values match the rescalstat parameter
    enum State
    {
        IDLE = 0,
        RUNNING = 1,
        DONE = 2,
        FAILED = 3
    };

    static const int32_t  MinSpread = 1200;
    static const uint32_t MinSeconds = 2;
    static const uint32_t TimeoutSeconds = 30;

    ResolverCalibrator()
        : state(IDLE), minSamples(0), timeoutSamples(0), samples(0),
          sinMin(0), sinMax(0), cosMin(0), cosMax(0),
          sinOffset(0), cosOffset(0), cosGainPercent(100)
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
        // Without a sample rate there is no way to time the calibration
        state = pwmFrequency > 0 ? RUNNING : FAILED;
    }

    void Update(uint16_t rawSin, uint16_t rawCos)
    {
        if (state != RUNNING)
            return;

        if (rawSin < sinMin) sinMin = rawSin;
        if (rawSin > sinMax) sinMax = rawSin;
        if (rawCos < cosMin) cosMin = rawCos;
        if (rawCos > cosMax) cosMax = rawCos;
        samples = samples + 1U;

        const int32_t sinSpread = sinMax - sinMin;
        const int32_t cosSpread = cosMax - cosMin;
        if (samples >= minSamples && sinSpread >= MinSpread && cosSpread >= MinSpread)
        {
            sinOffset = (sinMax + sinMin) / 2;
            cosOffset = (cosMax + cosMin) / 2;
            cosGainPercent = (100 * sinSpread + cosSpread / 2) / cosSpread;
            state = cosGainPercent >= ResolverMinGainPercent &&
                            cosGainPercent <= ResolverMaxGainPercent
                        ? DONE
                        : FAILED;
        }
        else if (samples >= timeoutSamples)
        {
            state = FAILED;
        }
    }

    State   GetState() const { return state; }
    int32_t SinOffset() const { return sinOffset; }
    int32_t CosOffset() const { return cosOffset; }
    int32_t CosGainPercent() const { return cosGainPercent; }

private:
    State    state;
    uint32_t minSamples;
    uint32_t timeoutSamples;
    uint32_t samples;
    int32_t  sinMin, sinMax, cosMin, cosMax;
    int32_t  sinOffset, cosOffset, cosGainPercent;
};

} // namespace encoder

#endif
