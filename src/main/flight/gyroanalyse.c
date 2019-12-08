/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/* original work by Rav
 * 2018_07 updated by ctzsnooze to post filter, wider Q, different peak detection
 * coding assistance and advice from DieHertz, Rav, eTracer
 * test pilots icr4sh, UAV Tech, Flint723
 */
#include <stdint.h>

#include "platform.h"

#ifdef USE_GYRO_DATA_ANALYSE
#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

#include "fc/core.h"

#include "gyroanalyse.h"

// The FFT splits the frequency domain into an number of bins
// A sampling frequency of 1000 and max frequency of 500 at a window size of 32 gives 16 frequency bins each 31.25Hz wide
// Eg [0,31), [31,62), [62, 93) etc
// for gyro loop >= 4KHz, sample rate 2000 defines FFT range to 1000Hz, 16 bins each 62.5 Hz wide
// NB  FFT_WINDOW_SIZE is set to 32 in gyroanalyse.h
#define FFT_BIN_COUNT             (FFT_WINDOW_SIZE / 2)
// we need 4 steps for each axis
#define DYN_NOTCH_CALC_TICKS      (XYZ_AXIS_COUNT * 4)

#define DYN_NOTCH_OSD_MIN_THROTTLE 20

static uint16_t FAST_RAM_ZERO_INIT   fftSamplingRateHz;
static float FAST_RAM_ZERO_INIT      fftResolution;
static uint8_t FAST_RAM_ZERO_INIT    fftStartBin;
static uint16_t FAST_RAM_ZERO_INIT   dynNotchMaxCtrHz;
static uint8_t dynNotchRange;
static float FAST_RAM_ZERO_INIT      dynNotchQ;
static float FAST_RAM_ZERO_INIT      dynNotch1Ctr;
static float FAST_RAM_ZERO_INIT      dynNotch2Ctr;
static uint16_t FAST_RAM_ZERO_INIT   dynNotchMinHz;
static bool FAST_RAM dualNotch = true;
static uint16_t FAST_RAM_ZERO_INIT dynNotchMaxFFT;
static FAST_RAM_ZERO_INIT flight_dynamics_index_t gyroDebugAxis;

// Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
static FAST_RAM_ZERO_INIT float hanningWindow[FFT_WINDOW_SIZE];

void gyroDataAnalyseInit()
{
    dynNotchRange     = gyroConfig()->dyn_notch_range;
    fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_LOW;
    dynNotchMinHz     = gyroConfig()->dyn_notch_min_hz;
    dynNotch1Ctr      = 1 - gyroConfig()->dyn_notch_width_percent / 100.0f;
    dynNotch2Ctr      = 1 + gyroConfig()->dyn_notch_width_percent / 100.0f;
    dynNotchQ         = gyroConfig()->dyn_notch_q / 100.0f;
 
    if (gyroConfig()->dyn_notch_width_percent == 0) {
        dualNotch = false;
    }

    if (dynNotchRange == DYN_NOTCH_RANGE_AUTO) {
        if (gyroConfig()->dyn_lpf_gyro_max_hz > 333) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_MEDIUM;
        }
        if (gyroConfig()->dyn_lpf_gyro_max_hz > 610) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_HIGH;
        }
    } else {
        if (dynNotchRange == DYN_NOTCH_RANGE_HIGH) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_HIGH;
        }
        else if (dynNotchRange == DYN_NOTCH_RANGE_MEDIUM) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_MEDIUM;
        }
    }
    // If we get at least 3 samples then use the default FFT sample frequency
    // otherwise we need to calculate a FFT sample frequency to ensure we get 3 samples (gyro loops < 4K)
    const int gyroLoopRateHz = lrintf((1.0f / gyro.targetLooptime) * 1e6f);
    
    fftSamplingRateHz = gyroLoopRateHz / MAX(1, gyroLoopRateHz / fftSamplingRateHz);

    fftResolution = (float)fftSamplingRateHz / FFT_WINDOW_SIZE;

    fftStartBin = dynNotchMinHz / lrintf(fftResolution);

    dynNotchMaxCtrHz = fftSamplingRateHz * 0.48f; //Stay below Nyquist

    for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
        hanningWindow[i] = (0.5f - 0.5f * cos_approx(2 * M_PIf * i / (FFT_WINDOW_SIZE - 1)));
    }

    gyroDebugAxis = gyroConfig()->gyro_filter_debug_axis;

}

void gyroDataAnalyseStateInit(gyroAnalyseState_t *state)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    // *** can this next line be removed ??? ***
    gyroDataAnalyseInit();

    const uint16_t samplingFrequency = 1000000 / gyro.targetLooptime;
    state->maxSampleCount = samplingFrequency / fftSamplingRateHz;
    state->maxSampleCountRcp = 1.f / state->maxSampleCount;

    arm_rfft_fast_init_f32(&state->fftInstance, FFT_WINDOW_SIZE);

//    recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
//    at 4khz gyro loop rate this means 4khz / 4 / 3 = 333Hz => update every 3ms
//    for gyro rate > 16kHz, we have update frequency of 1kHz => 1ms
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // any init value
        state->updateCenterFreq[axis] = false;
        state->centerFreq[axis] = dynNotchMaxCtrHz;
        state->centerPeak[axis] = 4;
        biquadFilterInit(&state->gyroNotch[axis], dynNotchMaxCtrHz, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
    }
}

void gyroDataAnalysePush(gyroAnalyseState_t *state, const int axis, const float sample)
{
    state->oversampledGyroAccumulator[axis] = sample;
}

static void gyroDataAnalyseUpdate(gyroAnalyseState_t *state);

/*
 * Collect gyro data, to be analysed in gyroDataAnalyseUpdate function
 */
void gyroDataAnalyse(gyroAnalyseState_t *state)
{
    // samples should have been pushed by `gyroDataAnalysePush`
    // if gyro sampling is > 1kHz, accumulate multiple samples
    state->sampleCount++;

    // this runs at 1kHz
    if (state->sampleCount == state->maxSampleCount) {
        state->sampleCount = 0;

        // calculate mean value of accumulated samples
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            float sample = state->oversampledGyroAccumulator[axis]; // * state->maxSampleCountRcp;
            state->downsampledGyroData[axis][state->circularBufferIdx] = sample;
            if (axis == gyroDebugAxis) {
                DEBUG_SET(DEBUG_FFT, 2, lrintf(sample));
            }

            state->oversampledGyroAccumulator[axis] = 0;
        }

        state->circularBufferIdx = (state->circularBufferIdx + 1) % FFT_WINDOW_SIZE;

        // We need DYN_NOTCH_CALC_TICKS tick to update all axis with newly sampled value
        state->updateTicks = DYN_NOTCH_CALC_TICKS;
    }

    // calculate FFT and update filters
    if (state->updateTicks > 0) {
        gyroDataAnalyseUpdate(state);
        --state->updateTicks;
    }
}

void stage_rfft_f32(arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut);
void arm_cfft_radix8by2_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_cfft_radix8by4_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_radix8_butterfly_f32(float32_t *pSrc, uint16_t fftLen, const float32_t *pCoef, uint16_t twidCoefModifier);
void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen, const uint16_t *pBitRevTable);

float calculateWeight(gyroAnalyseState_t *state, uint8_t k) {
    if (k == FFT_BIN_COUNT - 1){
        return k;
    } else {
        //Credit:  http://sam-koblenski.blogspot.com/2015/11/everyday-dsp-for-programmers-spectral.html
        return k + (state->fftData[k + 1] - state->fftData[k - 1]) / (4 * state->fftData[k] - 2 * state->fftData[k - 1] - 2 * state->fftData[k + 1]);
    }
}

/*
 * Analyse last gyro data from the last FFT_WINDOW_SIZE milliseconds
 */
static FAST_CODE_NOINLINE void gyroDataAnalyseUpdate(gyroAnalyseState_t *state)
{
    enum {
        STEP_ARM_CFFT_F32,
        STEP_BITREVERSAL,
        STEP_STAGE_RFFT_F32,
        STEP_ARM_CMPLX_MAG_F32,
        STEP_CALC_FREQUENCIES,
        STEP_UPDATE_FILTERS,
        STEP_HANNING,
        STEP_COUNT
    };

    arm_cfft_instance_f32 *Sint = &(state->fftInstance.Sint);

    uint32_t startTime = 0;
    if (debugMode == (DEBUG_FFT_TIME)) {
        startTime = micros();
    }

    DEBUG_SET(DEBUG_FFT_TIME, 0, state->updateStep);
    switch (state->updateStep) {
        case STEP_ARM_CFFT_F32:
        {
            switch (FFT_BIN_COUNT) {
            case 16:
                // 16us
                arm_cfft_radix8by2_f32(Sint, state->fftData);
                break;
            case 32:
                // 35us
                arm_cfft_radix8by4_f32(Sint, state->fftData);
                break;
            case 64:
                // 70us
                arm_radix8_butterfly_f32(state->fftData, FFT_BIN_COUNT, Sint->pTwiddle, 1);
                break;
            }
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            break;
        }
        case STEP_BITREVERSAL:
        {
            // 6us
            arm_bitreversal_32((uint32_t*) state->fftData, Sint->bitRevLength, Sint->pBitRevTable);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            state->updateStep++;
            FALLTHROUGH;
        }
        case STEP_STAGE_RFFT_F32:
        {
            // 14us
            // this does not work in place => fftData AND rfftData needed
            stage_rfft_f32(&state->fftInstance, state->fftData, state->rfftData);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            break;
        }
        case STEP_ARM_CMPLX_MAG_F32:
        {
            // 8us
            arm_cmplx_mag_f32(state->rfftData, state->fftData, FFT_BIN_COUNT);
            DEBUG_SET(DEBUG_FFT_TIME, 2, micros() - startTime);
            state->updateStep++;
            FALLTHROUGH;
        }
        case STEP_CALC_FREQUENCIES:
        {
            //Default to the last bin.
            int k = FFT_BIN_COUNT - 1;

            for (int i = FFT_BIN_COUNT - 2; i >= fftStartBin; i--) {
                if (lrintf(state->fftData[i] * state->fftData[i]) > lrintf(state->fftData[k] * state->fftData[k])){
                    k = i;
                }
            }

            uint16_t centerFreq;

            //calculate weighted bin.
            centerFreq = lrintf(calculateWeight(state, k) * fftResolution + 0.5f * fftResolution);

            //Store peak frequency for OSD.
            if (calculateThrottlePercentAbs() > DYN_NOTCH_OSD_MIN_THROTTLE) {
                uint16_t peakFreq = centerFreq;
                dynNotchMaxFFT = MAX(dynNotchMaxFFT, peakFreq);
            }

            //Constrain center frequencies.
            centerFreq = constrain(centerFreq, dynNotchMinHz, dynNotchMaxCtrHz);

            state->centerPeak[state->updateAxis] *= 0.995f;

            if (k != FFT_BIN_COUNT - 1) {
                //if no existing bin found
                if (lrintf(state->fftData[k] * state->fftData[k]) > lrintf(state->centerPeak[state->updateAxis] * state->centerPeak[state->updateAxis]))
                    {
                        state->updateCenterFreq[state->updateAxis] = true;
                        state->centerFreq[state->updateAxis] = centerFreq;
                        state->centerPeak[state->updateAxis] = state->fftData[k];
                    }
            }

            if (state->updateAxis == gyroDebugAxis) {
                DEBUG_SET(DEBUG_FFT, 3, lrintf(calculateWeight(state, k) * 100));
                DEBUG_SET(DEBUG_FFT_FREQ, 0, state->centerFreq[state->updateAxis]);
                DEBUG_SET(DEBUG_FFT_FREQ, 1, centerFreq);
            }

            // Debug FFT_Freq carries raw gyro, gyro after first filter set, FFT centre for roll and for pitch
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            break;
        }
        case STEP_UPDATE_FILTERS:
        {
            // 7us

            if (state->updateCenterFreq[state->updateAxis]) {
                biquadFilterUpdate(&state->gyroNotch[state->updateAxis], state->centerFreq[state->updateAxis], gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
                state->updateCenterFreq[state->updateAxis] = false;
            }

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            state->updateAxis = (state->updateAxis + 1) % XYZ_AXIS_COUNT;
            state->updateStep++;
            FALLTHROUGH;
        }
        case STEP_HANNING:
        {
            // 5us
            // apply hanning window to gyro samples and store result in fftData
            // hanning starts and ends with 0, could be skipped for minor speed improvement
            const uint8_t ringBufIdx = FFT_WINDOW_SIZE - state->circularBufferIdx;
            arm_mult_f32(&state->downsampledGyroData[state->updateAxis][state->circularBufferIdx], &hanningWindow[0], &state->fftData[0], ringBufIdx);
            if (state->circularBufferIdx > 0) {
                arm_mult_f32(&state->downsampledGyroData[state->updateAxis][0], &hanningWindow[ringBufIdx], &state->fftData[ringBufIdx], state->circularBufferIdx);
            }

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
        }
    }

    state->updateStep = (state->updateStep + 1) % STEP_COUNT;
}

//Apply notch filters to gyro data.
float FAST_CODE gyroDataAnalyseApply(gyroAnalyseState_t *state, int axis, float values) {
    values = biquadFilterApplyDF1(&state->gyroNotch[axis], values);
    return values;
}

uint16_t getMaxFFT(void) {
    return dynNotchMaxFFT;
}

void resetMaxFFT(void) {
    dynNotchMaxFFT = 0;
}

#endif // USE_GYRO_DATA_ANALYSE
