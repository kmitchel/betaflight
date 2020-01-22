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

// FFT_WINDOW_SIZE defaults to 32 (gyroanalyse.h)
// We get 16 frequency bins from 32 consecutive data values
// Bin 0 is DC and can't be used.  
// Only bins 1 to 15 are usable.

// A gyro sample is collected every gyro loop
// maxSampleCount recent gyro values are accumulated and averaged
// to ensure that 32 samples are collected at the right rate for the required FFT bandwidth

// For DYN_NOTCH_RANGE_HZ_MEDIUM, fftSamplingRateHz is 1333Hz (gyro.h ~90)
// For an 8k gyro loop, maxSampleCount = 6.  This means 6 sequential gyro data points are averaged
//   (For a 4k gyro loop, maxSampleCount is 3; 3 gyro data points are averaged)
//   (I'm not sure how this would work at 2k, but it should work at 1.333k)
// When sampleCount reaches maxSampleCount, the averaged gyro value is put into the circular buffer of 32 samples
// Hence the FFT input buffer takes 32 * 0.75ms = 24ms to completely be filled with clean new values

// The FFT code is split into steps.  It takes 4 gyro loops to calculate the FFT for one axis
//   (gyroDataAnalyseUpdate has 8 steps, but only four breaks)
// Since there are three axes, it takes 12 gyro loops to completely update all axes.
// At 8k, any one axis gets updated at 8000 / 12 or 666hz or every 1.5ms
//   (at 4k, it takes twice as long, ie each axis updates every 3ms)

// The Hanning window step loads gyro data (32 data points) for one axis from the circular buffer into fftData[i]
//   and applies the hanning window to the edge values
// Calculation steps 1 and 2 then calculate the fft output (32 data points) and put that back into the same fftData[i] array.
// We then use fftData[i] array for frequency centre calculations for that axis

// Each FFT output bin has width fftSamplingRateHz/32, ie 41.65Hz per bin at 1333Hz 
// Usable bandwidth is half this, ie 666Hz bandwidth in MEDIUM, bin 1 indicating 41.65hz, bin 2 83.3hz etc
// for LOW we get 16 x 31.25ms bins to 500hz, 
// for HIGH we get 16 x 62.5ms bins to 1000Hz

// ** issues and notes **
// 1.  The updateTicks counter seems useless unless we are running 32k, so isn't needed currently.
// DYN_NOTCH_CALC_TICKS is a value set to XYZ_AXIS_COUNT * 4, ie 12
// it stops gyroDataAnalyseUpdate once all 12 iterations have calculated the FFT for each axis
// when sampleCount is complete (new FFT value arrives), the updateTicks counter is reset to 12
// it then starts counting down to zero, and while non-zero, gyroDataAnalyseUpdate is called
// at 8k, MEDIUM, sampleCount is complete every 6 loops, so updateTicks never, reaches zero.

// 2..Is 2k gyro loop likely to work?

// 3.  What about Bosch gyro loop rates?  

// 4.  does gyroDataAnalyseInit need to be called every time gyroDataAnalyseStateInit is called ??

//#define FFT_WINDOW_SIZE 32  -> this is in gyroanalyse.h
#define DYN_NOTCH_SMOOTH_HZ 5
#define FFT_BIN_COUNT             (FFT_WINDOW_SIZE / 2) // 16
#define DYN_NOTCH_CALC_TICKS      (XYZ_AXIS_COUNT * 4) // 4 steps per axis
#define DYN_NOTCH_OSD_MIN_THROTTLE 20

static uint16_t FAST_RAM_ZERO_INIT   fftSamplingRateHz;
static float FAST_RAM_ZERO_INIT      fftResolution;
static uint8_t FAST_RAM_ZERO_INIT    fftStartBin;
static uint16_t FAST_RAM_ZERO_INIT   dynNotchMaxCtrHz;
static uint8_t                       dynamicFilterRange;
static float FAST_RAM_ZERO_INIT      dynNotchQ;
static float FAST_RAM_ZERO_INIT      dynNotch1Ctr;
static float FAST_RAM_ZERO_INIT      dynNotch2Ctr;
static uint16_t FAST_RAM_ZERO_INIT   dynNotchMinHz;
static bool FAST_RAM                 dualNotch = true;
static uint16_t FAST_RAM_ZERO_INIT   dynNotchMaxFFT;
static float FAST_RAM_ZERO_INIT      smoothFactor;

// Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
static FAST_RAM_ZERO_INIT float hanningWindow[FFT_WINDOW_SIZE];

void gyroDataAnalyseInit(uint32_t targetLooptimeUs)
{
#ifdef USE_MULTI_GYRO
    static bool gyroAnalyseInitialized;
    if (gyroAnalyseInitialized) {
        return;
    }
    gyroAnalyseInitialized = true;
#endif

    dynamicFilterRange = gyroConfig()->dyn_notch_range;
    fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_LOW;
    dynNotch1Ctr = 1 - gyroConfig()->dyn_notch_width_percent / 100.0f;
    dynNotch2Ctr = 1 + gyroConfig()->dyn_notch_width_percent / 100.0f;
    dynNotchQ = gyroConfig()->dyn_notch_q / 100.0f;
    dynNotchMinHz = gyroConfig()->dyn_notch_min_hz;

    if (gyroConfig()->dyn_notch_width_percent == 0) {
        dualNotch = false;
    }

    if (dynamicFilterRange == DYN_NOTCH_RANGE_AUTO) {
        if (gyroConfig()->dyn_lpf_gyro_max_hz > 333) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_MEDIUM;
        }
        if (gyroConfig()->dyn_lpf_gyro_max_hz > 610) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_HIGH;
        }
    } else {
        if (dynamicFilterRange == DYN_NOTCH_RANGE_HIGH) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_HIGH;
        }
        else if (dynamicFilterRange == DYN_NOTCH_RANGE_MEDIUM) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_MEDIUM;
        }
    }
    // If we get at least 3 samples then use the default FFT sample frequency
    // otherwise we need to calculate a FFT sample frequency to ensure we get 3 samples (gyro loops < 4K)
    const int gyroLoopRateHz = lrintf((1.0f / targetLooptimeUs) * 1e6f);
    
    fftSamplingRateHz = MIN((gyroLoopRateHz / 3), fftSamplingRateHz); // 1333hz at 8k medium

    fftResolution = (float)fftSamplingRateHz / FFT_WINDOW_SIZE; // 41.65hz per bin for medium

    fftStartBin = MAX(2, dynNotchMinHz / lrintf(fftResolution)); // can't use bin 0 because it is DC.

    dynNotchMaxCtrHz = fftSamplingRateHz * 0.48; // Stay below Nyquist; frequency at which each axis updates

    smoothFactor = 2 * M_PIf * DYN_NOTCH_SMOOTH_HZ / (gyroLoopRateHz / 12); // minimum PT1 k value

    for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
        hanningWindow[i] = (0.5f - 0.5f * cos_approx(2 * M_PIf * i / (FFT_WINDOW_SIZE - 1)));
        // for testing, to disable, set to hanningWindow[i] = 1.0f;
    }
}

void gyroDataAnalyseStateInit(gyroAnalyseState_t *state, uint32_t targetLooptimeUs)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    // *** can this next line be removed ??? ***
    gyroDataAnalyseInit(targetLooptimeUs);

    const uint16_t samplingFrequency = 1000000 / targetLooptimeUs;
    state->maxSampleCount = samplingFrequency / fftSamplingRateHz;
    state->maxSampleCountRcp = 1.f / state->maxSampleCount;

    arm_rfft_fast_init_f32(&state->fftInstance, FFT_WINDOW_SIZE);

//    recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
//    at 4kHz gyro loop rate this means 8kHz / 4 / 3 = 666Hz => update every 1.5ms
//    at 4kHz gyro loop rate this means 4kHz / 4 / 3 = 333Hz => update every 3ms
//    for gyro rate > 16kHz, we have update frequency of 1kHz => 1ms
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // any init value
        state->centerFreq[axis] = dynNotchMaxCtrHz;
    }
}

void gyroDataAnalysePush(gyroAnalyseState_t *state, const int axis, const float sample)
{
    state->oversampledGyroAccumulator[axis] += sample;
}

static void gyroDataAnalyseUpdate(gyroAnalyseState_t *state, biquadFilter_t *notchFilterDyn, biquadFilter_t *notchFilterDyn2);

/*
 * Collect gyro data, to be analysed in gyroDataAnalyseUpdate function
 */
void gyroDataAnalyse(gyroAnalyseState_t *state, biquadFilter_t *notchFilterDyn, biquadFilter_t *notchFilterDyn2)
{
    // samples should have been pushed by `gyroDataAnalysePush`
    // if gyro sampling is > 1kHz, accumulate multiple samples
    state->sampleCount++;

    // this runs at 1kHz
    if (state->sampleCount == state->maxSampleCount) {
        state->sampleCount = 0;

        // calculate mean value of accumulated samples
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            float sample = state->oversampledGyroAccumulator[axis] * state->maxSampleCountRcp;
            state->downsampledGyroData[axis][state->circularBufferIdx] = sample;
            if (axis == 0) {
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
        gyroDataAnalyseUpdate(state, notchFilterDyn, notchFilterDyn2);
        --state->updateTicks;
    }
}

void stage_rfft_f32(arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut);
void arm_cfft_radix8by2_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_cfft_radix8by4_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_radix8_butterfly_f32(float32_t *pSrc, uint16_t fftLen, const float32_t *pCoef, uint16_t twidCoefModifier);
void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen, const uint16_t *pBitRevTable);

/*
 * Analyse last gyro data from the last FFT_WINDOW_SIZE milliseconds
 */
static FAST_CODE_NOINLINE void gyroDataAnalyseUpdate(gyroAnalyseState_t *state, biquadFilter_t *notchFilterDyn, biquadFilter_t *notchFilterDyn2)
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
            // identify max bin and max/min heights 
            float dataMax = 0.0f;
            float dataMin = 1.0f;
            uint8_t binMax = 0;
            float dataMinHi = 1.0f;
            for (int i = fftStartBin; i < FFT_BIN_COUNT; i++) {
                if (state->fftData[i] > state->fftData[i - 1]) { // bin height increased
                    if (state->fftData[i] > dataMax) {
                        dataMax = state->fftData[i];
                        binMax = i;  // tallest bin so far
                    }
                }
            }
            if (binMax == 0) { // no bin increase, hold prev max bin, dataMin = 1 dataMax = 0, ie move slow
                binMax = lrintf(state->centerFreq[state->updateAxis] / fftResolution);
            } else { // there was a max, find min
                for (int i = binMax - 1; i > 1; i--) { // look for min below max
                    dataMin = state->fftData[i];
                    if (state->fftData[i - 1] > state->fftData[i]) { // up step below this one
                        break;
                    }
                }
                for (int i = binMax + 1; i < (FFT_BIN_COUNT - 1); i++) { // // look for min above max
                    dataMinHi = state->fftData[i];
                    if (state->fftData[i] < state->fftData[i + 1]) { // up step above this one
                        break;
                    }
                }
            }
            dataMin = fminf(dataMin, dataMinHi);

            // accumulate fftSum and fftWeightedSum from peak bin, and shoulder bins either side of peak
            float squaredData = state->fftData[binMax] * state->fftData[binMax];
            float fftSum = squaredData;
            float fftWeightedSum = squaredData * binMax;
            // accumulate upper shoulder unless it would be FFT_BIN_COUNT
            binMax += 1;
            if (binMax < FFT_BIN_COUNT){
                squaredData = state->fftData[binMax] * state->fftData[binMax];
                fftSum += squaredData;
                fftWeightedSum += squaredData * binMax;
            }
            binMax -= 1;
            // accumulate lower shoulder unless lower shoulder would be bin 0 (DC)
            if (binMax > 1){
                binMax -= 1;
                squaredData = state->fftData[binMax] * state->fftData[binMax];
                fftSum += squaredData;
                fftWeightedSum += squaredData * binMax;
                binMax += 1;
            }

            // get centerFreq in Hz from weighted bins
            float centerFreq = dynNotchMaxCtrHz;
            float fftMeanIndex = 0;
            if (fftSum > 0) {
                fftMeanIndex = (fftWeightedSum / fftSum);
                centerFreq = fftMeanIndex * fftResolution;
                // the above may not be optimal.  In theory, the index points to the centre frequency of the bin.
                // at 1333hz has bin widths 41.65Hz, so bin 2 has the range 83,3Hz and 124,95Hz
                // maybe should be centerFreq = (fftMeanIndex + 0.5) * fftResolution;
                // empirical checking says not adding 0.5 works well
            } else {
                centerFreq = state->centerFreq[state->updateAxis];
            }
            centerFreq = constrainf(centerFreq, dynNotchMinHz, dynNotchMaxCtrHz);

            // PT1 style dynamic smoothing moves rapidly towards big peaks and slowly away, up to 5x faster
            float dynamicFactor = constrainf(dataMax / dataMin, 1.0f, 5.0f);
            state->centerFreq[state->updateAxis] = state->centerFreq[state->updateAxis] + smoothFactor * dynamicFactor * (centerFreq - state->centerFreq[state->updateAxis]);

            if(calculateThrottlePercentAbs() > DYN_NOTCH_OSD_MIN_THROTTLE) {
                dynNotchMaxFFT = MAX(dynNotchMaxFFT, state->centerFreq[state->updateAxis]);
            }

            if (state->updateAxis == 0) {
                DEBUG_SET(DEBUG_FFT, 3, lrintf(fftMeanIndex * 100));
                DEBUG_SET(DEBUG_FFT_FREQ, 0, state->centerFreq[state->updateAxis]);
                DEBUG_SET(DEBUG_FFT_FREQ, 1, lrintf(dynamicFactor * 100));
                DEBUG_SET(DEBUG_DYN_LPF, 1, state->centerFreq[state->updateAxis]);
            }
//            if (state->updateAxis == 1) {
//            DEBUG_SET(DEBUG_FFT_FREQ, 1, state->centerFreq[state->updateAxis]);
//            }
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            break;
        }
        case STEP_UPDATE_FILTERS:
        {
            // 7us
            // calculate cutoffFreq and notch Q, update notch filter
            if (dualNotch) {
                biquadFilterUpdate(&notchFilterDyn[state->updateAxis], state->centerFreq[state->updateAxis] * dynNotch1Ctr, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
                biquadFilterUpdate(&notchFilterDyn2[state->updateAxis], state->centerFreq[state->updateAxis] * dynNotch2Ctr, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
            } else {
                biquadFilterUpdate(&notchFilterDyn[state->updateAxis], state->centerFreq[state->updateAxis], gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
            }
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            state->updateAxis = (state->updateAxis + 1) % XYZ_AXIS_COUNT;
            state->updateStep++;
            FALLTHROUGH;
        }
        case STEP_HANNING:
        {
            // 5us
            // apply hanning window to gyro samples and store result in fftData[i] to be used in step 1 and 2 and 3
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


uint16_t getMaxFFT(void) {
    return dynNotchMaxFFT;
}

void resetMaxFFT(void) {
    dynNotchMaxFFT = 0;
}

#endif // USE_GYRO_DATA_ANALYSE
