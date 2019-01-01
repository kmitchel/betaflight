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
#ifdef TM32F7
#define DYN_NOTCH_CALC_TICKS      (XYZ_AXIS_COUNT)
#else
#define DYN_NOTCH_CALC_TICKS      (XYZ_AXIS_COUNT * 4)
#endif
#define DYN_NOTCH_OSD_MIN_THROTTLE 20


static uint16_t FAST_RAM_ZERO_INIT fftSamplingRateHz;
static float FAST_RAM_ZERO_INIT fftResolution;
static uint8_t FAST_RAM_ZERO_INIT fftStartBin;
static uint16_t FAST_RAM_ZERO_INIT dynNotchMaxCtrHz;
static uint8_t dynNotchRange;
static float FAST_RAM_ZERO_INIT dynNotchQ;
static float FAST_RAM_ZERO_INIT dynNotch1Ctr;
static float FAST_RAM_ZERO_INIT dynNotch2Ctr;
static uint16_t FAST_RAM_ZERO_INIT dynNotchMinHz;
static bool FAST_RAM dualNotch = true;
static uint16_t FAST_RAM_ZERO_INIT dynNotchMaxFFT;

static uint8_t FAST_RAM_ZERO_INIT sampleCount;
static uint8_t FAST_RAM_ZERO_INIT maxSampleCount;
static float FAST_RAM_ZERO_INIT maxSampleCountRcp;
static float FAST_RAM_ZERO_INIT oversampledGyroAccumulator[XYZ_AXIS_COUNT];

// downsampled gyro data circular buffer for frequency analysis
static uint8_t FAST_RAM_ZERO_INIT circularBufferIdx;
static float FAST_RAM_ZERO_INIT downsampledGyroData[XYZ_AXIS_COUNT][FFT_WINDOW_SIZE];

// update state machine step information
static uint8_t FAST_RAM_ZERO_INIT updateTicks;
static uint8_t FAST_RAM_ZERO_INIT updateStep;
static uint8_t FAST_RAM_ZERO_INIT updateAxis;

static arm_rfft_fast_instance_f32 FAST_RAM_ZERO_INIT fftInstance;
static float FAST_RAM_ZERO_INIT fftData[FFT_WINDOW_SIZE];
static float FAST_RAM_ZERO_INIT rfftData[FFT_WINDOW_SIZE];

typedef struct notchGroup_s {
    pt1Filter_t centerFreqFilter[3];
    float centerFreq[3];
    biquadFilter_t gyroNotch[DYN_NOTCH_COUNT];
} notchGroup_t;

static FAST_RAM notchGroup_t notchGroup[XYZ_AXIS_COUNT];

static int FAST_RAM change = 0;

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
 
//    if (gyroConfig()->dyn_notch_width_percent == 0) {
        dualNotch = false;
//    }

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

    fftStartBin = dynNotchMinHz / rintf(fftResolution);

    dynNotchMaxCtrHz = fftSamplingRateHz * 0.48f; //Stay below Nyquist

    for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
        hanningWindow[i] = (0.5f - 0.5f * cos_approx(2 * M_PIf * i / (FFT_WINDOW_SIZE - 1)));
//        hanningWindow[i] = i > FFT_WINDOW_SIZE / 2 - 1 ? 2.0f - 2.0f * i / (FFT_WINDOW_SIZE - 1) : 2.0f * i / (FFT_WINDOW_SIZE - 1);
    }
}

void gyroDataAnalyseStateInit()
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    // *** can this next line be removed ??? ***
    gyroDataAnalyseInit();

    const uint16_t samplingFrequency = 1000000 / gyro.targetLooptime;
    maxSampleCount = samplingFrequency / fftSamplingRateHz;
    maxSampleCountRcp = 1.f / maxSampleCount;

    arm_rfft_fast_init_f32(&fftInstance, FFT_WINDOW_SIZE);

//    recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
//    at 4khz gyro loop rate this means 4khz / 4 / 3 = 333Hz => update every 3ms
//    for gyro rate > 16kHz, we have update frequency of 1kHz => 1ms
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // any init value
        for (int i = 0; i < 3; i++) {
            notchGroup[axis].centerFreq[i] = dynNotchMaxCtrHz;
            pt1FilterInit(&notchGroup[axis].centerFreqFilter[i], pt1FilterGain(gyroConfig()->dyn_notch_lpf_hz, 1.0f / fftSamplingRateHz));
            notchGroup[axis].centerFreqFilter[i].state = dynNotchMaxCtrHz;
        }
        for (int i = 0; i < DYN_NOTCH_COUNT; i++) {
            biquadFilterInit(&notchGroup[axis].gyroNotch[i], dynNotchMaxCtrHz, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);

        }
    }
}

void gyroDataAnalysePush(const int axis, const float sample)
{
    oversampledGyroAccumulator[axis] = sample;
}

static void gyroDataAnalyseUpdate();

/*
 * Collect gyro data, to be analysed in gyroDataAnalyseUpdate function
 */
void FAST_CODE_NOINLINE gyroDataAnalyse()
{
    // samples should have been pushed by `gyroDataAnalysePush`
    // if gyro sampling is > 1kHz, accumulate multiple samples
    sampleCount++;

    // this runs at 1kHz
    if (sampleCount == maxSampleCount) {
        sampleCount = 0;

        // calculate mean value of accumulated samples
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            float sample = oversampledGyroAccumulator[axis] * maxSampleCountRcp;
            downsampledGyroData[axis][circularBufferIdx] = sample;
            if (axis == 0) {
                DEBUG_SET(DEBUG_FFT, 2, lrintf(sample));
            }

            oversampledGyroAccumulator[axis] = 0;
        }

        circularBufferIdx = (circularBufferIdx + 1) % FFT_WINDOW_SIZE;

        // We need DYN_NOTCH_CALC_TICKS tick to update all axis with newly sampled value
        updateTicks = DYN_NOTCH_CALC_TICKS;
    }

    // calculate FFT and update filters
    if (updateTicks > 0) {
        gyroDataAnalyseUpdate();
        --updateTicks;
    }
}

void stage_rfft_f32(arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut);
void arm_cfft_radix8by2_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_cfft_radix8by4_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_radix8_butterfly_f32(float32_t *pSrc, uint16_t fftLen, const float32_t *pCoef, uint16_t twidCoefModifier);
void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen, const uint16_t *pBitRevTable);

// float calculateWeight(uint8_t k) {
//     //Credit:  http://sam-koblenski.blogspot.com/2015/11/everyday-dsp-for-programmers-spectral.html
//     return k + (fftData[k + 1] - fftData[k - 1]) / (4 * fftData[k] - 2 * fftData[k - 1] - 2 * fftData[k + 1]);
// }

float calculateWeight(uint8_t k) {
    if (k == FFT_BIN_COUNT - 1) {
        return k;
    } else {
        float dataSquared1 = fftData[k - 1] * fftData[k - 1];
        float dataSquared2 = fftData[k] * fftData[k];
        float dataSquared3 = fftData[k + 1] * fftData[k + 1];
        float sumSquared = dataSquared1 + dataSquared2 + dataSquared3;
        float sumSquaredWeighted = dataSquared1 * (k - 1);
        sumSquaredWeighted += dataSquared2 * (k);
        sumSquaredWeighted += dataSquared3 * (k + 1);
        return sumSquaredWeighted / sumSquared;
    }
}

/*
 * Analyse last gyro data from the last FFT_WINDOW_SIZE milliseconds
 */
static FAST_CODE_NOINLINE void gyroDataAnalyseUpdate()
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

    arm_cfft_instance_f32 *Sint = &(fftInstance.Sint);

    uint32_t startTime = 0;
    if (debugMode == (DEBUG_FFT_TIME)) {
        startTime = micros();
    }

    DEBUG_SET(DEBUG_FFT_TIME, 0, updateStep);
    switch (updateStep) {
        case STEP_ARM_CFFT_F32:
        {
            switch (FFT_BIN_COUNT) {
            case 16:
                // 16us
                arm_cfft_radix8by2_f32(Sint, fftData);
                break;
            case 32:
                // 35us
                arm_cfft_radix8by4_f32(Sint, fftData);
                break;
            case 64:
                // 70us
                arm_radix8_butterfly_f32(fftData, FFT_BIN_COUNT, Sint->pTwiddle, 1);
                break;
            }
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
#ifndef TM32F7
            break;
#else
            updateStep++;
            FALLTHROUGH;
#endif
        }
        case STEP_BITREVERSAL:
        {
            // 6us
            arm_bitreversal_32((uint32_t*) fftData, Sint->bitRevLength, Sint->pBitRevTable);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            updateStep++;
            FALLTHROUGH;
        }
        case STEP_STAGE_RFFT_F32:
        {
            // 14us
            // this does not work in place => fftData AND rfftData needed
            stage_rfft_f32(&fftInstance, fftData, rfftData);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
#ifndef TM32F7
            break;
#else
            updateStep++;
            FALLTHROUGH;
#endif
        }
        case STEP_ARM_CMPLX_MAG_F32:
        {
            // 8us
            arm_cmplx_mag_f32(rfftData, fftData, FFT_BIN_COUNT);
            DEBUG_SET(DEBUG_FFT_TIME, 2, micros() - startTime);
            updateStep++;
            FALLTHROUGH;
        }
        case STEP_CALC_FREQUENCIES:
        {
            float threshold = 0;
            int count = 0; 

            for (int i = FFT_BIN_COUNT - 2; i >= fftStartBin; i--) {
                threshold += fftData[i] * fftData[i];
                count++;
            }

            threshold /= count;
            threshold = sqrtf(threshold);
            threshold = MAX(threshold, 2);

            //Default to the last bin.
            int k[3] = {FFT_BIN_COUNT - 1, FFT_BIN_COUNT - 1, FFT_BIN_COUNT - 1};

            change = 0;

            for (int i = FFT_BIN_COUNT - 2; i > fftStartBin; i--) {
                //Compare bin against it's neighbors to find peak.  Rotate peaks high to low.
                if (fftData[i] > fftData[i - 1] && fftData[i] > fftData[i + 1] && fftData[i] > threshold) {
                    k[2] = k[1];
                    k[1] = k[0];
                    k[0] = i;
                    change++;
                }
            }

            //Sort bins.  Large bin low.
            if (fftData[k[0]] < fftData[k[2]] && change > 2) {
                int i = k[0];
                k[0] = k[2];
                k[2] = i;
            }

            if (fftData[k[0]] < fftData[k[1]] && change > 1) {
                int i = k[0];
                k[0] = k[1];
                k[1] = i;
            }

            if (fftData[k[1]] < fftData[k[2]] && change > 2) {
                int i = k[1];
                k[1] = k[2];
                k[2] = i;
            }

            uint16_t freq[3] = {dynNotchMaxCtrHz, dynNotchMaxCtrHz, dynNotchMaxCtrHz};

            if (change){
                //calculate weighted bin.
                freq[0] = calculateWeight(k[0]) * fftResolution;
                freq[1] = calculateWeight(k[1]) * fftResolution;
                freq[2] = calculateWeight(k[2]) * fftResolution;

                //Store peak frequency for OSD.
                if (calculateThrottlePercentAbs() > DYN_NOTCH_OSD_MIN_THROTTLE) {
                    uint16_t peakFreq = freq[0];
                    dynNotchMaxFFT = MAX(dynNotchMaxFFT, peakFreq);
                }
            }

            //Constrain and smooth center frequencies.
            for (int i = 0; i < change; i++) {
                freq[i] = constrain(freq[i], dynNotchMinHz, dynNotchMaxCtrHz);
                notchGroup[updateAxis].centerFreq[i] = pt1FilterApply(&notchGroup[updateAxis].centerFreqFilter[i], freq[i]);
            }

            if (updateAxis == 0) {
                DEBUG_SET(DEBUG_FFT, 3, lrintf(calculateWeight(k[0]) * 100));
                DEBUG_SET(DEBUG_FFT_FREQ, 0, notchGroup[updateAxis].centerFreq[0]);
                DEBUG_SET(DEBUG_DYN_LPF, 1, notchGroup[updateAxis].centerFreq[0]);
//                DEBUG_SET(DEBUG_DYN_LPF, 2, centerFreq[0]);
                DEBUG_SET(DEBUG_DYN_LPF, 2, notchGroup[updateAxis].centerFreq[1]);

            }
            if (updateAxis == 1) {
                DEBUG_SET(DEBUG_FFT_FREQ, 1, notchGroup[updateAxis].centerFreq[0]);
            }
            // Debug FFT_Freq carries raw gyro, gyro after first filter set, FFT centre for roll and for pitch
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
#ifndef TM32F7
            break;
#else
            updateStep++;
            FALLTHROUGH;
#endif
        }
        case STEP_UPDATE_FILTERS:
        {
            // 7us
            //Update notch filters.  When width != 0, cascade.  Otherwise one notch for each frequency.
            if (dualNotch && change > 0) {
                biquadFilterUpdate(&notchGroup[updateAxis].gyroNotch[0], notchGroup[updateAxis].centerFreq[0] * dynNotch1Ctr, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
                biquadFilterUpdate(&notchGroup[updateAxis].gyroNotch[1], notchGroup[updateAxis].centerFreq[0] * dynNotch2Ctr, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
                biquadFilterUpdate(&notchGroup[updateAxis].gyroNotch[2], notchGroup[updateAxis].centerFreq[1] * dynNotch1Ctr, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
                biquadFilterUpdate(&notchGroup[updateAxis].gyroNotch[3], notchGroup[updateAxis].centerFreq[1] * dynNotch2Ctr, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
            } else {
                for (int i = 0; i < change; i++) {
                    biquadFilterUpdate(&notchGroup[updateAxis].gyroNotch[i], notchGroup[updateAxis].centerFreq[i], gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
                }
            }
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            updateAxis = (updateAxis + 1) % XYZ_AXIS_COUNT;
            updateStep++;
            FALLTHROUGH;
        }
        case STEP_HANNING:
        {
            // 5us
            // apply hanning window to gyro samples and store result in fftData
            // hanning starts and ends with 0, could be skipped for minor speed improvement
            const uint8_t ringBufIdx = FFT_WINDOW_SIZE - circularBufferIdx;
            arm_mult_f32(&downsampledGyroData[updateAxis][circularBufferIdx], &hanningWindow[0], &fftData[0], ringBufIdx);
            if (circularBufferIdx > 0) {
                arm_mult_f32(&downsampledGyroData[updateAxis][0], &hanningWindow[ringBufIdx], &fftData[ringBufIdx], circularBufferIdx);
            }

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
        }
    }
    updateStep = (updateStep + 1) % STEP_COUNT;

}

//Apply notch filters to gyro data.
float gyroDataAnalyseApply(int axis, float values) {
    if (dualNotch){
        for (int i = 0; i < DYN_NOTCH_COUNT; i++) {
            values = biquadFilterApplyDF1(&notchGroup[axis].gyroNotch[i], values);
        }
    } else {
        for (int i = 0; i < 3; i++) {
            values = biquadFilterApplyDF1(&notchGroup[axis].gyroNotch[i], values);
        }
    }
    return values;
}

uint16_t getMaxFFT(void) {
    return dynNotchMaxFFT;
}

void resetMaxFFT(void) {
    dynNotchMaxFFT = 0;
}

#endif // USE_GYRO_DATA_ANALYSE