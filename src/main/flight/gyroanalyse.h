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

#pragma once

#include "arm_math.h"

#include "common/filter.h"


// max for F3 targets
#ifdef STM32F7
#define FFT_WINDOW_SIZE 64
#else
#define FFT_WINDOW_SIZE 32
#endif

#define DYN_NOTCH_COUNT 4

STATIC_ASSERT(FFT_WINDOW_SIZE <= (uint8_t) -1, window_size_greater_than_underlying_type);

void gyroDataAnalyseStateInit();
void gyroDataAnalysePush(int axis, float sample);
void gyroDataAnalyse();
float gyroDataAnalyseApply(int axis, float values);
uint16_t getMaxFFT(void);
void resetMaxFFT(void);
