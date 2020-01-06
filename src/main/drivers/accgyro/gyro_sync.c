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

/*
 * gyro_sync.c
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/gyro_sync.h"


bool gyroSyncCheckUpdate(gyroDev_t *gyro)
{
    bool ret;
    if (gyro->dataReady) {
        ret = true;
        gyro->dataReady= false;
    } else {
        ret = false;
    }
    return ret;
}

uint32_t gyroSetSampleRate(gyroDev_t *gyro)
{
    float gyroSamplePeriod;

    switch (gyro->mpuDetectionResult.sensor) {
        case BMI_160_SPI:
            gyro->gyroRateKHz = GYRO_RATE_3200_Hz;
            gyroSamplePeriod = 312.0f;
            break;
        case ICM_20649_SPI:
            gyro->gyroRateKHz = GYRO_RATE_9_kHz;
            gyroSamplePeriod = 1000000.0f / 9000.0f;
            break;
        default:
            gyro->gyroRateKHz = GYRO_RATE_8_kHz;
            gyroSamplePeriod = 125.0f;
            break;
    }

    // calculate gyro divider and targetLooptime (expected cycleTime)
    gyro->mpuDividerDrops  = 0;
    return (uint32_t)(gyroSamplePeriod);
}
