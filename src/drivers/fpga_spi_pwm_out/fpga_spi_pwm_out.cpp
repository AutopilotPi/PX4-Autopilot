/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_log.h>
#include <cmath>
#include "fpga_spi_pwm_out.h"
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

using namespace fpga_spi_pwm;

FPGA_SPI_PWM::FPGA_SPI_PWM(int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency):
	SPI(DRV_PWM_DEVTYPE_FPGA, "fpga_spi_pwm", bus, device, mode, frequency) {}


void FPGA_SPI_PWM::setEXTPWM(uint16_t *channel_maps)
{
	uint16_t config = 0;
	uint8_t channel_index = -1;
	uint16_t channel_map = 0;
	config = readReg(CONFIG_REG);

	for (int i = 0; i < FPGA_PWM_OUTPUT_MAX_CHANNELS; ++i) {
		if (i % 8 == 0) {
			channel_map = channel_maps[++channel_index];
			writeReg(PWM_CHANNEL_MAP0_REG + i / 8, channel_map);
		}

		_channel_mode_map[i] = channel_map & 0x3;

		if (_channel_mode_map[i] == 0x1) {
			config |= 1 << (14 - i / 8);
		}

		channel_map >>= 2;
	}

	writeReg(CONFIG_REG, config);
}


int FPGA_SPI_PWM::setFreq(uint8_t sub_pwm_n, float freq)
{
	uint32_t period_temp = floorl((float)FPGA_FREQ / _predivide / freq);

	if (period_temp > 0xFFFF) {
		PX4_DEBUG("frequency is too low");
		return -EINVAL;
	}

	setPeriod(sub_pwm_n, (uint16_t)period_temp);
	_freqs[sub_pwm_n] = freq;
	return PX4_OK;

}

void FPGA_SPI_PWM::disableAllOutput()
{
	for (int i = 0; i < FPGA_PWM_OUTPUT_MAX_CHANNELS; ++i) {
		writeReg(CCR0_REG + i, 0);
	}
}

void FPGA_SPI_PWM::setPeriod(uint8_t sub_pwm_n, uint16_t value)
{
	int ret = 0;
	uint8_t reg = _period_regs[sub_pwm_n];

	uint16_t config = readReg(CONFIG_REG) & ~0x1F;
	config |= (_predivide - 1) & 0x1F ;
	ret = writeReg(CONFIG_REG, config);

	ret |= writeReg(reg, value);

	if (OK != ret) {
		PX4_ERR("spi::transfer returned %d", ret);
		return;
	}

	_periods[sub_pwm_n] = value;
}





