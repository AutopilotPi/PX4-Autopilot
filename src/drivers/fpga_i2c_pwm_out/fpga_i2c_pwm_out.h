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

#pragma once
#include <cstdint>
#include <drivers/device/i2c.h>
//#include <drivers/drv_pwm_output.h>

namespace fpga_i2c_pwm
{

#define FPGA_I2C_REG_PERIOD 0x00
#define FPGA_I2C_REG_CH0_CCR 0x01
#define FPGA_FREQ  50000000  // default 50Mhz
#define FPGA_DEFAULT_PERIOD	2000
#define PWM_DEFAULT_FREQUENCY FPGA_FREQ/FPGA_DEFAULT_PERIOD

#define FPGA_PWM_OUTPUT_MAX_CHANNELS 4


class FPGA_I2C_PWM : public device::I2C
{
public:
	FPGA_I2C_PWM(int bus, int addr);

	int Stop();
	void status();
	uint16_t getPeriod();
	uint16_t getCCR(uint8_t chn);
	/*
	 * outputs formatted to us.
	 */
	int updatePWM(const uint16_t *outputs, unsigned num_outputs);

	int setFreq(float freq);

	~FPGA_I2C_PWM() override = default;

	void enableOutput(uint8_t channel_mark);

	inline float getFrequency() {return _freq;}

	/*
	 * disable all of the output
	 */
	void disableAllOutput();

	/*
	 * turn on output
	 */
	void triggerRestart();

protected:
	int probe() override;

	float _freq = PWM_DEFAULT_FREQUENCY;
	uint16_t _period=FPGA_DEFAULT_PERIOD;

	/**
	 * set PWM value for a channel[0,15].
	 * value should be range of 0-4095
	 */
	void setPWM(uint8_t channel, const uint16_t &value);

	/*
	 * set clock divider
	 */
	void setPeriod(uint16_t value);

private:


};

}
