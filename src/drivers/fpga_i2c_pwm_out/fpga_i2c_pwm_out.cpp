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
#include "fpga_i2c_pwm_out.h"

using namespace fpga_i2c_pwm;

FPGA_I2C_PWM::FPGA_I2C_PWM(int bus, int addr):
	I2C(DRV_PWM_DEVTYPE_PCA9685, MODULE_NAME, bus, addr, 400000)
{

}

int FPGA_I2C_PWM::Stop()
{
	disableAllOutput();
	return PX4_OK;
}

void FPGA_I2C_PWM::status(){
	PX4_INFO("period:%d  %d  freq:%d",getPeriod(),_period,(int)_freq);
	for(int i=0; i<4;++i){
		PX4_INFO("CCR%d:%d",i,getCCR(i));
	}


}

int FPGA_I2C_PWM::updatePWM(const uint16_t *outputs, unsigned num_outputs)
{
	if (num_outputs > FPGA_PWM_OUTPUT_MAX_CHANNELS) {
		num_outputs = FPGA_PWM_OUTPUT_MAX_CHANNELS;
		PX4_DEBUG("FPGA_I2C_PWM can only drive up to 16 channels");
	}

	uint16_t out[FPGA_PWM_OUTPUT_MAX_CHANNELS];
	memcpy(out, outputs, sizeof(uint16_t) * num_outputs);

	for (unsigned i = 0; i < num_outputs; ++i) {
		out[i] = (uint16_t)roundl((out[i]-1000)*_period /1000.0f); // convert us to 12 bit resolution
		setPWM(i, out[i]);
	}

	return 0;
}

int FPGA_I2C_PWM::setFreq(float freq)
{
	uint32_t period_temp = floorl((float)FPGA_FREQ / freq);

	if (period_temp > 0xFFFF) {
		PX4_DEBUG("frequency is too low");
		return -EINVAL;
	}

	setPeriod(period_temp);
	_freq=freq;

	return PX4_OK;

}

void FPGA_I2C_PWM::enableOutput(uint8_t channel_mark)
{
	;
}

int FPGA_I2C_PWM::probe()
{
	return I2C::probe();
}

void FPGA_I2C_PWM::setPWM(uint8_t channel, const uint16_t &value)
{
	uint8_t buf[3] = {};
	buf[0]=	FPGA_I2C_REG_CH0_CCR+channel;
	buf[1] = value>>8;
	buf[2] = value&0xFF;

	int ret = transfer(buf, 3, nullptr, 0);

	if (OK != ret) {
		PX4_DEBUG("setPWM: i2c::transfer returned %d", ret);
	}
}

void FPGA_I2C_PWM::disableAllOutput()
{
	uint16_t temp=0;
	for(int i=0;i<4;++i){
		setPWM(i,temp);
	}
}

void FPGA_I2C_PWM::setPeriod(uint16_t value)
{
	uint8_t buf[3] = {0x00};
	buf[1]=value>>8;
	buf[2]=value&0xFF;
	int ret = transfer(buf, 3, nullptr, 0);

	if (OK != ret) {
		PX4_ERR("i2c::transfer returned %d", ret);
		return;
	}
	_period=value;
}


uint16_t FPGA_I2C_PWM::getPeriod()
{
	uint8_t buf[3] = {0x00,0x01,0x02};
	int ret = transfer(buf, 1, buf, 2);

	if (OK != ret) {
		PX4_ERR("i2c::transfer returned %d", ret);
		return 0xFFFF;
	}
	return (uint16_t)buf[0]<<8 | buf[1];
}

uint16_t FPGA_I2C_PWM::getCCR(uint8_t chn)
{
	uint8_t buf[2] = {(uint8_t)(chn+FPGA_I2C_REG_CH0_CCR)};
	int ret = transfer(buf, 1, buf, 2);

	if (OK != ret) {
		PX4_ERR("i2c::transfer returned %d", ret);
		return 0xFFFF;
	}
	return (uint16_t)buf[0]<<8 | buf[1];
}
