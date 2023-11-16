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

int FPGA_I2C_PWM::writeReg(uint8_t reg_addr,uint16_t val){
	uint8_t buf[3]={reg_addr,(uint8_t)(val>>8),(uint8_t)(val&0xff)};
	return transfer(buf,3,nullptr,0);
}

void FPGA_I2C_PWM::setEXTPWM(uint16_t * bitmasks){
	uint16_t config=0;
	uint8_t buf[3] = {0x80};
	transfer(buf, 1,buf+1,2);  // read config
	config=buf[1]<<8 | buf[2];
	PX4_WARN("old config:%d ",config);

	PX4_WARN("bit mask is :0x%x",bitmasks[0]);
	for(int i=0;i<FPAG_I2C_PWM_SUBPWM_NUM-1;++i){
		uint16_t bitmask=bitmasks[i];

		if(bitmask!=0){
			config |= 1<<(14-i);
		}
		for(int j=0;j<FPGA_PWM_OUTPUT_MAX_CHANNELS;++j){
			if(bitmask & (1<<j) ){
				_channel_map |= (i+1) << j*2;    // i+1 is sub_pwm_n, j is chn
			}
		}

	}
	writeReg(0x20,_channel_map);
	writeReg(0x80,config);
}


int FPGA_I2C_PWM::Stop()
{
	disableAllOutput();
	return PX4_OK;
}

void FPGA_I2C_PWM::status(){
	PX4_INFO("main period:%d  %d  main_freq:%d",getPeriod(),_period,(int)_freq);
	for(int i=0; i<FPGA_PWM_OUTPUT_MAX_CHANNELS;++i){
		PX4_INFO("CCR%d:%d",i,getCCR(i));
	}


}

int FPGA_I2C_PWM::updatePWM(const uint16_t *outputs, unsigned num_outputs)
{
	if (num_outputs > FPGA_PWM_OUTPUT_MAX_CHANNELS) {
		num_outputs = FPGA_PWM_OUTPUT_MAX_CHANNELS;
		PX4_DEBUG("FPGA_I2C_PWM can only drive up to 8 channels");
	}

	uint16_t out[FPGA_PWM_OUTPUT_MAX_CHANNELS]={0};
	//memcpy(out, outputs, sizeof(uint16_t) * num_outputs);

	for (unsigned i = 0; i < num_outputs; ++i) {
		bool is_aux_chn = _channel_map & ( 1 << i*2 );
		float temp_freq= is_aux_chn?_freq_ext:_freq;
		uint16_t temp_period = is_aux_chn?_period_ext:_period;

		if(temp_freq>500){
			out[i] = outputs[i]*temp_period/10000;
			/*
				if freq > 500, then unit of output(us) make no sense,
				so in this situation, we treat it as Duty Mode.
				set PWM_XXX_MAX${i} to 10000, and PWM_XXX_MIN${i} to 0
			*/
		}else{
			out[i] = outputs[i]*temp_freq*temp_period/1000/1000;
		}
		setPWM(i, out[i]);
	}

	return 0;
}

int FPGA_I2C_PWM::setFreq(uint8_t sub_pwm_n,float freq)
{
	uint32_t period_temp = floorl((float)FPGA_FREQ/2/ freq);

	if (period_temp > 0xFFFF) {
		PX4_DEBUG("frequency is too low");
		return -EINVAL;
	}

	setPeriod(sub_pwm_n,period_temp);
	if(sub_pwm_n){
		_freq_ext=freq;
	}else{
		_freq=freq;
	}
	return PX4_OK;

}

void FPGA_I2C_PWM::initRegs()
{
	writeReg(0x80,0x01); // set predivider = 2
}

int FPGA_I2C_PWM::probe()
{
	return I2C::probe();
}

void FPGA_I2C_PWM::setPWM(uint8_t channel, const uint16_t &value)
{
	int ret;

	ret = writeReg(FPGA_I2C_REG_CH0_CCR+channel,value);
	if (OK != ret) {
		PX4_DEBUG("setPWM: i2c::transfer returned %d", ret);
	}
}

void FPGA_I2C_PWM::disableAllOutput()
{
	uint16_t temp=0;
	for(int i=0;i<FPGA_PWM_OUTPUT_MAX_CHANNELS;++i){
		setPWM(i,temp);
	}
}

void FPGA_I2C_PWM::setPeriod(uint8_t sub_pwm_n,uint16_t value)
{
	int ret=0;
	uint16_t * period_ptr=sub_pwm_n?&_period_ext:&_period;
	uint8_t reg= sub_pwm_n?0x40:0x00;

	ret=writeReg(reg,value);
	if (OK != ret) {
		PX4_ERR("i2c::transfer returned %d", ret);
		return;
	}
	*period_ptr=value;
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
