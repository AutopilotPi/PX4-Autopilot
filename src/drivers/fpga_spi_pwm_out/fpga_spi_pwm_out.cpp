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
#include <drivers/drv_mixer.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

using namespace fpga_spi_pwm;

FPGA_SPI_PWM::FPGA_SPI_PWM(const I2CSPIDriverConfig &config,int predivide):
	SPI(config),I2CSPIDriver(config),wq_config(config.wq_config),_predivide(predivide){}

int FPGA_SPI_PWM::init(){
	SPI::init();
	_class_instance=register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);
	// register /dev/pwm_outputX, then controller could operate it by ioctl

	I2CSPIDriver::Deinit();
	OutputModuleInterface::ChangeWorkQueue(wq_config);
	OutputModuleInterface::ScheduleNow();

	return 0;
}


void FPGA_SPI_PWM::setEXTPWM(uint16_t * bitmasks){
	uint16_t config=0;
	uint16_t channel_map=0;
	config=readReg(CONFIG_REG);
	for(int i=0;i<FPAG_PWM_SUBPWM_NUM-1;++i){
		uint16_t bitmask=bitmasks[i];

		if(bitmask!=0){
			config |= 1<<(14-i);
		}
		for(int j=0;j<FPGA_PWM_OUTPUT_MAX_CHANNELS;++j){
			if(bitmask & (1<<j) ){
				channel_map |= (i+1) << j*2;    // i+1 is sub_pwm_n, j is chn
			}
		}
		writeReg(PWM_CHANNEL_MAP0_REG + i, channel_map);
	}
	writeReg(CONFIG_REG,config);
}


int FPGA_SPI_PWM::setFreq(uint8_t sub_pwm_n,float freq)
{
	uint32_t period_temp = floorl((float)FPGA_FREQ/(_predivide+1)/ freq);

	if (period_temp > 0xFFFF) {
		PX4_DEBUG("frequency is too low");
		return -EINVAL;
	}

	setPeriod(sub_pwm_n,(uint16_t)period_temp);
	if(sub_pwm_n){
		_freq_ext=freq;
	}else{
		_freq=freq;
	}
	return PX4_OK;

}

void FPGA_SPI_PWM::disableAllOutput()
{
	for(int i=0;i<FPGA_PWM_OUTPUT_MAX_CHANNELS;++i){
		writeReg(CCR0_REG+i,0);
	}
}

void FPGA_SPI_PWM::setPeriod(uint8_t sub_pwm_n,uint16_t value)
{
	int ret=0;
	uint16_t * period_ptr=sub_pwm_n?&_period_ext:&_period;
	uint8_t reg= sub_pwm_n?SUBPWM1_PERIOD_REG:SUBPWM0_PERIOD_REG;

	ret=writeReg(reg,value);
	if (OK != ret) {
		PX4_ERR("spi::transfer returned %d", ret);
		return;
	}
	*period_ptr=value;
}

bool FPGA_SPI_PWM::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (num_outputs > FPGA_PWM_OUTPUT_MAX_CHANNELS) {
		num_outputs = FPGA_PWM_OUTPUT_MAX_CHANNELS;
		PX4_DEBUG("FPGA_SPI_PWM can only drive up to 8 channels");
	}
	//memcpy(out, outputs, sizeof(uint16_t) * num_outputs);
	for (unsigned i = 0; i < num_outputs; ++i) {
		bool is_aux_chn = _aux_channel_mask & ( 1 << i );
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
		writeReg(CCR0_REG+i,out[i]);
	}

	return true;

}

int FPGA_SPI_PWM::ioctl(cdev::file_t *filep, int cmd, unsigned long arg){
	int ret = OK;

	lock();

	switch (cmd) {
	case MIXERIOCRESET:
		_mixing_output.resetMixer();
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixer(buf, buflen);
			break;
		}

	case PWM_SERVO_GET_COUNT:
		*(unsigned *)arg = FPGA_PWM_OUTPUT_MAX_CHANNELS;
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
	case PWM_SERVO_CLEAR_ARM_OK:
	case PWM_SERVO_SET_FORCE_SAFETY_ON:
	case PWM_SERVO_ARM:
	case PWM_SERVO_DISARM:
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filep, cmd, arg);
	}

	return ret;
}


static int32_t getParam(const char * param_name){
	param_t param_h = param_find(param_name);
	int32_t temp=0xdeadbeef;
	if (param_h != PARAM_INVALID) {
		param_get(param_h,&temp);
		PX4_DEBUG("success get:%s",param_name);

	} else {
		PX4_DEBUG("PARAM_INVALID: %s", param_name);
	}
	return temp;
}

static int32_t getParamWithSprintf(const char * format, int num){

	char buf[16]={0};
	sprintf(buf,format,num);
	return getParam(buf);
}

void FPGA_SPI_PWM::updatePWMParams()
{
	if (_mixing_output.useDynamicMixing()) {
		return;
	}

	int32_t aux_out_temp=0;


	aux_out_temp=getParam("PWM_AUX_OUT");

	for(; aux_out_temp!=0 ;aux_out_temp/=10){    // aux_out_temp may be 1234, which means 0xF in bitmask
		int32_t temp = aux_out_temp % 10;
		_aux_channel_mask |= 1<<(temp-1);
	}

	// notice: use the val of PWM_MAIN as the default value, which is different from PX4 document.

	for(int i=0;i<FPGA_PWM_OUTPUT_MAX_CHANNELS;++i){
		uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();
		uint8_t param_index=i+1;  // param_index start from 1
		reverse_pwm_mask&= ~(1<<i);
		if( _aux_channel_mask  & (1<<i) ){
			_mixing_output.maxValue(i)=getParamWithSprintf("PWM_AUX_MAX%d",param_index);
			_mixing_output.minValue(i)=getParamWithSprintf("PWM_AUX_MIN%d",param_index);
			_mixing_output.failsafeValue(i)=getParamWithSprintf("PWM_AUX_FAIL%d",param_index);
			_mixing_output.disarmedValue(i)=getParamWithSprintf("PWM_AUX_DIS%d",param_index);
			reverse_pwm_mask|= (getParamWithSprintf("PWM_AUX_REV%d",param_index)?1:0)<<i;

		}else{
			_mixing_output.maxValue(i)=getParamWithSprintf("PWM_MAIN_MAX%d",param_index);
			_mixing_output.minValue(i)=getParamWithSprintf("PWM_MAIN_MIN%d",param_index);
			_mixing_output.failsafeValue(i)=getParamWithSprintf("PWM_MAIN_FAIL%d",param_index);
			_mixing_output.disarmedValue(i)=getParamWithSprintf("PWM_MAIN_DIS%d",param_index);
			reverse_pwm_mask|= (getParamWithSprintf("PWM_MAIN_REV%d",param_index)?1:0)<<i;
		}
	}

	_freq=getParam("PWM_MAIN_RATE");
	_freq_ext=getParam("PWM_AUX_RATE");

	if (_mixing_output.mixers()) { // only update trims if mixer loaded
		updatePWMParamTrim();
	}
}

void FPGA_SPI_PWM::updatePWMParamTrim()
{
	const char *pname_format_pwm_ch_trim[2] = {"PWM_MAIN_TRIM%d", "PWM_AUX_TRIM%d"};

	int16_t trim_values[FPGA_PWM_OUTPUT_MAX_CHANNELS] = {};

	for (int i = 0; i < FPGA_PWM_OUTPUT_MAX_CHANNELS; i++) {
		char pname[16];
		uint8_t param_group, param_index;
		if (i <= 7) {	// Main channel
			param_group = 0;
			param_index = i + 1;
		} else {	// AUX
			param_group = 1;
			param_index = i - 8 + 1;
		}

		sprintf(pname, pname_format_pwm_ch_trim[param_group], param_index);
		param_t param_h = param_find(pname);
		int32_t val;

		if (param_h != PARAM_INVALID) {
			param_get(param_h, &val);
			trim_values[i] = (int16_t)val;

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}
	}
	unsigned n_out = _mixing_output.mixers()->set_trims(trim_values, FPGA_PWM_OUTPUT_MAX_CHANNELS);
	PX4_DEBUG("set %d trims", n_out);
}


