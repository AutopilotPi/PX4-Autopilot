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
#include <drivers/device/spi.h>
#include "spi_pwm.h"
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/cdev/CDev.hpp>

namespace fpga_spi_pwm
{

/*
	why do this wrapper?
	the I2CSPIDriver and OutputModuleInterface are all the children of PX4::ScheduledWorkItem,
	and I2CSPIDriver has override the Run() method(and set it to final),
	while OutputModuleInterface didn't override it.

	So if we directly inherit from OutputModuleInterface, we will need to override Run(), then conficts happend
*/

class OutputModuleInterfaceWrapper:public OutputModuleInterface{
	public:
	OutputModuleInterfaceWrapper():	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::I2C0){};
	// the config parameter of OutputModuleInterface is useless here, because we use I2CSPIDriver to schedule{}

	void Run(){
		RunImpl();
	};

	virtual void RunImpl();
};

class FPGA_SPI_PWM :public device::SPI,public I2CSPIDriver<FPGA_SPI_PWM>,public OutputModuleInterfaceWrapper
{
public:
	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);

	FPGA_SPI_PWM(const I2CSPIDriverConfig &config,int predivide);

	const px4::wq_config_t &wq_config;

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			unsigned num_outputs, unsigned num_control_groups_updated) override;

	int ioctl(cdev::file_t *filep, int cmd, unsigned long arg) override;

	enum class STATE : uint8_t {
		INIT,
		RUNNING,
		PRINTSTATUS,
	};
	STATE _state{STATE::INIT};

	static void print_usage();

	void print_status() override;

	int init() override;

	void RunImpl() override;

	int setFreq(uint8_t sub_pwm_n,float freq);
	void setEXTPWM(uint16_t * bitmasks);


	~FPGA_SPI_PWM() override = default;

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

	/*
	 * set clock divider
	 */
	void setPeriod(uint8_t sub_pwm_n,uint16_t value);

	int writeReg(uint8_t reg_addr,uint16_t val){
		uint8_t buf[3]={(uint8_t)(reg_addr << 1 | 1), (uint8_t)(val>>8) ,(uint8_t)(val&0xff)};
		return transfer(buf,nullptr,3);
	}
	uint16_t readReg(uint8_t reg_addr){
		uint8_t buf[3]={(uint8_t)(reg_addr << 1 | 0)};
		transfer(buf,buf,3);
		return buf[1] <<8 | buf[2];
	}

private:
	uint16_t out[FPGA_PWM_OUTPUT_MAX_CHANNELS]={0};
	MixingOutput _mixing_output{"FPGA_PWM", FPGA_PWM_OUTPUT_MAX_CHANNELS, *this, MixingOutput::SchedulingPolicy::Disabled, true};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	int _class_instance{-1};  // used to save index of /dev/pwm_outputX

	// parameter related :
	int _predivide = 0;
	float _freq = PWM_DEFAULT_FREQUENCY;
	float _freq_ext = PWM_DEFAULT_FREQUENCY;
	uint16_t _period=FPGA_DEFAULT_PERIOD;
	uint16_t _period_ext= FPGA_DEFAULT_PERIOD;
	uint16_t _aux_channel_mask=0;   // we only have 8 channels now, so we can directly use a 16bit variable  here
	void updatePWMParams();
	void updatePWMParamTrim();

};

}
