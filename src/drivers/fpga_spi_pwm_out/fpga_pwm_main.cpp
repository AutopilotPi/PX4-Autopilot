/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

#include "fpga_spi_pwm_out.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/WorkItemSingleShot.hpp>

#include <functional>
#include <sys/ioctl.h>

using namespace fpga_spi_pwm;
void FPGA_SPI_PWM::print_usage()
{
	PRINT_MODULE_USAGE_NAME("fpga_pwm_out", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("pwm");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 2, 0, 32, "predivide of pwm clk. 0 = no predivide", true);
	PRINT_MODULE_USAGE_PARAM_INT('F', 400, 50, 1000, "schedule freq", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int schd_rate_limit=400;
int predivide = 2; // clk_out = clk/(_predivide + 1);     TODO:make _predivide could change when starting.

I2CSPIDriverBase * FPGA_SPI_PWM::instantiate(const I2CSPIDriverConfig &config, int runtime_instance){

	// implement this func only to make module_start happy
	FPGA_SPI_PWM *instance = new FPGA_SPI_PWM(config,predivide);

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init()) {
		delete instance;
		return nullptr;
	}
	return instance;
}

void FPGA_SPI_PWM::RunImpl(){
	if (should_exit()) {
		OutputModuleInterface::ScheduleClear();
		_mixing_output.unregister();
		unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);
		disableAllOutput();
		exit_and_cleanup();
		return;
	}
	switch (_state) {
	case STATE::INIT:
		updatePWMParams();
		writeReg(CONFIG_REG,_predivide); // set predivider
		if (setFreq(0,_freq) != PX4_OK) {
			PX4_ERR("failed to set main pwm frequency to %.2f", (double)_freq);
		}

		if (setFreq(1,_freq_ext) != PX4_OK) {
			PX4_ERR("failed to set aux pwm frequency to %.2f", (double)_freq_ext);
		}
		assert(FPAG_I2C_PWM_SUBPWM_NUM==2);
		setEXTPWM(&_aux_channel_mask);

		_state = STATE::RUNNING;
		OutputModuleInterface::ScheduleOnInterval(1000000 / schd_rate_limit, 1000000 / schd_rate_limit);
		break;

	case STATE::RUNNING:
		_mixing_output.update();
		if (_parameter_update_sub.updated()) {
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);
			_state = STATE::INIT; // goback to INIT
		}
		_mixing_output.updateSubscriptions(false);
		break;
	case STATE::PRINTSTATUS:
		PX4_INFO("predivide:%d",_predivide);
		PX4_INFO("support freq:%d ~ %d",FPGA_FREQ/(_predivide+1)/65535,FPGA_FREQ/(_predivide+1)/100);
		PX4_INFO("main period:%d  main_freq:%d",readReg(SUBPWM0_PERIOD_REG),(int)_freq);
		PX4_INFO("aux period:%d  aux_freq:%d",readReg(SUBPWM1_PERIOD_REG),(int)_freq_ext);
		PX4_INFO("aux channel mask:%#x",_aux_channel_mask);
		PX4_INFO("config reg:%#x",readReg(CONFIG_REG));
		for(int i=0; i<FPGA_PWM_OUTPUT_MAX_CHANNELS;++i){
			PX4_INFO("CCR%d:%d",i,readReg(CCR0_REG+i));
		}
		_mixing_output.printStatus();
		_state = STATE::RUNNING;
		break;
	}
}

void FPGA_SPI_PWM::print_status(){
	I2CSPIDriver::print_status();
	_state = STATE::PRINTSTATUS;
}


extern "C" int fpga_pwm_out_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = FPGA_SPI_PWM;
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = 10000000;

	//const char *verb = cli.parseDefaultArguments(argc, argv);
	while ((ch = cli.getOpt(argc, argv, "p:F:")) != EOF) {
		switch (ch) {
		case 'p':
			predivide = atoi(cli.optArg());
			break;
		case 'F':
			schd_rate_limit = atoi(cli.optArg());
			break;
		}

	}

	const char *verb = cli.optArg();
	if (!verb) {
		FPGA_SPI_PWM::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_PWM_DEVTYPE_FPGA);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}
	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	FPGA_SPI_PWM::print_usage();
	return -1;
}
