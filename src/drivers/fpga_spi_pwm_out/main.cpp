/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file fpga_pwm/main.cpp
 * A cross-platform driver and wrapper for fpga_pwm modules.
 * Designed to support all control-groups by binding to correct mixer files
 * @author ncerzzk <huangcmzzk@gmail.com>
 */

#include <px4_log.h>
#include <drivers/device/device.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>

#include "fpga_spi_pwm_out.h"
#include <stdarg.h>

#define FPGA_PWM_DEFAULT_IICBUS  0
#define FPGA_PWM_DEFAULT_ADDRESS (0x20)


using namespace fpga_spi_pwm;
using namespace time_literals;

class FPGA_PWM_Wrapper : public cdev::CDev, public ModuleBase<FPGA_PWM_Wrapper>, public OutputModuleInterface
{
public:

	FPGA_PWM_Wrapper();
	~FPGA_PWM_Wrapper() override ;

	int init() override;

	int ioctl(cdev::file_t *filep, int cmd, unsigned long arg) override;

	void mixerChanged() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);
	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

	FPGA_PWM_Wrapper(const FPGA_PWM_Wrapper &) = delete;
	FPGA_PWM_Wrapper operator=(const FPGA_PWM_Wrapper &) = delete;

	int print_status() override;

private:
	perf_counter_t	_cycle_perf;

	int		_class_instance{-1};

	enum class STATE : uint8_t {
		INIT,
		RUNNING
	};
	STATE _state{STATE::INIT};
	// used to compare and cancel unecessary scheduling changes caused by parameter update
	int32_t _last_fetched_Freq = -1;
	// If this value is above zero, then change freq and scheduling in running state.
	float _targetFreq = 500.0f;
	float _auxFreq = 20000.0f;
	uint16_t _aux_out_channels=0;


	void Run() override;

protected:
	void updateParams() override;

	void updatePWMParams();

	void updatePWMParamTrim();

	int _schd_rate_limit = 400;

	FPGA_SPI_PWM *fpga_pwm = nullptr; // driver handle.

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	MixingOutput _mixing_output{"FPGA_PWM", FPGA_PWM_OUTPUT_MAX_CHANNELS, *this, MixingOutput::SchedulingPolicy::Disabled, true};
};

FPGA_PWM_Wrapper::FPGA_PWM_Wrapper() :
	CDev(PWM_OUTPUT_BASE_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	if (!_mixing_output.useDynamicMixing()) {
		_mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
		_mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);
	}
}

FPGA_PWM_Wrapper::~FPGA_PWM_Wrapper()
{
	if (fpga_pwm != nullptr) { // normally this should not be called.
		PX4_DEBUG("Destruction of FPGA_PWM_Wrapper without pwmDevice unloaded!");
		fpga_pwm->Stop(); // force stop
		delete fpga_pwm;
		fpga_pwm = nullptr;
	}

	perf_free(_cycle_perf);
}

int FPGA_PWM_Wrapper::init()
{
	int ret = CDev::init();

	if (ret != PX4_OK) {
		return ret;
	}

	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	this->ChangeWorkQueue(px4::device_bus_to_wq(fpga_pwm->get_device_id()));

	PX4_INFO("running on I2C bus %d address 0x%.2x", fpga_pwm->get_device_bus(), fpga_pwm->get_device_address());

	ScheduleNow();

	return PX4_OK;
}

void FPGA_PWM_Wrapper::updateParams()
{
	updatePWMParams();
	ModuleParams::updateParams();

}


int32_t getParam(const char * param_name){
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

int32_t getParamWithSprintf(const char * format, int num){

	char buf[16]={0};
	//va_list ap;
	//va_start(ap, format);
	sprintf(buf,format,num);
	//va_end(ap);
	return getParam(buf);
}

void FPGA_PWM_Wrapper::updatePWMParams()
{
	if (_mixing_output.useDynamicMixing()) {
		return;
	}

	int32_t aux_out_temp=0;


	aux_out_temp=getParam("PWM_AUX_OUT");

	for(; aux_out_temp!=0 ;aux_out_temp/=10){    // aux_out_temp may be 1234, which means 0xF in bitmask
		int32_t temp = aux_out_temp % 10;
		_aux_out_channels |= 1<<(temp-1);
	}

	// notice: use the val of PWM_MAIN as the default value, which is different from PX4 document.

	for(int i=0;i<FPGA_PWM_OUTPUT_MAX_CHANNELS;++i){
		uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();
		uint8_t param_index=i+1;  // param_index start from 1
		reverse_pwm_mask&= ~(1<<i);
		if( _aux_out_channels  & (1<<i) ){
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

	_targetFreq=getParam("PWM_MAIN_RATE");
	_auxFreq=getParam("PWM_AUX_RATE");

	if (_mixing_output.mixers()) { // only update trims if mixer loaded
		updatePWMParamTrim();
	}
}




void FPGA_PWM_Wrapper::updatePWMParamTrim()
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

bool FPGA_PWM_Wrapper::updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	if(!fpga_pwm){
		return true;
	}
	return fpga_pwm->updatePWM(outputs, num_outputs) == 0 ? true : false;
}

void FPGA_PWM_Wrapper::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();
		unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

		fpga_pwm->Stop();
		delete fpga_pwm;
		fpga_pwm = nullptr;

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	switch (_state) {
	case STATE::INIT:
		updatePWMParams();  // target frequency fetched, immediately apply it
		if (fpga_pwm->setFreq(0,_targetFreq) != PX4_OK) {
			PX4_ERR("failed to set main pwm frequency to %.2f", (double)_targetFreq);
		}

		if (fpga_pwm->setFreq(1,_auxFreq) != PX4_OK) {
			PX4_ERR("failed to set aux pwm frequency to %.2f", (double)_auxFreq);
		}

		assert(FPAG_I2C_PWM_SUBPWM_NUM==2);
		fpga_pwm->setEXTPWM(&_aux_out_channels);

		_state = STATE::RUNNING;

		ScheduleOnInterval(1000000 / _schd_rate_limit, 1000000 / _schd_rate_limit);
		break;

	case STATE::RUNNING:
		_mixing_output.update();

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			if (fpga_pwm->setFreq(0,_targetFreq) != PX4_OK) {
				PX4_ERR("failed to set main pwm frequency to %.2f", (double)_targetFreq);
			}

			if (fpga_pwm->setFreq(1,_auxFreq) != PX4_OK) {
				PX4_ERR("failed to set aux pwm frequency to %.2f", (double)_auxFreq);
			}
			assert(FPAG_I2C_PWM_SUBPWM_NUM==2);
			fpga_pwm->setEXTPWM(&_aux_out_channels);
		}
		fpga_pwm->updatePWMTrue();
		_mixing_output.updateSubscriptions(false);
		break;
	}
	perf_end(_cycle_perf);
}

int FPGA_PWM_Wrapper::ioctl(cdev::file_t *filep, int cmd, unsigned long arg)
{
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

int FPGA_PWM_Wrapper::print_usage(const char *reason)
{
	if (reason) {
		PX4_DEBUG("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for generate pwm pulse with PCA9685 chip.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

### Implementation
This module depends on ModuleBase and OutputModuleInterface.
IIC communication is based on CDev::I2C

### Examples
It is typically started with:
$ pca9685_pwm_out start -a 64 -b 1

Use the `mixer` command to load mixer files.
`mixer load /dev/pwm_outputX etc/mixers/quad_x.main.mix`
The number X can be acquired by executing
`pca9685_pwm_out status` when this driver is running.
)DESCR_STR");

    	PRINT_MODULE_USAGE_NAME("fpga_pwm_out", "driver");
    	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
        PRINT_MODULE_USAGE_PARAM_INT('a',FPGA_PWM_DEFAULT_ADDRESS,0,255,"device address on this bus",true);
    	PRINT_MODULE_USAGE_PARAM_INT('b',FPGA_PWM_DEFAULT_IICBUS,0,255,"bus that fpag_i2c_pwm is connected to",true);
	PRINT_MODULE_USAGE_PARAM_INT('r',400,50,400,"schedule rate limit",true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int FPGA_PWM_Wrapper::print_status() {
    int ret =  ModuleBase::print_status();
    PX4_INFO("CDev path: %s%d", PWM_OUTPUT_BASE_DEVICE_PATH, this->_class_instance);
    fpga_pwm->status();

    return ret;
}

int FPGA_PWM_Wrapper::custom_command(int argc, char **argv) { // only for test use
    return PX4_OK;
}

int FPGA_PWM_Wrapper::task_spawn(int argc, char **argv) {

	int ch;
	//int address=FPGA_PWM_DEFAULT_ADDRESS;
	//int iicbus=FPGA_PWM_DEFAULT_IICBUS;

	int myoptind = 1;
	const char *myoptarg = nullptr;
	while ((ch = px4_getopt(argc, argv, "a:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
			case 'a':
				//address = atoi(myoptarg);
				break;

			case 'b':
				//iicbus = atoi(myoptarg);
				break;
			case '?':
				PX4_DEBUG("Unsupported args");
				return PX4_ERROR;

			default:
				break;
		}
	}


    auto *instance = new FPGA_PWM_Wrapper();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init() == PX4_OK) {
            return PX4_OK;
        } else {
            PX4_ERR("driver init failed");
        }
    } else {
        PX4_ERR("alloc failed");
	    return PX4_ERROR;
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

void FPGA_PWM_Wrapper::mixerChanged() {
    OutputModuleInterface::mixerChanged();
    if (_mixing_output.mixers()) { // only update trims if mixer loaded
        updatePWMParamTrim();
    }
    _mixing_output.updateSubscriptions(false);
}

extern "C" __EXPORT int fpga_pwm_out_main(int argc, char *argv[]);

int fpga_pwm_out_main(int argc, char *argv[]){
	return FPGA_PWM_Wrapper::main(argc, argv);
}
