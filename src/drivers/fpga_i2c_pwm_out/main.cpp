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
 * @file fpga_i2c_pwm/main.cpp
 * A cross-platform driver and wrapper for fpga_i2c_pwm modules.
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

#include "fpga_i2c_pwm_out.h"

#define FPGA_I2C_PWM_DEFAULT_IICBUS  0
#define FPGA_I2C_PWM_DEFAULT_ADDRESS (0x20)

using namespace fpga_i2c_pwm;
using namespace time_literals;

class FPGA_I2C_PWM_Wrapper : public cdev::CDev, public ModuleBase<FPGA_I2C_PWM_Wrapper>, public OutputModuleInterface
{
public:

	FPGA_I2C_PWM_Wrapper();
	~FPGA_I2C_PWM_Wrapper() override ;

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

	FPGA_I2C_PWM_Wrapper(const FPGA_I2C_PWM_Wrapper &) = delete;
	FPGA_I2C_PWM_Wrapper operator=(const FPGA_I2C_PWM_Wrapper &) = delete;

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
	float _targetFreq = 8000.0f;


	void Run() override;

protected:
	void updateParams() override;

	void updatePWMParams();

	void updatePWMParamTrim();

	int _schd_rate_limit = 400;

	FPGA_I2C_PWM *fpga_i2c_pwm = nullptr; // driver handle.

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	MixingOutput _mixing_output{"FPGA_I2C_PWM", FPGA_PWM_OUTPUT_MAX_CHANNELS, *this, MixingOutput::SchedulingPolicy::Disabled, true};
};

FPGA_I2C_PWM_Wrapper::FPGA_I2C_PWM_Wrapper() :
	CDev(PWM_OUTPUT_BASE_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	if (!_mixing_output.useDynamicMixing()) {
		_mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
		_mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);
	}
}

FPGA_I2C_PWM_Wrapper::~FPGA_I2C_PWM_Wrapper()
{
	if (fpga_i2c_pwm != nullptr) { // normally this should not be called.
		PX4_DEBUG("Destruction of FPGA_I2C_PWM_Wrapper without pwmDevice unloaded!");
		fpga_i2c_pwm->Stop(); // force stop
		delete fpga_i2c_pwm;
		fpga_i2c_pwm = nullptr;
	}

	perf_free(_cycle_perf);
}

int FPGA_I2C_PWM_Wrapper::init()
{
	int ret = CDev::init();

	if (ret != PX4_OK) {
		return ret;
	}

	ret = fpga_i2c_pwm->init();

	if (ret != PX4_OK) {
		return ret;
	}

	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	this->ChangeWorkQueue(px4::device_bus_to_wq(fpga_i2c_pwm->get_device_id()));

	PX4_INFO("running on I2C bus %d address 0x%.2x", fpga_i2c_pwm->get_device_bus(), fpga_i2c_pwm->get_device_address());

	ScheduleNow();

	return PX4_OK;
}

void FPGA_I2C_PWM_Wrapper::updateParams()
{
	updatePWMParams();
	ModuleParams::updateParams();

}

void FPGA_I2C_PWM_Wrapper::updatePWMParams()
{
	if (_mixing_output.useDynamicMixing()) {
		return;
	}

	// update pwm params
	const char *pname_format_pwm_ch_max[2] = {"PWM_MAIN_MAX%d", "PWM_AUX_MAX%d"};
	const char *pname_format_pwm_ch_min[2] = {"PWM_MAIN_MIN%d", "PWM_AUX_MIN%d"};
	const char *pname_format_pwm_ch_fail[2] = {"PWM_MAIN_FAIL%d", "PWM_AUX_FAIL%d"};
	const char *pname_format_pwm_ch_dis[2] = {"PWM_MAIN_DIS%d", "PWM_AUX_DIS%d"};
	const char *pname_format_pwm_ch_rev[2] = {"PWM_MAIN_REV%d", "PWM_AUX_REV%d"};

	int32_t default_pwm_max = PWM_DEFAULT_MAX,
		default_pwm_min = PWM_DEFAULT_MIN,
		default_pwm_fail = PWM_DEFAULT_MIN,
		default_pwm_dis = PWM_MOTOR_OFF;

	param_t param_h = param_find("PWM_MAIN_MAX");

	if (param_h != PARAM_INVALID) {
		param_get(param_h, &default_pwm_max);

	} else {
		PX4_DEBUG("PARAM_INVALID: %s", "PWM_MAIN_MAX");
	}

	param_h = param_find("PWM_MAIN_MIN");

	if (param_h != PARAM_INVALID) {
		param_get(param_h, &default_pwm_min);

	} else {
		PX4_DEBUG("PARAM_INVALID: %s", "PWM_MAIN_MIN");
	}

	param_h = param_find("PWM_MAIN_RATE");

	if (param_h != PARAM_INVALID) {
		int32_t pval = 0;
		param_get(param_h, &pval);

		if (_last_fetched_Freq != pval) {
			_last_fetched_Freq = pval;
			_targetFreq = (float)pval;  // update only if changed
		}

	} else {
		PX4_DEBUG("PARAM_INVALID: %s", "PWM_MAIN_RATE");
	}

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

		sprintf(pname, pname_format_pwm_ch_max[param_group], param_index);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.maxValue(i) = pval;

			} else {
				_mixing_output.maxValue(i) = default_pwm_max;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_pwm_ch_min[param_group], param_index);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.minValue(i) = pval;

			} else {
				_mixing_output.minValue(i) = default_pwm_min;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_pwm_ch_fail[param_group], param_index);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.failsafeValue(i) = pval;

			} else {
				_mixing_output.failsafeValue(i) = default_pwm_fail;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_pwm_ch_dis[param_group], param_index);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.disarmedValue(i) = pval;

			} else {
				_mixing_output.disarmedValue(i) = default_pwm_dis;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_pwm_ch_rev[param_group], param_index);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();
			int32_t pval = 0;
			param_get(param_h, &pval);
			reverse_pwm_mask &= (0xfffe << i);  // clear this bit
			reverse_pwm_mask |= (((uint16_t)(pval != 0)) << i); // set to new val

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}
	}

	if (_mixing_output.mixers()) { // only update trims if mixer loaded
		updatePWMParamTrim();
	}
}

void FPGA_I2C_PWM_Wrapper::updatePWMParamTrim()
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

bool FPGA_I2C_PWM_Wrapper::updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	return fpga_i2c_pwm->updatePWM(outputs, num_outputs) == 0 ? true : false;
}

void FPGA_I2C_PWM_Wrapper::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();
		unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

		fpga_i2c_pwm->Stop();
		delete fpga_i2c_pwm;
		fpga_i2c_pwm = nullptr;

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	switch (_state) {
	case STATE::INIT:
		//fpga_i2c_pwm->initReg();
		updatePWMParams();  // target frequency fetched, immediately apply it

		if (_targetFreq > 0.0f) {
			if (fpga_i2c_pwm->setFreq(_targetFreq) != PX4_OK) {
				PX4_ERR("failed to set pwm frequency to %.2f, fall back to 50Hz", (double)_targetFreq);
			}

			_targetFreq = -1.0f;

		} else {
			// should not happen
			PX4_ERR("INIT failed: invalid initial frequency settings");
		}

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
		}

		_mixing_output.updateSubscriptions(false);

		if (_targetFreq > 0.0f) { // check if frequency should be changed
			ScheduleClear();
			fpga_i2c_pwm->disableAllOutput();

			if (fpga_i2c_pwm->setFreq(_targetFreq) != PX4_OK) {
				PX4_ERR("failed to set pwm frequency, fall back to 50Hz");
				fpga_i2c_pwm->setFreq(50.0f);	// this should not fail
			}

			ScheduleOnInterval(1000000 / 400, 1000000 / 400);
			//fpga_i2c_pwm->enableOutput()
		}

		break;
	}

	perf_end(_cycle_perf);
}

int FPGA_I2C_PWM_Wrapper::ioctl(cdev::file_t *filep, int cmd, unsigned long arg)
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

int FPGA_I2C_PWM_Wrapper::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
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

    PRINT_MODULE_USAGE_NAME("fpga_i2c_pwm_out", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
    PRINT_MODULE_USAGE_PARAM_INT('a',32,0,255,"device address on this bus",true);
	PRINT_MODULE_USAGE_PARAM_INT('b',0,0,255,"bus that pca9685 is connected to",true);
	PRINT_MODULE_USAGE_PARAM_INT('r',400,50,400,"schedule rate limit",true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int FPGA_I2C_PWM_Wrapper::print_status() {
    int ret =  ModuleBase::print_status();
    PX4_INFO("fpga_i2c_pwm_out @I2C Bus %d, address 0x%.2x, true frequency %.5f",
            fpga_i2c_pwm->get_device_bus(),
            fpga_i2c_pwm->get_device_address(),
             (double)(fpga_i2c_pwm->getFrequency()));
    PX4_INFO("CDev path: %s%d", PWM_OUTPUT_BASE_DEVICE_PATH, this->_class_instance);
    fpga_i2c_pwm->status();

    return ret;
}

int FPGA_I2C_PWM_Wrapper::custom_command(int argc, char **argv) { // only for test use
    return PX4_OK;
}

int FPGA_I2C_PWM_Wrapper::task_spawn(int argc, char **argv) {

	int ch;
	int address=FPGA_I2C_PWM_DEFAULT_ADDRESS;
	int iicbus=FPGA_I2C_PWM_DEFAULT_IICBUS;

	int myoptind = 1;
	const char *myoptarg = nullptr;
	while ((ch = px4_getopt(argc, argv, "a:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
			case 'a':
				address = atoi(myoptarg);
				break;

			case 'b':
				iicbus = atoi(myoptarg);
				break;
			case '?':
				PX4_WARN("Unsupported args");
				return PX4_ERROR;

			default:
				break;
		}
	}

    auto *instance = new FPGA_I2C_PWM_Wrapper();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        instance->fpga_i2c_pwm = new FPGA_I2C_PWM(iicbus, address);
        if(instance->fpga_i2c_pwm==nullptr){
            PX4_ERR("alloc failed");
            goto driverInstanceAllocFailed;
        }

        if (instance->init() == PX4_OK) {
            return PX4_OK;
        } else {
            PX4_ERR("driver init failed");
            delete instance->fpga_i2c_pwm;
            instance->fpga_i2c_pwm=nullptr;
        }
    } else {
        PX4_ERR("alloc failed");
	    return PX4_ERROR;
    }

    driverInstanceAllocFailed:
    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

void FPGA_I2C_PWM_Wrapper::mixerChanged() {
    OutputModuleInterface::mixerChanged();
    if (_mixing_output.mixers()) { // only update trims if mixer loaded
        updatePWMParamTrim();
    }
    _mixing_output.updateSubscriptions(false);
}

extern "C" __EXPORT int fpga_i2c_pwm_out_main(int argc, char *argv[]);

int fpga_i2c_pwm_out_main(int argc, char *argv[]){
	return FPGA_I2C_PWM_Wrapper::main(argc, argv);
}
