/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file fpga_spi_pwm_out/main.cpp
 *
 * This file serves as the wrapper layer for fpga_spi_pwm_out driver, working with parameters
 * and scheduling stuffs on PX4 side.
 *
 */

#include <px4_log.h>
#include <drivers/device/device.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/sem.hpp>

#include "fpga_spi_pwm_out.h"


using namespace time_literals;
namespace fpga_spi_pwm
{
class FPGA_SPI_PWM_Wrapper : public ModuleBase<FPGA_SPI_PWM_Wrapper>, public OutputModuleInterface
{
public:
	FPGA_SPI_PWM_Wrapper();
	~FPGA_SPI_PWM_Wrapper() override;
	FPGA_SPI_PWM_Wrapper(const FPGA_SPI_PWM_Wrapper &) = delete;
	FPGA_SPI_PWM_Wrapper operator=(const FPGA_SPI_PWM_Wrapper &) = delete;

	int init();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

	int print_status() override;

protected:
	void updateParams() override;

private:
	perf_counter_t	_cycle_perf;

	enum class STATE : uint8_t {
		INIT,
		RUNNING
	} state{STATE::INIT};

	FPGA_SPI_PWM *fpga_spi_pwm_out = nullptr;
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	MixingOutput _mixing_output {
		"FPGA",
		FPGA_PWM_OUTPUT_MAX_CHANNELS,
		*this,
		MixingOutput::SchedulingPolicy::Disabled,
		true
	};

	float param_pwm_freq, previous_pwm_freq;
	float param_schd_rate, previous_schd_rate;
	uint32_t param_duty_mode;

	void Run() override;
};

FPGA_SPI_PWM_Wrapper::FPGA_SPI_PWM_Wrapper() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

FPGA_SPI_PWM_Wrapper::~FPGA_SPI_PWM_Wrapper()
{
	if (fpga_spi_pwm_out != nullptr) {
		fpga_spi_pwm_out->disableAllOutput();
		delete fpga_spi_pwm_out;
	}

	perf_free(_cycle_perf);
}

int FPGA_SPI_PWM_Wrapper::init()
{
	int ret = fpga_spi_pwm_out->init();

	if (ret != PX4_OK) { return ret; }

	this->ChangeWorkQueue(px4::device_bus_to_wq(fpga_spi_pwm_out->get_device_id()));

	PX4_INFO("running on SPI bus %d.%d", fpga_spi_pwm_out->get_device_bus(), fpga_spi_pwm_out->get_device_id() >> 8 & 0xff);

	ScheduleNow();

	return PX4_OK;
}

bool FPGA_SPI_PWM_Wrapper::updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	if (state != STATE::RUNNING) { return false; }

	num_outputs = num_outputs > FPGA_PWM_OUTPUT_MAX_CHANNELS ? FPGA_PWM_OUTPUT_MAX_CHANNELS : num_outputs;

	for (uint8_t i = 0; i < num_outputs; ++i) {
		uint8_t chn_mode = fpga_spi_pwm_out -> _channel_mode_map[i];
		chn_mode = chn_mode > (FPAG_PWM_SUBPWM_NUM - 1) ? (FPAG_PWM_SUBPWM_NUM - 1) : chn_mode;
		float freq = fpga_spi_pwm_out->_freqs[chn_mode];
		uint16_t period = fpga_spi_pwm_out->_periods[chn_mode];
		uint16_t out = 0;

		if (freq > 500) {
			out = outputs[i] * period / 10000; // duty mode

		} else {
			out = outputs[i] * freq * period / 1000 / 1000;  // pulse width mode
		}

		fpga_spi_pwm_out -> writeReg(CCR0_REG + i, out);
	}

	return true;
}

void FPGA_SPI_PWM_Wrapper::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		fpga_spi_pwm_out->disableAllOutput();
		delete fpga_spi_pwm_out;
		fpga_spi_pwm_out = nullptr;

		exit_and_cleanup();
		return;
	}

	switch (state) {
	case STATE::INIT:
		updateParams();
		state = STATE::RUNNING;
		ScheduleOnInterval(2500, 0);
		break;

	case STATE::RUNNING:
		perf_begin(_cycle_perf);

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

		perf_end(_cycle_perf);
		break;
	}
}

int FPGA_SPI_PWM_Wrapper::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is a fpga_spi_pwm_out PWM output driver.

It runs on SPI workqueue which is synchronous with FC control loop.

### Examples
It is typically started with:
$ fpga_pwm_out start -b 0 -s 2

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("fpga_pwm_out", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
    PRINT_MODULE_USAGE_PARAM_INT('s', 0 ,0, 255,"spi device index",true);
    PRINT_MODULE_USAGE_PARAM_INT('b',1,0,255,"bus that fpga_pwm_out is connected to",true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int FPGA_SPI_PWM_Wrapper::print_status() {
    int ret =  ModuleBase::print_status();
    return ret;
}

int FPGA_SPI_PWM_Wrapper::custom_command(int argc, char **argv) {
    return PX4_OK;
}

int FPGA_SPI_PWM_Wrapper::task_spawn(int argc, char **argv) {
	int ch;
	int device_num = 0;
	uint32_t spi_freq=5000000;
	int bus = 0;
	enum spi_mode_e spi_mode = SPIDEV_MODE0;

	int myoptind = 1;
	const char *myoptarg = nullptr;
	while ((ch = px4_getopt(argc, argv, "s:b:f:m", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
			case 's':
				device_num=strtol(myoptarg, nullptr, 0);
				break;
			case 'b':
				bus = strtol(myoptarg, nullptr, 0);
				break;
			case 'f':
				spi_freq = strtol(myoptarg, nullptr, 0);
				break;
			case 'm':
				spi_mode = ( enum spi_mode_e )strtol(myoptarg, nullptr, 0);
				break;
			case '?':
				PX4_WARN("Unsupported args");
				return PX4_ERROR;

			default:
				break;
		}
	}

    auto *instance = new FPGA_SPI_PWM_Wrapper();
    uint32_t device_id = PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, device_num);
    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        instance->fpga_spi_pwm_out = new FPGA_SPI_PWM(bus,device_id,spi_mode,spi_freq);
        if(instance->fpga_spi_pwm_out==nullptr){
            PX4_ERR("alloc failed");
            goto driverInstanceAllocFailed;
        }

        if (instance->init() == PX4_OK) {
            return PX4_OK;
        } else {
            PX4_ERR("driver init failed");
            delete instance->fpga_spi_pwm_out;
            instance->fpga_spi_pwm_out=nullptr;
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

void FPGA_SPI_PWM_Wrapper::updateParams() {
    ModuleParams::updateParams();

    param_t param = param_find("FPGA_PREDIVIDER");
    if (param != PARAM_INVALID) {
	int predivider=0;
        param_get(param, &predivider);
	fpga_spi_pwm_out->setPredivider(predivider);
    } else {
        PX4_ERR("param FPGA_PWM_PREDIVIDER not found");
    }

    param = param_find("FPGA_PWM_FREQ");
    if (param != PARAM_INVALID) {
	float freq=0;
        param_get(param, &freq);
	fpga_spi_pwm_out->setFreq(0,freq);
    } else {
        PX4_ERR("param FPGA_PWM_FREQ not found");
    }

    param = param_find("FPGA_PWM_FREQ2");
    if (param != PARAM_INVALID) {
	float freq=0;
        param_get(param, &freq);
	fpga_spi_pwm_out->setFreq(1,freq);
    } else {
        PX4_ERR("param FPGA_PWM_FREQ2 not found");
    }

    param = param_find("FPGA_PWM_CHN_MAP");
    if (param != PARAM_INVALID) {
	int channel_map=0;
        param_get(param, &channel_map);
	uint16_t channel_map_low16bit = channel_map & 0xffff;  // prevent failing when running at big endian machine.
	fpga_spi_pwm_out->setEXTPWM((uint16_t *)&channel_map_low16bit);
    } else {
        PX4_ERR("param FPGA_PWM_CHN_MAP not found");
    }

}
}

extern "C" __EXPORT int fpga_pwm_out_main(int argc, char *argv[]){
	return fpga_spi_pwm::FPGA_SPI_PWM_Wrapper::main(argc, argv);
}
