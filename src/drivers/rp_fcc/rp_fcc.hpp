#pragma once
#include <px4_platform_common/module.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <drivers/drv_hrt.h>
#include "pthread.h"
extern "C" {
	#include "rp_fcc_protocol.h"
	#include "minihdlc.h"
}

#define DSHOT_BAUDRATE 1152000

using namespace time_literals;

class RP_FCC final : public ModuleBase<RP_FCC>, public OutputModuleInterface
{
public:
	RP_FCC(int fd);
	~RP_FCC() override;
	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	static int open_serial_device(const char *device_name);
	static int serial_config(int fd, int baudrate);

	bool updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	void Run() override;

	int send_dshot_cmd(uint16_t cmd, int dshot_channel_mask);
	void update_params();

	MixingOutput _mixing_output{PARAM_PREFIX, DSHOT_CHANNEL_NUM, *this, MixingOutput::SchedulingPolicy::Auto, true};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	int uart_fd;
	pthread_mutex_t mutex;

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
