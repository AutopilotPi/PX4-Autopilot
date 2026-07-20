#pragma once

#include <px4_platform_common/module.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <drivers/drv_hrt.h>
#include <drivers/drv_dshot.h>

#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <pthread.h>
#include <stdint.h>

using namespace time_literals;

class FlexbusDShot final : public ModuleBase<FlexbusDShot>, public OutputModuleInterface
{
public:
	static constexpr unsigned DSHOT_CHANNELS = 4;
	static constexpr uint16_t DSHOT_DISARM_VALUE = 0;
	static constexpr uint16_t DSHOT_MIN_THROTTLE = 1;
	static constexpr uint16_t DSHOT_MAX_THROTTLE = 1999;
	static constexpr uint16_t DSHOT_COMMAND_OFFSET = DShot_cmd_MIN_throttle;
	static constexpr uint32_t DSHOT_DEFAULT_RATE = 600000;
	static constexpr hrt_abstime ESC_INIT_DURATION = 1200_ms;
	static constexpr hrt_abstime ESC_INIT_INTERVAL = 2_ms;
	static constexpr const char *DEFAULT_DEVICE = "/dev/rk-flexbus-dshot";

	FlexbusDShot(int fd, const char *device_name, uint32_t rate_hz, bool telemetry);
	~FlexbusDShot() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	bool updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	struct rk_dshot_frame {
		uint16_t value[DSHOT_CHANNELS];
	};

	void Run() override;

	static int open_device(const char *device_name, uint32_t rate_hz, bool telemetry);
	bool send_frame(const rk_dshot_frame &frame);
	int send_dshot_cmd(uint16_t cmd, int dshot_channel_mask);
	void update_params();

	MixingOutput _mixing_output{PARAM_PREFIX, DSHOT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	int _fd{-1};
	const char *_device_name{DEFAULT_DEVICE};
	uint32_t _rate_hz{DSHOT_DEFAULT_RATE};
	bool _telemetry{false};
	hrt_abstime _esc_init_start{0};
	bool _esc_init_done{false};
	uint16_t _last_outputs[DSHOT_CHANNELS] {};
	pthread_mutex_t _mutex;

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t _io_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": io errors")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::DSHOT_MIN>) _param_dshot_min
	)
};
