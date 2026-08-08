#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/atomic.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <drivers/drv_hrt.h>
#include <drivers/drv_dshot.h>

#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

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
	static constexpr uint32_t DSHOT_DEFAULT_RATE = 300000;
	static constexpr hrt_abstime ESC_INIT_DURATION = 1200_ms;
	static constexpr hrt_abstime ESC_INIT_INTERVAL = 2_ms;
	static constexpr unsigned COMMAND_QUEUE_SIZE = 4;
	static constexpr const char *DEFAULT_DEVICE = "/dev/rk-flexbus-dshot";

	FlexbusDShot(int fd, uint32_t rate_hz, bool telemetry);
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

	struct rk_dshot_telemetry {
		uint32_t erpm[DSHOT_CHANNELS];
		uint16_t raw[DSHOT_CHANNELS];
		uint8_t valid_mask;
		uint8_t no_response_mask;
		uint16_t reserved;
		uint32_t packet_count[DSHOT_CHANNELS];
		uint32_t error_count[DSHOT_CHANNELS];
	};

	struct rk_dshot_telemetry_xfer {
		rk_dshot_frame frame;
		rk_dshot_telemetry telemetry;
	};
	static_assert(sizeof(rk_dshot_telemetry) == 60, "unexpected Flexbus DShot telemetry ABI size");
	static_assert(sizeof(rk_dshot_telemetry_xfer) == 68, "unexpected Flexbus DShot telemetry transfer ABI size");

	struct Command {
		dshot_command_t command{DShot_cmd_motor_stop};
		int num_repetitions{0};
		uint8_t motor_mask{0};
		bool save{false};

		bool valid() const { return num_repetitions > 0; }
	};

	void Run() override;

	static int open_device(const char *device_name, uint32_t rate_hz, bool telemetry);
	bool send_frame(const rk_dshot_frame &frame);
	int send_dshot_cmd(uint16_t cmd, int dshot_channel_mask);
	int enqueue_command(dshot_command_t command, int num_repetitions, uint8_t motor_mask, bool save);
	bool dequeue_command(Command &command);
	void clear_command_queue();
	void handle_vehicle_commands();
	void publish_esc_status();
	void update_params();

	MixingOutput _mixing_output{PARAM_PREFIX, DSHOT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	int _fd{-1};
	uint32_t _rate_hz{DSHOT_DEFAULT_RATE};
	bool _telemetry{false};
	bool _telemetry_xfer_supported{true};
	bool _telemetry_data_available{false};
	hrt_abstime _esc_init_start{0};
	bool _esc_init_done{false};
	uint16_t _last_outputs[DSHOT_CHANNELS] {};
	rk_dshot_telemetry _last_telemetry{};
	uint16_t _esc_status_counter{0};
	Command _current_command{};
	Command _command_queue[COMMAND_QUEUE_SIZE] {};
	unsigned _command_queue_head{0};
	unsigned _command_queue_tail{0};
	unsigned _command_queue_count{0};
	px4::atomic_bool _armed{false};
	pthread_mutex_t _mutex;
	pthread_mutex_t _command_mutex;

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t _io_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": io errors")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::DSHOT_MIN>) _param_dshot_min,
		(ParamInt<px4::params::MOT_POLE_COUNT>) _param_mot_pole_count
	)
};
