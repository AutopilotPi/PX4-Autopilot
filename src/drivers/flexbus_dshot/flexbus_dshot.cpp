#include "flexbus_dshot.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace
{
constexpr char RK_DSHOT_IOCTL_BASE = 'D';
constexpr unsigned RK_DSHOT_MIN_RATE = 150000;
constexpr unsigned RK_DSHOT_MAX_RATE = 1200000;

#define RK_DSHOT_IOC_SET_RATE      _IOW(RK_DSHOT_IOCTL_BASE, 0x00, uint32_t)
#define RK_DSHOT_IOC_SET_TELEMETRY _IOW(RK_DSHOT_IOCTL_BASE, 0x01, uint32_t)
#define RK_DSHOT_IOC_SEND_FRAME    _IOW(RK_DSHOT_IOCTL_BASE, 0x03, FlexbusDShot::rk_dshot_frame)
}

FlexbusDShot::FlexbusDShot(int fd, const char *device_name, uint32_t rate_hz, bool telemetry) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_fd(fd),
	_device_name(device_name),
	_rate_hz(rate_hz),
	_telemetry(telemetry)
{
	_mixing_output.setMaxNumOutputs(DSHOT_CHANNELS);
	pthread_mutex_init(&_mutex, nullptr);
	pthread_mutex_init(&_command_mutex, nullptr);
	update_params();
}

FlexbusDShot::~FlexbusDShot()
{
	rk_dshot_frame frame {};
	send_frame(frame);

	if (_fd >= 0) {
		close(_fd);
	}

	pthread_mutex_destroy(&_command_mutex);
	pthread_mutex_destroy(&_mutex);
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
	perf_free(_io_error_perf);
}

int FlexbusDShot::open_device(const char *device_name, uint32_t rate_hz, bool telemetry)
{
	int fd = open(device_name, O_RDWR | O_NONBLOCK);

	if (fd < 0) {
		PX4_ERR("failed to open %s: %s", device_name, strerror(errno));
		return -1;
	}

	if (rate_hz > 0) {
		if (rate_hz < RK_DSHOT_MIN_RATE || rate_hz > RK_DSHOT_MAX_RATE) {
			PX4_ERR("rate must be between %u and %u", RK_DSHOT_MIN_RATE, RK_DSHOT_MAX_RATE);
			close(fd);
			return -1;
		}

		if (ioctl(fd, RK_DSHOT_IOC_SET_RATE, &rate_hz) < 0) {
			PX4_ERR("failed to set DShot rate: %s", strerror(errno));
			close(fd);
			return -1;
		}
	}

	uint32_t telemetry_value = telemetry ? 1 : 0;

	if (ioctl(fd, RK_DSHOT_IOC_SET_TELEMETRY, &telemetry_value) < 0) {
		PX4_ERR("failed to set telemetry: %s", strerror(errno));
		close(fd);
		return -1;
	}

	return fd;
}

int FlexbusDShot::task_spawn(int argc, char *argv[])
{
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int ch;

	const char *device_name = DEFAULT_DEVICE;
	uint32_t rate_hz = DSHOT_DEFAULT_RATE;
	bool telemetry = false;

	while ((ch = px4_getopt(argc, argv, "d:r:t", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case 'r':
			rate_hz = strtoul(myoptarg, nullptr, 0);
			break;

		case 't':
			telemetry = true;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	int fd = open_device(device_name, rate_hz, telemetry);

	if (fd < 0) {
		return PX4_ERROR;
	}

	FlexbusDShot *instance = new FlexbusDShot(fd, device_name, rate_hz, telemetry);

	if (instance == nullptr) {
		PX4_ERR("failed to allocate instance");
		close(fd);
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();

	return PX4_OK;
}

bool FlexbusDShot::send_frame(const rk_dshot_frame &frame)
{
	if (_fd < 0) {
		perf_count(_io_error_perf);
		return false;
	}

	if (ioctl(_fd, RK_DSHOT_IOC_SEND_FRAME, &frame) < 0) {
		perf_count(_io_error_perf);
		return false;
	}

	return true;
}

bool FlexbusDShot::updateOutputs(uint16_t outputs[MAX_ACTUATORS],
				 unsigned num_outputs, unsigned num_control_groups_updated)
{
	(void)num_control_groups_updated;

	rk_dshot_frame frame {};
	bool command_sent = false;

	for (unsigned i = 0; i < DSHOT_CHANNELS; ++i) {
		uint16_t value = DSHOT_DISARM_VALUE;

		if (i < num_outputs) {
			const uint16_t output = math::min(outputs[i], DSHOT_MAX_THROTTLE);

			// MixingOutput uses the same 0..1999 throttle range as the standard PX4
			// DShot driver. Values 0..47 are reserved for DShot commands, so add the
			// command offset only to armed throttle values. A zero output must remain
			// zero so it is transmitted as the motor-stop command.
			if (output == DSHOT_DISARM_VALUE) {
				if (_current_command.valid() && (_current_command.motor_mask & (1u << i))) {
					value = _current_command.command;
					command_sent = true;
				}

			} else {
				value = output + DSHOT_COMMAND_OFFSET;
			}
		}

		frame.value[i] = value;
	}

	pthread_mutex_lock(&_mutex);
	bool ret = send_frame(frame);

	if (ret) {
		memcpy(_last_outputs, frame.value, sizeof(_last_outputs));

		if (command_sent && _current_command.valid()) {
			--_current_command.num_repetitions;

			// Persist the setting after the requested command burst.
			if (_current_command.num_repetitions == 0 && _current_command.save) {
				_current_command.command = DShot_cmd_save_settings;
				_current_command.num_repetitions = 10;
				_current_command.save = false;
			}
		}
	}

	pthread_mutex_unlock(&_mutex);

	return ret;
}

int FlexbusDShot::send_dshot_cmd(uint16_t cmd, int dshot_channel_mask)
{
	rk_dshot_frame frame {};

	for (unsigned i = 0; i < DSHOT_CHANNELS; ++i) {
		frame.value[i] = (dshot_channel_mask & (1 << i)) ? cmd : DSHOT_DISARM_VALUE;
	}

	return send_frame(frame) ? PX4_OK : PX4_ERROR;
}

int FlexbusDShot::enqueue_command(dshot_command_t command, int num_repetitions, uint8_t motor_mask, bool save)
{
	Command queued_command{};
	queued_command.command = command;
	queued_command.num_repetitions = num_repetitions;
	queued_command.motor_mask = motor_mask;
	queued_command.save = save;

	pthread_mutex_lock(&_command_mutex);

	if (_armed.load()) {
		pthread_mutex_unlock(&_command_mutex);
		PX4_WARN("DShot commands require disarmed outputs");
		return -EBUSY;
	}

	if (_command_queue_count >= COMMAND_QUEUE_SIZE) {
		pthread_mutex_unlock(&_command_mutex);
		PX4_WARN("DShot command queue full");
		return -EBUSY;
	}

	_command_queue[_command_queue_tail] = queued_command;
	_command_queue_tail = (_command_queue_tail + 1) % COMMAND_QUEUE_SIZE;
	++_command_queue_count;
	pthread_mutex_unlock(&_command_mutex);

	ScheduleNow();
	return PX4_OK;
}

bool FlexbusDShot::dequeue_command(Command &command)
{
	pthread_mutex_lock(&_command_mutex);

	if (_command_queue_count == 0) {
		pthread_mutex_unlock(&_command_mutex);
		return false;
	}

	command = _command_queue[_command_queue_head];
	_command_queue_head = (_command_queue_head + 1) % COMMAND_QUEUE_SIZE;
	--_command_queue_count;
	pthread_mutex_unlock(&_command_mutex);
	return true;
}

void FlexbusDShot::clear_command_queue()
{
	pthread_mutex_lock(&_command_mutex);
	_command_queue_head = 0;
	_command_queue_tail = 0;
	_command_queue_count = 0;
	pthread_mutex_unlock(&_command_mutex);
}

void FlexbusDShot::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	if (!_esc_init_done) {
		if (_esc_init_start == 0) {
			_esc_init_start = hrt_absolute_time();
			PX4_INFO("sending zero throttle frames for ESC init");
		}

		rk_dshot_frame frame {};

		pthread_mutex_lock(&_mutex);
		send_frame(frame);
		pthread_mutex_unlock(&_mutex);

		if (hrt_elapsed_time(&_esc_init_start) < ESC_INIT_DURATION) {
			perf_end(_cycle_perf);
			ScheduleDelayed(ESC_INIT_INTERVAL);
			return;
		}

		_esc_init_done = true;
		PX4_INFO("ESC init complete");
	}

	_mixing_output.update();
	const bool was_armed = _armed.load();
	_armed.store(_mixing_output.armed().armed);

	if (_armed.load()) {
		if (_current_command.valid()) {
			PX4_WARN("cancelling DShot command while armed");
			_current_command = {};
		}

		if (!was_armed) {
			clear_command_queue();
		}
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		update_params();
	}

	handle_vehicle_commands();

	if (!_armed.load() && !_current_command.valid()) {
		dequeue_command(_current_command);
	}

	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

void FlexbusDShot::handle_vehicle_commands()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR) {
			continue;
		}

		int function = (int)(vehicle_command.param5 + 0.5);

		if (function < 1000) {
			const int first_motor_function = 1; // MAVLink ACTUATOR_OUTPUT_FUNCTION_MOTOR1
			const int first_servo_function = 33;

			if (function >= first_motor_function && function < first_motor_function + actuator_test_s::MAX_NUM_MOTORS) {
				function = function - first_motor_function + actuator_test_s::FUNCTION_MOTOR1;

			} else if (function >= first_servo_function
				   && function < first_servo_function + actuator_test_s::MAX_NUM_SERVOS) {
				function = function - first_servo_function + actuator_test_s::FUNCTION_SERVO1;

			} else {
				function = INT32_MAX;
			}

		} else {
			function -= 1000;
		}

		const int type = (int)(vehicle_command.param1 + 0.5f);
		int index = -1;

		for (unsigned i = 0; i < DSHOT_CHANNELS; ++i) {
			if ((int)_mixing_output.outputFunction(i) == function) {
				index = i;
				break;
			}
		}

		vehicle_command_ack_s command_ack{};
		command_ack.command = vehicle_command.command;
		command_ack.target_system = vehicle_command.source_system;
		command_ack.target_component = vehicle_command.source_component;
		command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

		if (index != -1) {
			dshot_command_t command = DShot_cmd_motor_stop;

			switch (type) {
			case 1: command = DShot_cmd_beacon1; break;
			case 2: command = DShot_cmd_3d_mode_on; break;
			case 3: command = DShot_cmd_3d_mode_off; break;
			case 4: command = DShot_cmd_spin_direction_1; break;
			case 5: command = DShot_cmd_spin_direction_2; break;
			}

			if (command == DShot_cmd_motor_stop) {
				PX4_WARN("unknown actuator configuration command: %i", type);

			} else {
				PX4_DEBUG("setting DShot command: index: %i type: %i", index, type);
				const int ret = enqueue_command(command, 10, 1u << index, true);

				if (ret == PX4_OK) {
					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else if (ret == -EBUSY) {
					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}
			}
		}

		command_ack.timestamp = hrt_absolute_time();
		_command_ack_pub.publish(command_ack);
	}
}

int FlexbusDShot::custom_command(int argc, char *argv[])
{
	int arg_index = 0;
	int motor_index = 0;
	int cmd = 0;
	int repeat_cnt = 1;
	const char *device_name = DEFAULT_DEVICE;
	uint32_t rate_hz = DSHOT_DEFAULT_RATE;
	bool telemetry = false;

	if (argc > 0 && !strcmp(argv[0], "cmd")) {
		arg_index = 1;
	}

	while (arg_index < argc) {
		const char *arg = argv[arg_index++];

		if (!strcmp(arg, "-d")) {
			if (arg_index >= argc) {
				return print_usage("missing -d argument");
			}

			device_name = argv[arg_index++];

		} else if (!strcmp(arg, "-r")) {
			if (arg_index >= argc) {
				return print_usage("missing -r argument");
			}

			rate_hz = strtoul(argv[arg_index++], nullptr, 0);

		} else if (!strcmp(arg, "-t")) {
			telemetry = true;

		} else if (!strcmp(arg, "-m")) {
			if (arg_index >= argc) {
				return print_usage("missing -m argument");
			}

			motor_index = strtol(argv[arg_index++], nullptr, 0);

			if (motor_index < 0 || motor_index >= (int)DSHOT_CHANNELS) {
				return print_usage("motor index must be between 0 and 3");
			}

		} else if (!strcmp(arg, "-c")) {
			if (arg_index >= argc) {
				return print_usage("missing -c argument");
			}

			cmd = strtol(argv[arg_index++], nullptr, 0);

			if (cmd < 0 || cmd > 47) {
				return print_usage("command must be between 0 and 47");
			}

		} else if (!strcmp(arg, "-n")) {
			if (arg_index >= argc) {
				return print_usage("missing -n argument");
			}

			repeat_cnt = strtol(argv[arg_index++], nullptr, 0);

			if (repeat_cnt < 1 || repeat_cnt > 10) {
				return print_usage("repeat count must be between 1 and 10");
			}

		} else {
			return print_usage("unrecognized argument");
		}
	}

	int fd = -1;
	FlexbusDShot *instance = nullptr;

	if (is_running()) {
		instance = _object.load();

		if (instance == nullptr) {
			PX4_ERR("instance not found");
			return PX4_ERROR;
		}

		const int ret = instance->enqueue_command(static_cast<dshot_command_t>(cmd), repeat_cnt,
				1u << motor_index, false);

		if (ret == PX4_OK) {
			PX4_INFO("queued DShot command %d for motor %d repeat %d", cmd, motor_index, repeat_cnt);
		}

		return ret;

	} else {
		fd = open_device(device_name, rate_hz, telemetry);

		if (fd < 0) {
			return PX4_ERROR;
		}

		instance = new FlexbusDShot(fd, device_name, rate_hz, telemetry);

		if (instance == nullptr) {
			PX4_ERR("failed to allocate temporary instance");
			close(fd);
			return PX4_ERROR;
		}
	}

	int ret = PX4_OK;

	pthread_mutex_lock(&instance->_mutex);

	for (int i = 0; i < repeat_cnt; ++i) {
		ret = instance->send_dshot_cmd(cmd, 1 << motor_index);

		if (ret != PX4_OK) {
			break;
		}

		usleep(200);
	}

	usleep(260000);
	pthread_mutex_unlock(&instance->_mutex);

	if (fd >= 0) {
		delete instance;
	}

	if (ret != PX4_OK) {
		PX4_ERR("failed to send DShot command");
		return PX4_ERROR;
	}

	PX4_INFO("sent DShot command %d to motor %d repeat %d", cmd, motor_index, repeat_cnt);
	return PX4_OK;
}

void FlexbusDShot::update_params()
{
	ModuleParams::updateParams();

	// Match the standard PX4 DShot driver's normalized minimum-throttle
	// semantics. The DShot command offset is added immediately before the
	// raw frame is sent to the Flexbus kernel driver.
	_mixing_output.setAllMinValues(math::constrain(static_cast<int>(_param_dshot_min.get()
				       * static_cast<float>(DSHOT_MAX_THROTTLE)),
				       static_cast<int>(DSHOT_MIN_THROTTLE), static_cast<int>(DSHOT_MAX_THROTTLE)));
}

int FlexbusDShot::print_status()
{
	PX4_INFO("device: %s", _device_name);
	PX4_INFO("rate: %u Hz, telemetry: %s", _rate_hz, _telemetry ? "enabled" : "disabled");
	PX4_INFO("outputs: %u", DSHOT_CHANNELS);
	PX4_INFO("ESC init: %s", _esc_init_done ? "complete" : "running");
	_mixing_output.printStatus();
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	perf_print_counter(_io_error_perf);
	return PX4_OK;
}

int FlexbusDShot::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Drive DShot outputs through the Rockchip flexbus DShot kernel driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flexbus_dshot", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', DEFAULT_DEVICE, nullptr, "Device path", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', DSHOT_DEFAULT_RATE, RK_DSHOT_MIN_RATE, RK_DSHOT_MAX_RATE, "DShot rate in Hz", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('t', "Enable DShot telemetry bit", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("cmd", "Send DShot command to a motor");
	PRINT_MODULE_USAGE_PARAM_INT('m', 0, 0, DSHOT_CHANNELS - 1, "Motor index", true);
	PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, 47, "DShot command", true);
	PRINT_MODULE_USAGE_PARAM_INT('n', 1, 1, 10, "Repeat count", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flexbus_dshot_main(int argc, char *argv[])
{
	return FlexbusDShot::main(argc, argv);
}
