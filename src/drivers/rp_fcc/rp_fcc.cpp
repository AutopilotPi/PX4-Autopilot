/****************************************************************************
 *
 * Copyright (C) 2024 PX4 Development Team. All rights reserved.
 * Author: canming huang <huangcmzzk@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *  used to endorse or promote products derived from this software
 *  without specific prior written permission.
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

#include "rp_fcc.hpp"

#include <px4_platform_common/getopt.h>

#if defined(__PX4_LINUX)
#include <sys/ioctl.h>
#include <asm-generic/termbits.h>
#else
#include <termios.h>
#endif

#include <drivers/drv_dshot.h>


RP_FCC::RP_FCC(int fd) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),uart_fd{fd}
{
	_mixing_output.setMaxNumOutputs(DSHOT_CHANNEL_NUM);
	pthread_mutex_init(&mutex, NULL);
	// Getting initial parameter values
	update_params();
}

RP_FCC::~RP_FCC(){
	close(uart_fd);
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int RP_FCC::open_serial_device(const char *device_name)
{
	if (device_name == nullptr) {
		PX4_ERR("Device name is null");
		return PX4_ERROR;
	}

	int fd = ::open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		PX4_ERR("Failed to open device %s: %s", device_name, strerror(errno));
		return PX4_ERROR;
	}

	PX4_INFO("Opened device %s successfully", device_name);

	if(serial_config(fd, DSHOT_BAUDRATE) != 0) {
		PX4_ERR("Failed to configure serial device %s", device_name);
		::close(fd);
		return PX4_ERROR;
	}

	PX4_INFO("Configured serial device %s with baudrate %d", device_name, DSHOT_BAUDRATE);
	return fd;
}

int RP_FCC::serial_config(int fd, int baudrate)
{

#if defined(__PX4_LINUX)
	struct termios2 t;
	ioctl(fd, TCGETS2, &t);
	/* no parity, one stop bit */

	t.c_cflag &= ~(CSTOPB | PARENB |CBAUD);
		t.c_cflag |= BOTHER | CREAD;
	t.c_ispeed = baudrate;
	t.c_ospeed = baudrate;

	return ioctl(fd, TCSETS2, &t);
#else

	struct termios t;

	/* no parity, one stop bit */
	tcgetattr(fd, &t);
	cfsetspeed(&t, baudrate);
	t.c_cflag &= ~(CSTOPB | PARENB);
	return tcsetattr(fd, TCSANOW, &t);
#endif

}

int RP_FCC::task_spawn(int argc, char *argv[])
{
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int ch;

	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;
		default:
			PX4_WARN("unrecognized flag");
			return PX4_ERROR;
		}
	}

	if (device_name && (access(device_name, R_OK | W_OK) == 0)) {
		int fd = open_serial_device(device_name);
		if (fd < 0) {
			PX4_ERR("Failed to open serial device %s", device_name);
			return PX4_ERROR;
		}
		RP_FCC *instance = new RP_FCC(fd);

		if (instance == nullptr) {
			PX4_ERR("Failed to allocate instance");
			return PX4_ERROR;
		}
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleNow();
		return PX4_OK;
	} else {
		if (device_name) {
			PX4_ERR("invalid device (-d) %s", device_name);

		} else {
			PX4_INFO("valid device required");
		}
	}

	return PX4_ERROR;
}

bool RP_FCC::updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	pthread_mutex_lock(&mutex);


	rp_fcc_output_s output_msg;
	output_msg.cmd = RP_FCC_OUTPUT;
	memcpy(output_msg.outputs, outputs, sizeof(uint16_t) * DSHOT_CHANNEL_NUM);

	uint8_t buffer[64] = {0};
	uint16_t len = minihdlc_serialize(buffer, (uint8_t *)&output_msg, sizeof(output_msg));
	int ret = write(uart_fd, buffer, len);
	if (ret < 0) {
		PX4_ERR("Failed to write outputs");
		return false;
	}

	pthread_mutex_unlock(&mutex);
	return true;
}

int RP_FCC::send_dshot_cmd(uint16_t cmd, int dshot_channel_mask)
{
	if (uart_fd < 0) {
		PX4_ERR("UART not initialized");
		return -1;
	}

	uint8_t buffer[64]={0};

	rp_fcc_dshot_cmd_s cmd_msg;
	cmd_msg.cmd = RP_FCC_DSHOT_CMD;
	cmd_msg.dshot_cmd = cmd & 0xFF; // DShot command (low byte)
	cmd_msg.channel_mask = dshot_channel_mask & 0xFF; // Channel mask (low byte)

	uint16_t len = minihdlc_serialize(buffer, (uint8_t *)&cmd_msg, sizeof(cmd_msg));
	return write(uart_fd, buffer, len);
}

void RP_FCC::Run()
{
	if(should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}
	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	_mixing_output.update();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		update_params();
	}

	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

int RP_FCC::custom_command(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	int motor_index = 0; // select motor index, default: 0
	int cmd = 0; // command to send, default: 0
	int repeat_cnt = 1; // number of times to repeat the command, default: 1

	while ((ch = px4_getopt(argc, argv, "m:c:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm':
			motor_index = strtol(myoptarg, nullptr, 0);
			if (motor_index < 0 || motor_index >= DSHOT_CHANNEL_NUM) {
				return print_usage("motor index must be between 0 and DSHOT_CHANNEL_NUM");
			}
			break;
		case 'c':
			cmd = strtol(myoptarg, nullptr, 0);
			if (cmd < 0 || cmd > 47) {
				return print_usage("command must be between 0 and 47");
			}
			break;
		case 'n':
			repeat_cnt = strtol(myoptarg, nullptr, 0);
			if (repeat_cnt < 1 || repeat_cnt > 10) {
				return print_usage("repeat count must be between 1 and 10");
			}
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	PX4_INFO("get cmd :%d for motor:%d repeat:%d", cmd, motor_index,repeat_cnt);
	if(is_running()){
		RP_FCC *instance = _object.load();

		if (instance) {
			int ret = 0;
			pthread_mutex_lock(&instance->mutex);
			for(int i = 0; i < repeat_cnt; ++i) {
				ret = instance->send_dshot_cmd(cmd, 1 << motor_index);
				usleep(200);
			}
			usleep(260000); // wait for 260ms to ensure the command is sent and processed
			pthread_mutex_unlock(&instance->mutex);
			if (!ret) {
				PX4_ERR("Failed to send DShot command");
				return PX4_ERROR;
			}
			PX4_INFO("DShot command sent successfully");
		} else {
			PX4_ERR("Instance not found");
			return PX4_ERROR;
		}
	} else {
		PX4_ERR("RP_FCC is not running");
		return PX4_ERROR;
	}
	return PX4_OK;
}

void RP_FCC::update_params()
{
	// Update parameters if needed
	// This could involve reading from a parameter store or similar
	// For now, we just log that this function was called
	PX4_INFO("Updating parameters");
}

int RP_FCC::print_status()
{
	PX4_INFO("RP_FCC running with %u outputs", DSHOT_CHANNEL_NUM);
	return PX4_OK;
}

int RP_FCC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for driving the output pins.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rp_fcc", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rp_fcc_main(int argc, char *argv[])
{
	return RP_FCC::main(argc, argv);
}
