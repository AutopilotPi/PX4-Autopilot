/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "SC7U22.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/time.h>

using namespace Silan_SC7U22;
using namespace time_literals;

SC7U22::SC7U22(const I2CSPIDriverConfig &config, device::Device *interface) :
	I2CSPIDriver(config),
	_interface(interface),
	_px4_accel(interface->get_device_id(), config.rotation),
	_px4_gyro(interface->get_device_id(), config.rotation)
{
	_px4_accel.set_device_type(DRV_IMU_DEVTYPE_SC7U22);
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
	_px4_accel.set_scale((16.f * CONSTANTS_ONE_G) / 32768.f);

	_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_SC7U22);
	_px4_gyro.set_range(math::radians(2000.f));
	_px4_gyro.set_scale(math::radians(2000.f) / 32768.f);
}

SC7U22::~SC7U22()
{
	perf_free(_sample_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);

	delete _interface;
}

int SC7U22::init()
{
	for (int attempt = 0; attempt < 5; attempt++) {
		RegisterWrite(Register::SEG_SEL, 0x00);

		if (RegisterRead(Register::WHO_AM_I) == WHOAMI) {
			break;
		}

		if (attempt == 4) {
			PX4_DEBUG("unexpected WHO_AM_I 0x%02x", RegisterRead(Register::WHO_AM_I));
			return PX4_ERROR;
		}

		px4_usleep(10_ms);
	}

	if (!Configure()) {
		return PX4_ERROR;
	}

	ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
	return PX4_OK;
}

bool SC7U22::Configure()
{
	if (RegisterWrite(Register::SEG_SEL, 0x00) != PX4_OK) {
		return false;
	}

	RegisterWrite(Register::COM_CONF, COM_CONF_BIT::BDU | COM_CONF_BIT::ADDR_AUTO);
	RegisterWrite(Register::SOFT_RST, SOFT_RESET_VALUE);
	px4_usleep(1_ms);
	RegisterWrite(Register::SOFT_RST, SOFT_RESET_VALUE);
	px4_usleep(200_ms);

	RegisterWrite(Register::SEG_SEL, 0x00);
	RegisterWrite(Register::COM_CONF, COM_CONF_BIT::BDU | COM_CONF_BIT::ADDR_AUTO);
	RegisterWrite(Register::PWR_CTRL, 0x00);
	px4_usleep(1_ms);

	RegisterWrite(Register::ACC_RANGE, ACC_RANGE_16G);
	RegisterWrite(Register::GYR_RANGE, GYR_RANGE_2000DPS);
	RegisterWrite(Register::ACC_CONF, ACC_FILTER_PERF | ACC_BWP_OSR4_AVG1 | ACC_ODR_1600);
	RegisterWrite(Register::GYR_CONF, GYR_FILTER_PERF | GYR_BWP_OSR4_AVG1 | GYR_ODR_1600);
	px4_usleep(2_ms);

	RegisterWrite(Register::PWR_CTRL, PWR_CTRL_BIT::TEMP_EN | PWR_CTRL_BIT::ACC_EN | PWR_CTRL_BIT::GYR_EN);
	px4_usleep(60_ms);

	return RegisterRead(Register::WHO_AM_I) == WHOAMI;
}

void SC7U22::RunImpl()
{
	perf_begin(_sample_perf);

	Data data{};
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (_interface->read(static_cast<uint8_t>(Register::ACC_XH), &data, sizeof(data)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		perf_cancel(_sample_perf);
		return;
	}

	const int16_t accel_x = Combine(data.accel_x_msb, data.accel_x_lsb);
	const int16_t accel_y = Combine(data.accel_y_msb, data.accel_y_lsb);
	const int16_t accel_z = Combine(data.accel_z_msb, data.accel_z_lsb);
	const int16_t gyro_x = Combine(data.gyro_x_msb, data.gyro_x_lsb);
	const int16_t gyro_y = Combine(data.gyro_y_msb, data.gyro_y_lsb);
	const int16_t gyro_z = Combine(data.gyro_z_msb, data.gyro_z_lsb);

	_px4_accel.set_error_count(perf_event_count(_bad_transfer_perf));
	_px4_accel.update(timestamp_sample, accel_x, accel_y, accel_z);

	_px4_gyro.set_error_count(perf_event_count(_bad_transfer_perf));
	_px4_gyro.update(timestamp_sample, gyro_x, gyro_y, gyro_z);

	perf_end(_sample_perf);
}

void SC7U22::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

uint8_t SC7U22::RegisterRead(Register reg)
{
	uint8_t value = 0;

	if (_interface->read(static_cast<uint8_t>(reg), &value, 1) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return value;
}

int SC7U22::RegisterWrite(Register reg, uint8_t value)
{
	if (_interface->write(static_cast<uint8_t>(reg), &value, 1) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return PX4_ERROR;
	}

	return PX4_OK;
}
