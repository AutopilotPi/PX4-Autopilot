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

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

SC7U22::~SC7U22()
{
	perf_free(_sample_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);

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

	ScheduleOnInterval(_fifo_read_interval_us, _fifo_read_interval_us);
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

	// Store filtered accel and gyro samples at the full 1.6 kHz ODR, without headers.
	RegisterWrite(Register::FIFO_DOWNS, FIFO_DOWNS_FILTERED_NO_DOWNSAMPLE);
	RegisterWrite(Register::FIFO_CFG0, FIFO_CFG0_BIT::FIFO_ACC_EN | FIFO_CFG0_BIT::FIFO_GYR_EN);
	RegisterWrite(Register::FIFO_CFG2, _fifo_watermark_words & 0xFF);
	RegisterWrite(Register::FIFO_CFG1,
		      FIFO_MODE_STREAM | ((_fifo_watermark_words >> 8) & FIFO_CFG1_THRESHOLD_HIGH_MASK));

	return RegisterRead(Register::WHO_AM_I) == WHOAMI;
}

void SC7U22::ConfigureSampleRate(int sample_rate)
{
	const float requested_interval_us = 1e6f / math::max(sample_rate, 1);
	const uint32_t fifo_samples = math::constrain(static_cast<uint32_t>(roundf(requested_interval_us / FIFO_SAMPLE_DT_US)),
				      1u, static_cast<uint32_t>(FIFO_MAX_SAMPLES));

	_fifo_read_interval_us = fifo_samples * FIFO_SAMPLE_DT_US;
	_fifo_watermark_words = fifo_samples * FIFO_WORDS_PER_SAMPLE;
}

void SC7U22::RunImpl()
{
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	uint8_t fifo_status = 0;
	const uint16_t fifo_words = FIFOReadCount(fifo_status);

	if (fifo_status & FIFO_STAT0_BIT::FIFO_OVERFLOW) {
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		perf_cancel(_sample_perf);
		return;
	}

	const uint16_t complete_samples = fifo_words / FIFO_WORDS_PER_SAMPLE;

	if (complete_samples == 0) {
		perf_count(_fifo_empty_perf);
		perf_cancel(_sample_perf);
		return;
	}

	if (complete_samples > FIFO_MAX_SAMPLES) {
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		perf_cancel(_sample_perf);
		return;
	}

	if (!FIFORead(timestamp_sample, complete_samples)) {
		perf_cancel(_sample_perf);
		return;
	}

	// The datasheet requires cycling through bypass after each complete FIFO read.
	FIFOReset();

	perf_end(_sample_perf);
}

void SC7U22::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
}

uint16_t SC7U22::FIFOReadCount(uint8_t &status)
{
	uint8_t fifo_status[2] {};

	if (_interface->read(static_cast<uint8_t>(Register::FIFO_STAT0), fifo_status, sizeof(fifo_status)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		status = 0;
		return 0;
	}

	status = fifo_status[0];
	return ((fifo_status[0] & FIFO_STAT0_COUNT_HIGH_MASK) << 8) | fifo_status[1];
}

bool SC7U22::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOData data[FIFO_MAX_SAMPLES] {};
	const size_t transfer_size = samples * sizeof(FIFOData);

	if (_interface->read(static_cast<uint8_t>(Register::FIFO_DATA), data, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		FIFOReset();
		return false;
	}

	sensor_accel_fifo_s accel{};
	sensor_gyro_fifo_s gyro{};
	accel.timestamp_sample = timestamp_sample;
	gyro.timestamp_sample = timestamp_sample;
	accel.dt = FIFO_SAMPLE_DT_US;
	gyro.dt = FIFO_SAMPLE_DT_US;
	accel.samples = samples;
	gyro.samples = samples;

	for (uint8_t i = 0; i < samples; i++) {
		accel.x[i] = Combine(data[i].accel_x_msb, data[i].accel_x_lsb);
		accel.y[i] = Combine(data[i].accel_y_msb, data[i].accel_y_lsb);
		accel.z[i] = Combine(data[i].accel_z_msb, data[i].accel_z_lsb);
		gyro.x[i] = Combine(data[i].gyro_x_msb, data[i].gyro_x_lsb);
		gyro.y[i] = Combine(data[i].gyro_y_msb, data[i].gyro_y_lsb);
		gyro.z[i] = Combine(data[i].gyro_z_msb, data[i].gyro_z_lsb);
	}

	const uint64_t error_count = perf_event_count(_bad_transfer_perf) + perf_event_count(_fifo_overflow_perf);
	_px4_accel.set_error_count(error_count);
	_px4_gyro.set_error_count(error_count);
	_px4_accel.updateFIFO(accel);
	_px4_gyro.updateFIFO(gyro);
	return true;
}

void SC7U22::FIFOReset()
{
	perf_count(_fifo_reset_perf);
	RegisterWrite(Register::FIFO_CFG1, FIFO_MODE_BYPASS);
	RegisterWrite(Register::FIFO_CFG1,
		      FIFO_MODE_STREAM | ((_fifo_watermark_words >> 8) & FIFO_CFG1_THRESHOLD_HIGH_MASK));
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
