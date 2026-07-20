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
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>

#include <limits.h>

class SC7U22_IIO final : public ModuleBase<SC7U22_IIO>, public px4::ScheduledWorkItem
{
public:
	SC7U22_IIO(const char *device_path, enum Rotation rotation, uint8_t bus, uint8_t chip_select);
	~SC7U22_IIO() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int init();
	int print_status() override;

private:
	static constexpr uint32_t SAMPLE_RATE_HZ{1600};
	static constexpr uint32_t SAMPLE_INTERVAL_US{1000000 / SAMPLE_RATE_HZ};
	static constexpr uint8_t SAMPLES_PER_PUBLISH{2};
	static constexpr uint32_t PUBLISH_INTERVAL_US{SAMPLE_INTERVAL_US * SAMPLES_PER_PUBLISH};
	static constexpr size_t IIO_SCAN_SIZE{24};
	static constexpr size_t IIO_TIMESTAMP_OFFSET{16};
	static constexpr size_t MAX_READ_SCANS{64};

	void Run() override;
	bool configure_iio();
	void disable_iio();
	bool validate_scan_layout() const;
	bool process_scan(const uint8_t *scan);
	void publish_pending();
	void reset_pending();
	void update_error_count();

	static uint32_t make_device_id(uint8_t bus, uint8_t chip_select);
	static bool find_iio_device(char *device_path, size_t device_path_len);
	static bool make_sysfs_path(const char *device_path, char *sysfs_path, size_t sysfs_path_len);
	static bool write_text_file(const char *path, const char *value, bool quiet = false);
	static bool read_text_file(const char *path, char *value, size_t value_len, bool quiet = false);
	static int16_t read_le16(const uint8_t *data);
	static int64_t read_le64(const uint8_t *data);

	char _device_path[PATH_MAX] {};
	char _sysfs_path[PATH_MAX] {};
	int _fd{-1};
	bool _buffer_enabled{false};

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	sensor_accel_fifo_s _accel_fifo{};
	sensor_gyro_fifo_s _gyro_fifo{};
	uint64_t _last_timestamp_us{0};
	uint8_t _pending_samples{0};
	uint8_t _read_buffer[IIO_SCAN_SIZE * MAX_READ_SCANS] {};

	perf_counter_t _read_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": read")};
	perf_counter_t _publish_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": publish interval")};
	perf_counter_t _bad_read_perf{perf_alloc(PC_COUNT, MODULE_NAME ": bad read")};
	perf_counter_t _bad_scan_perf{perf_alloc(PC_COUNT, MODULE_NAME ": bad scan")};
	perf_counter_t _timestamp_gap_perf{perf_alloc(PC_COUNT, MODULE_NAME ": timestamp gap")};
	perf_counter_t _stale_sample_perf{perf_alloc(PC_COUNT, MODULE_NAME ": stale sample")};
};
