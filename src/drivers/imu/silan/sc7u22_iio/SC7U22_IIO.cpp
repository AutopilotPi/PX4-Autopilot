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

#include "SC7U22_IIO.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/getopt.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

using namespace time_literals;

namespace
{
constexpr const char *IIO_SYSFS_ROOT = "/sys/bus/iio/devices";
constexpr const char *EXPECTED_IIO_NAME = "sc7u22";

struct ScanElement {
	const char *name;
	const char *type;
	int index;
};

constexpr ScanElement scan_elements[] {
	{"in_accel_x",   "le:s16/16>>0", 0},
	{"in_accel_y",   "le:s16/16>>0", 1},
	{"in_accel_z",   "le:s16/16>>0", 2},
	{"in_anglvel_x", "le:s16/16>>0", 3},
	{"in_anglvel_y", "le:s16/16>>0", 4},
	{"in_anglvel_z", "le:s16/16>>0", 5},
	{"in_timestamp", "le:s64/64>>0", 6},
};

void trim_newline(char *text)
{
	const size_t length = strlen(text);

	if (length > 0 && text[length - 1] == '\n') {
		text[length - 1] = '\0';
	}
}
} // namespace

uint32_t SC7U22_IIO::make_device_id(uint8_t bus, uint8_t chip_select)
{
	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType_SPI;
	id.devid_s.bus = bus;
	id.devid_s.address = chip_select;
	id.devid_s.devtype = DRV_IMU_DEVTYPE_SC7U22;
	return id.devid;
}

SC7U22_IIO::SC7U22_IIO(const char *device_path, enum Rotation rotation, uint8_t bus, uint8_t chip_select) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_px4_accel(make_device_id(bus, chip_select), rotation),
	_px4_gyro(make_device_id(bus, chip_select), rotation)
{
	if (device_path != nullptr) {
		strncpy(_device_path, device_path, sizeof(_device_path) - 1);
	}

	_px4_accel.set_device_type(DRV_IMU_DEVTYPE_SC7U22);
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
	_px4_accel.set_scale((16.f * CONSTANTS_ONE_G) / 32768.f);
	_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_SC7U22);
	_px4_gyro.set_range(math::radians(2000.f));
	_px4_gyro.set_scale(math::radians(2000.f) / 32768.f);
}

SC7U22_IIO::~SC7U22_IIO()
{
	ScheduleClear();
	disable_iio();

	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}

	perf_free(_read_perf);
	perf_free(_publish_interval_perf);
	perf_free(_bad_read_perf);
	perf_free(_bad_scan_perf);
	perf_free(_timestamp_gap_perf);
	perf_free(_stale_sample_perf);
}

bool SC7U22_IIO::write_text_file(const char *path, const char *value, bool quiet)
{
	const int fd = ::open(path, O_WRONLY | O_CLOEXEC);

	if (fd < 0) {
		if (!quiet) {
			PX4_ERR("open %s failed: %s", path, strerror(errno));
		}

		return false;
	}

	const size_t length = strlen(value);
	const ssize_t written = ::write(fd, value, length);
	const int saved_errno = errno;
	::close(fd);

	if (written != static_cast<ssize_t>(length)) {
		if (!quiet) {
			PX4_ERR("write %s failed: %s", path, strerror(saved_errno));
		}

		return false;
	}

	return true;
}

bool SC7U22_IIO::read_text_file(const char *path, char *value, size_t value_len, bool quiet)
{
	if (value_len == 0) {
		return false;
	}

	const int fd = ::open(path, O_RDONLY | O_CLOEXEC);

	if (fd < 0) {
		if (!quiet) {
			PX4_ERR("open %s failed: %s", path, strerror(errno));
		}

		return false;
	}

	const ssize_t length = ::read(fd, value, value_len - 1);
	const int saved_errno = errno;
	::close(fd);

	if (length <= 0) {
		if (!quiet) {
			PX4_ERR("read %s failed: %s", path, strerror(saved_errno));
		}

		return false;
	}

	value[length] = '\0';
	trim_newline(value);
	return true;
}

bool SC7U22_IIO::find_iio_device(char *device_path, size_t device_path_len)
{
	DIR *directory = opendir(IIO_SYSFS_ROOT);

	if (directory == nullptr) {
		PX4_ERR("open %s failed: %s", IIO_SYSFS_ROOT, strerror(errno));
		return false;
	}

	bool found = false;
	struct dirent *entry = nullptr;

	while ((entry = readdir(directory)) != nullptr) {
		if (strncmp(entry->d_name, "iio:device", strlen("iio:device")) != 0) {
			continue;
		}

		char name_path[PATH_MAX];
		char name[64];
		snprintf(name_path, sizeof(name_path), "%s/%s/name", IIO_SYSFS_ROOT, entry->d_name);

		if (read_text_file(name_path, name, sizeof(name), true) && strcmp(name, EXPECTED_IIO_NAME) == 0) {
			snprintf(device_path, device_path_len, "/dev/%s", entry->d_name);
			found = true;
			break;
		}
	}

	closedir(directory);
	return found;
}

bool SC7U22_IIO::make_sysfs_path(const char *device_path, char *sysfs_path, size_t sysfs_path_len)
{
	const char *device_name = strrchr(device_path, '/');
	device_name = (device_name != nullptr) ? device_name + 1 : device_path;

	if (strncmp(device_name, "iio:device", strlen("iio:device")) != 0) {
		PX4_ERR("invalid IIO device path %s", device_path);
		return false;
	}

	const int length = snprintf(sysfs_path, sysfs_path_len, "%s/%s", IIO_SYSFS_ROOT, device_name);
	return length > 0 && static_cast<size_t>(length) < sysfs_path_len;
}

bool SC7U22_IIO::validate_scan_layout() const
{
	char path[PATH_MAX];
	char value[64] {};

	for (const ScanElement &element : scan_elements) {
		snprintf(path, sizeof(path), "%s/scan_elements/%s_type", _sysfs_path, element.name);

		if (!read_text_file(path, value, sizeof(value)) || strcmp(value, element.type) != 0) {
			PX4_ERR("unexpected %s type '%s'", element.name, value);
			return false;
		}

		snprintf(path, sizeof(path), "%s/scan_elements/%s_index", _sysfs_path, element.name);

		if (!read_text_file(path, value, sizeof(value)) || atoi(value) != element.index) {
			PX4_ERR("unexpected %s index '%s'", element.name, value);
			return false;
		}
	}

	return true;
}

bool SC7U22_IIO::configure_iio()
{
	char path[PATH_MAX];
	char value[64];

	if (_device_path[0] == '\0' && !find_iio_device(_device_path, sizeof(_device_path))) {
		PX4_ERR("SC7U22 IIO device not found");
		return false;
	}

	if (!make_sysfs_path(_device_path, _sysfs_path, sizeof(_sysfs_path))) {
		return false;
	}

	snprintf(path, sizeof(path), "%s/name", _sysfs_path);

	if (!read_text_file(path, value, sizeof(value)) || strcmp(value, EXPECTED_IIO_NAME) != 0) {
		PX4_ERR("%s is not an SC7U22 IIO device", _device_path);
		return false;
	}

	/* Clock must match PX4's CLOCK_MONOTONIC hrt_absolute_time(). */
	snprintf(path, sizeof(path), "%s/buffer/enable", _sysfs_path);
	write_text_file(path, "0", true);
	snprintf(path, sizeof(path), "%s/current_timestamp_clock", _sysfs_path);

	if (!write_text_file(path, "monotonic")) {
		return false;
	}

	if (!read_text_file(path, value, sizeof(value)) || strcmp(value, "monotonic") != 0) {
		PX4_ERR("IIO timestamp clock is '%s', expected monotonic", value);
		return false;
	}

	if (!validate_scan_layout()) {
		return false;
	}

	for (const ScanElement &element : scan_elements) {
		snprintf(path, sizeof(path), "%s/scan_elements/%s_en", _sysfs_path, element.name);

		if (!write_text_file(path, "1")) {
			return false;
		}
	}

	snprintf(path, sizeof(path), "%s/buffer/length", _sysfs_path);

	if (!write_text_file(path, "256")) {
		return false;
	}

	/* Wake readers only after the two scans consumed by one 800 Hz PX4 update. */
	snprintf(path, sizeof(path), "%s/buffer/watermark", _sysfs_path);
	write_text_file(path, "2", true);

	_fd = ::open(_device_path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);

	if (_fd < 0) {
		PX4_ERR("open %s failed: %s", _device_path, strerror(errno));
		return false;
	}

	snprintf(path, sizeof(path), "%s/buffer/enable", _sysfs_path);

	if (!write_text_file(path, "1")) {
		::close(_fd);
		_fd = -1;
		return false;
	}

	_buffer_enabled = true;
	return true;
}

void SC7U22_IIO::disable_iio()
{
	if (_buffer_enabled && _sysfs_path[0] != '\0') {
		char path[PATH_MAX];
		snprintf(path, sizeof(path), "%s/buffer/enable", _sysfs_path);
		write_text_file(path, "0", true);
		_buffer_enabled = false;
	}
}

int SC7U22_IIO::init()
{
	if (!configure_iio()) {
		disable_iio();
		return PX4_ERROR;
	}

	PX4_INFO("using %s (24-byte scan, monotonic timestamp)", _device_path);
	ScheduleOnInterval(PUBLISH_INTERVAL_US, PUBLISH_INTERVAL_US);
	return PX4_OK;
}

int16_t SC7U22_IIO::read_le16(const uint8_t *data)
{
	return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
				    (static_cast<uint16_t>(data[1]) << 8));
}

int64_t SC7U22_IIO::read_le64(const uint8_t *data)
{
	uint64_t value = 0;

	for (unsigned i = 0; i < sizeof(value); i++) {
		value |= static_cast<uint64_t>(data[i]) << (8 * i);
	}

	return static_cast<int64_t>(value);
}

void SC7U22_IIO::reset_pending()
{
	_pending_samples = 0;
	_accel_fifo = {};
	_gyro_fifo = {};
}

void SC7U22_IIO::update_error_count()
{
	const uint64_t errors = perf_event_count(_bad_read_perf) + perf_event_count(_bad_scan_perf)
				+ perf_event_count(_timestamp_gap_perf) + perf_event_count(_stale_sample_perf);
	_px4_accel.set_error_count(errors);
	_px4_gyro.set_error_count(errors);
}

void SC7U22_IIO::publish_pending()
{
	_accel_fifo.timestamp_sample = _last_timestamp_us;
	_gyro_fifo.timestamp_sample = _last_timestamp_us;
	_accel_fifo.dt = SAMPLE_INTERVAL_US;
	_gyro_fifo.dt = SAMPLE_INTERVAL_US;
	_accel_fifo.samples = _pending_samples;
	_gyro_fifo.samples = _pending_samples;
	update_error_count();
	_px4_accel.updateFIFO(_accel_fifo);
	_px4_gyro.updateFIFO(_gyro_fifo);
	perf_count(_publish_interval_perf);
	reset_pending();
}

bool SC7U22_IIO::process_scan(const uint8_t *scan)
{
	const int64_t timestamp_ns = read_le64(scan + IIO_TIMESTAMP_OFFSET);

	if (timestamp_ns <= 0) {
		perf_count(_bad_scan_perf);
		reset_pending();
		return false;
	}

	const uint64_t timestamp_us = static_cast<uint64_t>(timestamp_ns / 1000);
	const hrt_abstime now = hrt_absolute_time();

	/* Reject a mismatched clock domain and samples stalled for an excessive time. */
	if (timestamp_us > now + 5_ms || now > timestamp_us + 1_s) {
		perf_count(_stale_sample_perf);
		reset_pending();
		_last_timestamp_us = timestamp_us;
		return false;
	}

	if (_last_timestamp_us != 0) {
		const int64_t delta_us = static_cast<int64_t>(timestamp_us) - static_cast<int64_t>(_last_timestamp_us);

		if (delta_us < static_cast<int64_t>(SAMPLE_INTERVAL_US - 100)
		    || delta_us > static_cast<int64_t>(SAMPLE_INTERVAL_US + 100)) {
			perf_count(_timestamp_gap_perf);
			reset_pending();
		}
	}

	if (_pending_samples >= SAMPLES_PER_PUBLISH) {
		perf_count(_bad_scan_perf);
		reset_pending();
	}

	const uint8_t sample = _pending_samples;
	_accel_fifo.x[sample] = read_le16(scan + 0);
	_accel_fifo.y[sample] = read_le16(scan + 2);
	_accel_fifo.z[sample] = read_le16(scan + 4);
	_gyro_fifo.x[sample] = read_le16(scan + 6);
	_gyro_fifo.y[sample] = read_le16(scan + 8);
	_gyro_fifo.z[sample] = read_le16(scan + 10);
	_pending_samples++;
	_last_timestamp_us = timestamp_us;

	if (_pending_samples == SAMPLES_PER_PUBLISH) {
		publish_pending();
	}

	return true;
}

void SC7U22_IIO::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_read_perf);

	for (unsigned iteration = 0; iteration < 8; iteration++) {
		const ssize_t bytes = ::read(_fd, _read_buffer, sizeof(_read_buffer));

		if (bytes < 0) {
			if (errno != EAGAIN) {
				perf_count(_bad_read_perf);
			}

			break;
		}

		if (bytes == 0) {
			break;
		}

		if ((bytes % IIO_SCAN_SIZE) != 0) {
			perf_count(_bad_read_perf);
			break;
		}

		const size_t scans = static_cast<size_t>(bytes) / IIO_SCAN_SIZE;

		for (size_t i = 0; i < scans; i++) {
			process_scan(&_read_buffer[i * IIO_SCAN_SIZE]);
		}

		if (static_cast<size_t>(bytes) < sizeof(_read_buffer)) {
			break;
		}
	}

	perf_end(_read_perf);
}

int SC7U22_IIO::print_status()
{
	PX4_INFO("device: %s", _device_path);
	PX4_INFO("sysfs: %s", _sysfs_path);
	PX4_INFO("buffer enabled: %s, pending scans: %u", _buffer_enabled ? "yes" : "no", _pending_samples);
	PX4_INFO("last sample timestamp: %" PRIu64 " us", _last_timestamp_us);
	perf_print_counter(_read_perf);
	perf_print_counter(_publish_interval_perf);
	perf_print_counter(_bad_read_perf);
	perf_print_counter(_bad_scan_perf);
	perf_print_counter(_timestamp_gap_perf);
	perf_print_counter(_stale_sample_perf);
	return 0;
}

int SC7U22_IIO::task_spawn(int argc, char *argv[])
{
	int myoptind = 1;
	const char *myoptarg = nullptr;
	const char *device_path = nullptr;
	int rotation = ROTATION_NONE;
	int bus = 0;
	int chip_select = 0;
	int ch;

	while ((ch = px4_getopt(argc, argv, "d:R:b:c:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		case 'R':
			rotation = atoi(myoptarg);
			break;

		case 'b':
			bus = atoi(myoptarg);
			break;

		case 'c':
			chip_select = atoi(myoptarg);
			break;

		default:
			return print_usage("unrecognized option");
		}
	}

	if (rotation < 0 || rotation > 35 || bus < 0 || bus > 31 || chip_select < 0 || chip_select > 255) {
		return print_usage("invalid rotation, bus, or chip-select");
	}

	SC7U22_IIO *instance = new SC7U22_IIO(device_path, static_cast<enum Rotation>(rotation),
						static_cast<uint8_t>(bus), static_cast<uint8_t>(chip_select));

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	return PX4_OK;
}

int SC7U22_IIO::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SC7U22_IIO::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
PX4 userspace adapter for the Linux SC7U22 IIO kernel driver. The adapter
configures the IIO scan buffer for six-axis data plus a monotonic timestamp,
then publishes two 1600 Hz scans per 800 Hz PX4 FIFO update.

The Linux kernel driver must own the SPI device. Do not start the legacy
`sc7u22` spidev driver at the same time.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("sc7u22_iio", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "IIO device path (auto-detect if omitted)", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, 31, "SPI bus number used in PX4 device ID", true);
	PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, 255, "SPI chip-select used in PX4 device ID", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int sc7u22_iio_main(int argc, char *argv[])
{
	return SC7U22_IIO::main(argc, argv);
}
