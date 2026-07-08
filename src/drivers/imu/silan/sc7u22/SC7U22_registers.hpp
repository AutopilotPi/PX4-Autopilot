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

#pragma once

#include <stdint.h>

namespace Silan_SC7U22
{

static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint8_t DIR_READ = 0x80;
static constexpr uint8_t WHOAMI = 0x6A;

static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000;
static constexpr uint32_t I2C_SPEED = 400 * 1000;
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x19;
static constexpr uint8_t I2C_ADDRESS_SDO_LOW = 0x18;

enum class Register : uint8_t {
	WHO_AM_I = 0x01,
	COM_CONF = 0x04,
	ACC_XH = 0x0C,
	ACC_XL = 0x0D,
	ACC_YH = 0x0E,
	ACC_YL = 0x0F,
	ACC_ZH = 0x10,
	ACC_ZL = 0x11,
	GYR_XH = 0x12,
	GYR_XL = 0x13,
	GYR_YH = 0x14,
	GYR_YL = 0x15,
	GYR_ZH = 0x16,
	GYR_ZL = 0x17,
	ACC_CONF = 0x40,
	ACC_RANGE = 0x41,
	GYR_CONF = 0x42,
	GYR_RANGE = 0x43,
	SOFT_RST = 0x4A,
	PWR_CTRL = 0x7D,
	SEG_SEL = 0x7F,
};

enum COM_CONF_BIT : uint8_t {
	BDU = Bit6,
	ADDR_AUTO = Bit4,
};

enum PWR_CTRL_BIT : uint8_t {
	TEMP_EN = Bit3,
	ACC_EN = Bit2,
	GYR_EN = Bit1,
};

static constexpr uint8_t ACC_FILTER_PERF = Bit7;
static constexpr uint8_t ACC_BWP_OSR4_AVG1 = (0x00 << 4);
static constexpr uint8_t ACC_ODR_1600 = 0x0C;
static constexpr uint8_t ACC_RANGE_16G = 0x03;

static constexpr uint8_t GYR_FILTER_PERF = Bit7;
static constexpr uint8_t GYR_BWP_OSR4_AVG1 = (0x00 << 4);
static constexpr uint8_t GYR_ODR_1600 = 0x0C;
static constexpr uint8_t GYR_RANGE_2000DPS = 0x00;

static constexpr uint8_t SOFT_RESET_VALUE = 0xA5;

#pragma pack(push, 1)
struct Data {
	uint8_t accel_x_msb;
	uint8_t accel_x_lsb;
	uint8_t accel_y_msb;
	uint8_t accel_y_lsb;
	uint8_t accel_z_msb;
	uint8_t accel_z_lsb;
	uint8_t gyro_x_msb;
	uint8_t gyro_x_lsb;
	uint8_t gyro_y_msb;
	uint8_t gyro_y_lsb;
	uint8_t gyro_z_msb;
	uint8_t gyro_z_lsb;
};
#pragma pack(pop)

static_assert(sizeof(Data) == 12);

} // namespace Silan_SC7U22
