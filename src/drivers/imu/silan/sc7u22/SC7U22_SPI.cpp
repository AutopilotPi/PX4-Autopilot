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

#include "SC7U22_registers.hpp"

#include <lib/drivers/device/spi.h>

#if defined(CONFIG_SPI)

#include <string.h>

namespace sc7u22
{

device::Device *SC7U22_SPI_interface(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);

class SC7U22_SPI : public device::SPI
{
public:
	SC7U22_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
	~SC7U22_SPI() override = default;

	int read(unsigned address, void *data, unsigned count) override;
	int write(unsigned address, void *data, unsigned count) override;
};

device::Device *SC7U22_SPI_interface(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode)
{
	return new SC7U22_SPI(bus, device, bus_frequency, spi_mode);
}

SC7U22_SPI::SC7U22_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_IMU_DEVTYPE_SC7U22, MODULE_NAME, bus, device, spi_mode, bus_frequency)
{
}

int SC7U22_SPI::read(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32] {};

	if (count + 1 > sizeof(buf)) {
		return -EIO;
	}

	buf[0] = address | Silan_SC7U22::DIR_READ;

	const int ret = transfer(buf, buf, count + 1);

	if (ret == PX4_OK) {
		memcpy(data, &buf[1], count);
	}

	return ret;
}

int SC7U22_SPI::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (count + 1 > sizeof(buf)) {
		return -EIO;
	}

	buf[0] = address & ~Silan_SC7U22::DIR_READ;
	memcpy(&buf[1], data, count);

	return transfer(buf, nullptr, count + 1);
}

} // namespace sc7u22

#endif // CONFIG_SPI
