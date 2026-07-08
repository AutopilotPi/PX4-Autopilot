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

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace sc7u22
{
device::Device *SC7U22_SPI_interface(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
#if defined(CONFIG_I2C)
device::Device *SC7U22_I2C_interface(uint8_t bus, uint32_t address, int bus_frequency);
#endif // CONFIG_I2C
}

void SC7U22::print_usage()
{
	PRINT_MODULE_USAGE_NAME("sc7u22", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
#if defined(CONFIG_I2C)
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, true);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(Silan_SC7U22::I2C_ADDRESS_DEFAULT);
#else
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
#endif
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *SC7U22::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	device::Device *interface = nullptr;

#if defined(CONFIG_I2C)

	if (config.bus_type == BOARD_I2C_BUS) {
		interface = sc7u22::SC7U22_I2C_interface(config.bus, config.i2c_address, config.bus_frequency);

	} else
#endif // CONFIG_I2C
		if (config.bus_type == BOARD_SPI_BUS) {
			interface = sc7u22::SC7U22_SPI_interface(config.bus, config.spi_devid, config.bus_frequency, config.spi_mode);
		}

	if (interface == nullptr) {
		PX4_ERR("failed creating interface for bus %i", config.bus);
		return nullptr;
	}

	if (interface->init() != PX4_OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i", config.bus);
		return nullptr;
	}

	SC7U22 *dev = new SC7U22(config, interface);

	if (dev == nullptr) {
		delete interface;
		return nullptr;
	}

	if (dev->init() != PX4_OK) {
		delete dev;
		return nullptr;
	}

	return dev;
}

extern "C" int sc7u22_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = SC7U22;

#if defined(CONFIG_I2C)
	BusCLIArguments cli{true, true};
	cli.i2c_address = Silan_SC7U22::I2C_ADDRESS_DEFAULT;
	cli.default_i2c_frequency = Silan_SC7U22::I2C_SPEED;
#else
	BusCLIArguments cli{false, true};
#endif // CONFIG_I2C

	cli.default_spi_frequency = Silan_SC7U22::SPI_SPEED;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = static_cast<enum Rotation>(atoi(cli.optArg()));
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_IMU_DEVTYPE_SC7U22);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
