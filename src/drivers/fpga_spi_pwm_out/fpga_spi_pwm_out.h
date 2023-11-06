/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
#include <cstdint>
#include <drivers/device/spi.h>
#include "spi_pwm.h"
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/cdev/CDev.hpp>

class FPGA_SPI_PWM_Wrapper;

namespace fpga_spi_pwm
{

class FPGA_SPI_PWM :public device::SPI
{
public:
	FPGA_SPI_PWM(int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency);

	int init() override{return device::SPI::init();};
	int setFreq(uint8_t sub_pwm_n,float freq);
	void setEXTPWM(uint16_t * bitmasks);
	void setPredivider(int predivider){_predivide = predivider;};

	~FPGA_SPI_PWM() override = default;

	/*
	 * disable all of the output
	 */
	void disableAllOutput();

protected:

	void setPeriod(uint8_t sub_pwm_n,uint16_t value);

	int writeReg(uint8_t reg_addr,uint16_t val){
		uint8_t buf[3]={(uint8_t)(reg_addr << 1 | 1), (uint8_t)(val>>8) ,(uint8_t)(val&0xff)};
		return transfer(buf,nullptr,3);
	}
	uint16_t readReg(uint8_t reg_addr){
		uint8_t buf[3]={(uint8_t)(reg_addr << 1 | 0)};
		transfer(buf,buf,3);
		return buf[1] <<8 | buf[2];
	}

private:
	friend class FPGA_SPI_PWM_Wrapper;
	uint8_t _channel_mode_map[FPGA_PWM_OUTPUT_MAX_CHANNELS];
	int _predivide = 0;
	float _freqs[FPAG_PWM_SUBPWM_NUM];
	uint16_t _periods[FPAG_PWM_SUBPWM_NUM];
	uint16_t _period_regs[FPAG_PWM_SUBPWM_NUM] = { SUBPWM0_PERIOD_REG, SUBPWM1_PERIOD_REG};

};

}
