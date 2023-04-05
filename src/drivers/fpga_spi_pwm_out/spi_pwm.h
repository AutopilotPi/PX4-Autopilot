#ifndef __SPI_PWM_H
#define __SPI_PWM_H

#include <stdint.h>

#define FPAG_OSC_FREQ	238996176
#define FPGA_FREQ  FPAG_OSC_FREQ/4
#define FPGA_DEFAULT_PERIOD	2000
#define PWM_DEFAULT_FREQUENCY FPGA_FREQ/FPGA_DEFAULT_PERIOD

#define FPGA_PWM_OUTPUT_MAX_CHANNELS 8
#define FPAG_PWM_SUBPWM_NUM 2

#define SUBPWM0_PERIOD_REG 0x0
#define CCR0_REG 0x1
#define CCR1_REG 0x2
#define CCR2_REG 0x3
#define CCR3_REG 0x4
#define CCR4_REG 0x5
#define CCR5_REG 0x6
#define CCR6_REG 0x7
#define CCR7_REG 0x8
#define PWM_CHANNEL_MAP0_REG 0x20
#define SUBPWM1_PERIOD_REG 0x40
#define CONFIG_REG 0x50
#define WATCHDOG_REG 0x7f
#define TIMEOUT_MAX_HIGH_REG 0x7e
#define TIMEOUT_MAX_LOW_REG 0x7d

/*
    the struct pwm_t below is useless, because we cannot access it by mmio.
    while i keep it here to show how the regs map.
*/
typedef struct {
    uint16_t subpwm0_period;
    uint16_t ccr0;
    uint16_t ccr1;
    uint16_t ccr2;
    uint16_t ccr3;
    uint16_t ccr4;
    uint16_t ccr5;
    uint16_t ccr6;
    uint16_t ccr7;
    uint16_t reserved1[4];
    uint32_t pwm_channel_map0;
    uint16_t reserved2[15];
    uint16_t subpwm1_period;
    uint16_t reserved3[15];
    uint16_t config;
    uint16_t reserved4[15];
    uint16_t watchdog;
    uint16_t timeout_max_high;
    uint16_t timeout_max_low;
} pwm_t;



#endif /* PWM_H */
