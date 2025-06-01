#ifndef __RP_FCC_PROTOCOL_H
#define __RP_FCC_PROTOCOL_H

#include "stdint.h"

#define DSHOT_CHANNEL_NUM 4
typedef enum
{
	RP_FCC_DSHOT_CMD,
	RP_FCC_OUTPUT
} rp_fcc_cmd_t;

typedef struct
{
	uint8_t cmd; // use uint8_t to identify the size fo enum
	uint16_t outputs[DSHOT_CHANNEL_NUM];
} __attribute__((packed)) rp_fcc_output_s;

typedef struct
{
	uint8_t cmd;
	uint8_t dshot_cmd;
	uint8_t channel_mask; // Bitmask of channels (LSB = channel 0) to enable.
} __attribute__((packed)) rp_fcc_dshot_cmd_s;

#endif
