/*
 *    driver, char device interface
 *
 *    Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#include "acamera_types.h"
#include "acamera_logger.h"
#include "acamera_math.h"
#include "system_hw_io.h"
#include "acamera_firmware_config.h"
#include "acamera_control_config.h"
#include "acamera_command_api.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#endif

#define HEADER_SIZE 12

enum TransactionType {
	TransactionTypeRegRead = 1,
	TransactionTypeRegWrite,
	TransactionTypeRegMaskWrite,
	TransactionTypeLUTRead,
	TransactionTypeLUTWrite,

	TransactionTypeAPIRead = 10,
	TransactionTypeAPIWrite,

	TransactionTypeBUFRead = 20,
	TransactionTypeBUFWrite
};

void process_ioctl_buf(uint32_t *buf)
{
	uint8_t port = 0;
	uint32_t *rx_buf = (uint32_t *)buf;
	uint32_t *tx_buf = (uint32_t *)buf;
	uint16_t type = (*(rx_buf + 2)) & 0xFFFF;
	switch (type) {
	case TransactionTypeRegRead:
		if (buf[0] >= HEADER_SIZE + 8) {
		uint32_t addr = rx_buf[3];
		uint32_t size = rx_buf[4];
		if (size <= CONNECTION_BUFFER_SIZE - HEADER_SIZE - 4) {
			tx_buf[0] = HEADER_SIZE + 4 + size;
			tx_buf[3] = SUCCESS;
			uint8_t *b = (uint8_t *)&buf[5];

			while (size--) {
				//*b++ = read_32(addr++); //system_sw_read_8(addr++);
			}
		} else {
			tx_buf[0] = HEADER_SIZE + 4;
			tx_buf[3] = FAIL;
			LOG(LOG_WARNING, "Wrong request size %u for type %u", (unsigned int)size, (unsigned int)type);
		}
	} else {
		tx_buf[0] = HEADER_SIZE;
		LOG(LOG_WARNING, "Wrong packet size %u for type %u", (unsigned int)buf[0], (unsigned int)type);
	}
	break;
	case TransactionTypeRegWrite:
		if (buf[0] >= HEADER_SIZE + 8) {
			uint32_t addr = rx_buf[3];
			uint32_t size = rx_buf[4];
			buf[0] = HEADER_SIZE + 4;
			if (size <= buf[0] - HEADER_SIZE - 8) {
				tx_buf[3] = SUCCESS;
				uint8_t *b = (uint8_t *)&buf[5];
				while (size--) {
					//write_32(addr++, *b++, 0xFF);
				}
			} else {
				tx_buf[3] = FAIL;
				LOG(LOG_WARNING, "Wrong request size %u for type %u", (unsigned int)size, (unsigned int)type);
			}
		} else {
			buf[0] = HEADER_SIZE;
			LOG(LOG_WARNING, "Wrong packet size %u for type %u", (unsigned int)buf[0], (unsigned int)type);
	}
	break;
	case TransactionTypeRegMaskWrite:
		if (buf[0] >= HEADER_SIZE + 8) {
			uint32_t addr = rx_buf[3];
			uint32_t size = rx_buf[4];
			buf[0] = HEADER_SIZE + 4;
			if (2 * size <= buf[0] - HEADER_SIZE - 8) {
				tx_buf[3] = SUCCESS;
				uint8_t *b = (uint8_t *)&buf[5];
				uint8_t *m = (uint8_t *)&buf[5 + size];
				while (size--) {
					uint8_t mask = *m++;
					uint8_t val = *b++;
					//write_32(addr, val, mask);
					addr++;
				}
			} else {
				tx_buf[3] = FAIL;
				LOG(LOG_WARNING, "Wrong request size %u for type %u", size, type);
			}
		} else {
			buf[0] = HEADER_SIZE;
			LOG(LOG_WARNING, "Wrong packet size %u for type %u", buf[0], type);
		}
	break;
	case TransactionTypeLUTRead:
		if (buf[0] == HEADER_SIZE + 8) {
			uint32_t addr = rx_buf[3];
			uint32_t size = rx_buf[4];
			if (size <= CONNECTION_BUFFER_SIZE - HEADER_SIZE - 4 && !(size & 3) && !(addr & 3)) {
				buf[0] = HEADER_SIZE + 4 + size;
				tx_buf[3] = SUCCESS;
				uint32_t *b = &tx_buf[5];
				uint32_t i;
				for (i = 0; i < size; i += 4) {
					system_hw_write_32(addr, i >> 2);
					*b++ = system_hw_read_32(addr + 8);
				}
			} else {
				buf[0] = HEADER_SIZE + 4;
				tx_buf[3] = FAIL;
				LOG(LOG_WARNING, "Wrong request size %u for type %u", size, type);
			}
		} else {
			buf[0] = HEADER_SIZE;
			LOG(LOG_WARNING, "Wrong packet size %u for type %u", buf[0], type);
		}
	break;
	case TransactionTypeLUTWrite:
		if (buf[0] >= HEADER_SIZE + 8) {
			uint32_t addr = rx_buf[3];
			uint32_t size = rx_buf[4];
			port = (rx_buf[2] >> 16) & 0xFF;
			buf[0] = HEADER_SIZE + 4;
			if (size <= buf[0] - HEADER_SIZE - 8 && !(size & 3) && !(addr & 3)) {
				tx_buf[3] = SUCCESS;
				uint32_t i;
				for (i = 0; i < size; i += 4) {
					system_hw_write_32(addr, i >> 2);
					system_hw_write_32(addr + 4, rx_buf[i+5]);
				}
			} else {
				tx_buf[3] = FAIL;
				LOG(LOG_WARNING, "Wrong request size %u for type %u", size, type);
			}
		} else {
			buf[0] = HEADER_SIZE;
			LOG(LOG_WARNING, "Wrong packet size %u for type %u", buf[0], type);
		}
	break;
	case TransactionTypeAPIRead:
		if (buf[0] == HEADER_SIZE + 8) {
			uint8_t t = rx_buf[3] & 0xFF;
			uint8_t c = (rx_buf[3] >> 8) & 0xFF;
			port = (rx_buf[2] >> 16) & 0xFF;
			buf[0] = HEADER_SIZE + 8;
			tx_buf[3] = acamera_command(port, t, c, rx_buf[4], COMMAND_GET, &tx_buf[4]);
		} else {
			buf[0] = HEADER_SIZE;
			LOG(LOG_WARNING, "Wrong packet size %u for type %u", buf[0], type);
		}
	break;
	case TransactionTypeAPIWrite:
		if (buf[0] == HEADER_SIZE + 8) {
			uint8_t t = rx_buf[3] & 0xFF;
			uint8_t c = (rx_buf[3] >> 8) & 0xFF;
			port = (rx_buf[2] >> 16) & 0xFF;
			buf[0] = HEADER_SIZE + 8;
			LOG(LOG_DEBUG, "port %u,t %u, c %u, data %u, return %u", port, t, c, rx_buf[4], tx_buf[3]);
		} else {
			buf[0] = HEADER_SIZE;
			LOG(LOG_WARNING, "Wrong packet size %u for type %u", buf[0], type);
		}
	break;
	case TransactionTypeBUFRead:
		if (buf[0] == HEADER_SIZE + 8) {
			uint8_t id = rx_buf[3] & 0xFF;
			uint8_t buf_class = (rx_buf[3] >> 8) & 0xFF;
			port = (rx_buf[2] >> 16) & 0xFF;
			uint32_t size = rx_buf[4];
			uint32_t value;
			if (size <= CONNECTION_BUFFER_SIZE - HEADER_SIZE - 4) {
				switch (buf_class) {
				case STATIC_CALIBRATIONS_ID:
				case DYNAMIC_CALIBRATIONS_ID:
					buf[0] = HEADER_SIZE + 4 + size;
					tx_buf[3] = acamera_api_calibration(port,
						buf_class, id, COMMAND_GET, &tx_buf[5], size, &value);
					if (tx_buf[3] != SUCCESS) {
						buf[0] = HEADER_SIZE + 4;
					}
				break;
				default:
					buf[0] = HEADER_SIZE + 4;
					tx_buf[3] = FAIL;
					LOG(LOG_WARNING, "Wrong buffer class %u", buf_class);
				}
			} else {
				buf[0] = HEADER_SIZE + 4;
				tx_buf[3] = FAIL;
				LOG(LOG_WARNING, "Wrong request size %u for type %u", size, type);
			}
		} else {
			buf[0] = HEADER_SIZE;
			LOG(LOG_WARNING, "Wrong packet size %u for type %u", buf[0], type);
		}
	break;
	case TransactionTypeBUFWrite:
		if (buf[0] >= HEADER_SIZE + 8) {
			uint8_t id = rx_buf[3] & 0xFF;
			uint8_t buf_class = (rx_buf[3] >> 8) & 0xFF;
			port = (rx_buf[2] >> 16) & 0xFF;
			uint32_t size = rx_buf[4];
			uint32_t value;
			buf[0] = HEADER_SIZE + 4;
			if (size <= buf[0] - HEADER_SIZE - 8) {
				switch (buf_class) {
				case STATIC_CALIBRATIONS_ID:
				case DYNAMIC_CALIBRATIONS_ID:
					tx_buf[3] = acamera_api_calibration(port, buf_class,
						id, COMMAND_SET, &rx_buf[5], size, &value);
				break;
				default:
					tx_buf[3] = FAIL;
					LOG(LOG_WARNING, "Wrong buffer class %d", buf_class);
				}
			} else {
				tx_buf[3] = FAIL;
				LOG(LOG_WARNING, "Wrong request size %u for type %u", size, type);
			}
		} else {
			buf[0] = HEADER_SIZE;
			LOG(LOG_WARNING, "Wrong packet size %u for type %u", buf[0], type);
		}
	break;
	default:
		buf[0] = HEADER_SIZE;
		LOG(LOG_WARNING, "Wrong packet type %d", type);
	}
}

