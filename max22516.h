/***************************************************************************//**
 *   @file   max22516.h
 *   @brief  Header file for max22516 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef MAX22516_H_
#define MAX22516_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define REG_CHIP_ID		0x00
#define REG_REV_ID		0x01
#define REG_IOL_STAT		0x02
#define REG_DEV_STAT1		0x03
#define REG_DEV_STAT2		0x04
#define REG_ISDU_STAT		0x05
#define REG_IOL_ERR_CNT		0x06
#define REG_FRM_ERR_CNT		0x07
#define REG_IOL_INT		0x08
#define REG_DEV_INT		0x09
#define REG_ISDU_INT		0x0A
#define REG_IOL_INT_EN		0x0E
#define REG_DEV_INT_EN		0x0F
#define REG_ISDU_INT_EN		0x10
#define REG_IOL_CFG		0x14
#define REG_Watchdog		0x15
#define REG_WDGclr		0x16
#define REG_MISC_CFG		0x17
#define REG_CLK_CFG		0x18
#define REG_CLK_TRIM		0x19
#define REG_PG1_MstCmd		0x1A
#define REG_PG1_MstCycTm	0x1B
#define REG_PG1_MinCycTm	0x1C
#define REG_PG1_MseqCap		0x1D
#define REG_PG1_RevID		0x1E
#define REG_PG1_PDIN		0x1F
#define REG_PG1_PDOUT		0x20
#define REG_PG1_VID1		0x21
#define REG_PG1_VID2		0x22
#define REG_PG1_DevID1		0x23
#define REG_PG1_DevID2		0x24
#define REG_PG1_DevID3		0x25
#define REG_PG1_FuncID1		0x26
#define REG_PG1_FuncID2		0x27
#define REG_PG1_res1		0x28
#define REG_PG1_res2		0x29
#define REG_WDG_EVENT		0x2A
#define REG_STATUS_CODE_Def	0x2B
#define REG_STATUS_CODE		0x2C
#define REG_EVENT_Qual		0x2D
#define REG_EVENT_Code_MSB	0x2E
#define REG_EVENT_Code_LSB	0x2F
#define REG_EVENT_Flag		0x30
#define REG_PDIN_FIFO		0x35
#define REG_PDIN_Data_Rdy	0x36
#define REG_PDOUT_FIFO		0x37
#define REG_ISDU_Offset		0x3F
#define REG_ISDU_InFifo		0x40
#define REG_ISDU_DataRdy	0x41
#define REG_ISDU_OutFifo	0x42
#define REG_ISDU_Level		0x43
#define REG_LED1_Ctrl_MSB	0x50
#define REG_LED1_Ctrl_LSB	0x51
#define REG_LED2_Ctrl_MSB	0x52
#define REG_LED2_Ctrl_LSB	0x53
#define REG_GPIO1_Ctrl		0x54
#define REG_GPIO2_Ctrl		0x55
#define REG_CQ_Ctrl1		0x56
#define REG_CQ_Ctrl2		0x57
#define REG_DO_Ctrl1		0x58
#define REG_DO_Ctrl2		0x59
#define REG_TX_Ctrl		0x5a
#define REG_RX_Ctrl		0x5b
#define REG_MISC_Ctrl		0x5c

/* MAX22516 Extra Definitions */
#define MAX22516_SPI_DUMMY_DATA		0x00
#define MAX22516_BUFF_SIZE_BYTES     	2
#define MAX22516_SPI_READ_CMD		NO_OS_BIT(7)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/**
 * @struct max22516_init_param
 * @brief MAX22516 Initialization Parameters structure.
 */
struct max22516_init_param {
	/* SPI Initialization parameters */
	struct no_os_spi_init_param	*spi_init;
};

/**
 * @struct max22516_dev
 * @brief MAX22516 Device Descriptor.
 */
struct max22516_dev {
	/* SPI Initialization parameters */
	struct no_os_spi_desc	*spi_desc;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/** MAX22516 SPI write */
int max22516_write(struct max22516_dev *dev, uint8_t reg_addr,
		   uint8_t data);

/** MAX22516 SPI Read */
int max22516_read(struct max22516_dev *dev, uint8_t reg_addr,
		  uint8_t *data);

/* MAX22516 Register Update */
int max22516_update(struct max22516_dev *dev, uint8_t reg_addr,
		    uint8_t mask, uint8_t data);

/* MAX22516 Burst Write */
int max22516_burst_write_register(struct max22516_dev *dev, uint8_t reg_addr,
				  uint8_t count, uint8_t *data);

/* MAX22516 Burst Read */
int max22516_burst_read_register(struct max22516_dev *dev, uint8_t reg_addr,
				 uint8_t count, uint8_t *data);

/* MAX22516 Initialization */
int max22516_init(struct max22516_dev **device,
		  struct max22516_init_param *init_param);

/** MAX22516 Resources Deallocation */
int max22516_remove(struct max22516_dev *dev);

#endif /* MAX22516_H_ */