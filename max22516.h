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
#define REG_CHIP_ID		0X00
#define REG_REV_ID		0X01
#define REG_IOL_STAT		0X02
#define REG_DEV_STAT1		0X03
#define REG_DEV_STAT2		0X04
#define REG_ISDU_STAT		0X05
#define REG_IOL_ERR_CNT		0X06
#define REG_FRM_ERR_CNT		0X07
#define REG_IOL_INT		0X08
#define REG_DEV_INT		0X09
#define REG_ISDU_INT		0X0A
#define REG_IOL_INT_EN		0X0E
#define REG_DEV_INT_EN		0X0F
#define REG_ISDU_INT_EN		0X10
#define REG_IOL_CFG		0X14
#define REG_WATCHDOG		0X15
#define REG_WDGCLR		0X16
#define REG_MISC_CFG		0X17
#define REG_CLK_CFG		0X18
#define REG_CLK_TRIM		0X19
#define REG_PG1_MSTCMD		0X1A
#define REG_PG1_MSTCYCTM	0X1B
#define REG_PG1_MINCYCTM	0X1C
#define REG_PG1_MSEQCAP		0X1D
#define REG_PG1_REVID		0X1E
#define REG_PG1_PDIN		0X1F
#define REG_PG1_PDOUT		0X20
#define REG_PG1_VID1		0X21
#define REG_PG1_VID2		0X22
#define REG_PG1_DEVID1		0X23
#define REG_PG1_DEVID2		0X24
#define REG_PG1_DEVID3		0X25
#define REG_PG1_FUNCID1		0X26
#define REG_PG1_FUNCID2		0X27
#define REG_PG1_RES1		0X28
#define REG_PG1_RES2		0X29
#define REG_WDG_EVENT		0X2A
#define REG_STATUS_CODE_DEF	0X2B
#define REG_STATUS_CODE		0X2C
#define REG_EVENT_QUAL		0X2D
#define REG_EVENT_CODE_MSB	0X2E
#define REG_EVENT_CODE_LSB	0X2F
#define REG_EVENT_FLAG		0X30
#define REG_PDIN_FIFO		0X35
#define REG_PDIN_DATA_RDY	0X36
#define REG_PDOUT_FIFO		0X37
#define REG_ISDU_OFFSET		0X3F
#define REG_ISDU_INFIFO		0X40
#define REG_ISDU_DATARDY	0X41
#define REG_ISDU_OUTFIFO	0X42
#define REG_ISDU_LEVEL		0X43
#define REG_LED1_CTRL_MSB	0X50
#define REG_LED1_CTRL_LSB	0X51
#define REG_LED2_CTRL_MSB	0X52
#define REG_LED2_CTRL_LSB	0X53
#define REG_GPIO1_CTRL		0X54
#define REG_GPIO2_CTRL		0X55
#define REG_CQ_CTRL1		0X56
#define REG_CQ_CTRL2		0X57
#define REG_DO_CTRL1		0X58
#define REG_DO_CTRL2		0X59
#define REG_TX_CTRL		0X5A
#define REG_RX_CTRL		0X5B
#define REG_MISC_CTRL		0X5C

/* REG_DEV_STAT2 */
#define DEV_STAT2_SET_DOFAULT       0x20
#define DEV_STAT2_SET_CQFAULT       0x10
#define DEV_STAT2_SET_V24ERR        0x08
#define DEV_STAT2_SET_VMErr         0x04
#define DEV_STAT2_SET_THWARN        0x02
#define DEV_STAT2_SET_TSHD          0x01

/* REG_CQ_CTRL1 */
#define BIT_CQCTRL1_CQ_SLEW0	0x00
#define BIT_CQCTRL1_CQ_SLEW1	0x40
#define BIT_CQCTRL1_CQ_SLEW2	0x80
#define BIT_CQCTRL1_CQ_SLEW3	0xC0
#define BIT_CQCTRL1_CQ_PD	0x20
#define BIT_CQCTRL1_CQ_PU	0x10
#define BIT_CQCTRL1_CQ_NPN	0x08
#define BIT_CQCTRL1_CQ_PP	0x04
#define BIT_CQCTRL1_CQ_INV	0x02
#define BIT_CQCTRL1_CQ_EN	0x01

/* REG_CQ_CTRL2 */
#define BIT_CQ_CL_50MA			0x00
#define BIT_CQ_CL_100MA			0x40
#define BIT_CQ_CL_200MA			0x80
#define BIT_CQ_CL_250MA			0xC0
#define BIT_CQ_CLBL_128US		0x00
#define BIT_CQ_CLBL_500US		0x08
#define BIT_CQ_CLBL_1000US		0x10
#define BIT_CQ_CLBL_5000US		0x18
#define BIT_CQ_AUTORTY_TIME_50MS	0x00
#define BIT_CQ_AUTORTY_TIME_100MS	0x02
#define BIT_CQ_AUTORTY_TIME_200MS	0x04
#define BIT_CQ_AUTORTY_TIME_500MS	0x06
#define BIT_CQ_AUTORTY			0x01

/* REG_DO_CTRL1 */
#define BIT_DOCTRL1_DO_SLEW0		0x00
#define BIT_DOCTRL1_DO_SLEW1		0x40
#define BIT_DOCTRL1_DO_SLEW2		0x80
#define BIT_DOCTRL1_DO_SLEW3		0xC0
#define BIT_DOCTRL1_DO_PD		0x20
#define BIT_DOCTRL1_DO_PU		0x10
#define BIT_DOCTRL1_DO_NPN		0x08
#define BIT_DOCTRL1_DO_PP		0x04
#define BIT_DOCTRL1_DO_INV		0x02
#define BIT_DOCTRL1_DO_EN		0x01

/* REG_DO_CTRL2 */
#define BIT_DO_CL_50MA			0x00
#define BIT_DO_CL_100MA			0x40
#define BIT_DO_CL_200MA			0x80
#define BIT_DO_CL_250MA			0xC0
#define BIT_DO_CLBL_128US		0x00
#define BIT_DO_CLBL_500US		0x08
#define BIT_DO_CLBL_1000US		0x10
#define BIT_DO_CLBL_5000US		0x18
#define BIT_DO_AUTORTY_TIME_50MS	0x00
#define BIT_DO_AUTORTY_TIME_100MS	0x02
#define BIT_DO_AUTORTY_TIME_200MS	0x04
#define BIT_DO_AUTORTY_TIME_500MS	0x06
#define BIT_DO_AUTORTY			0x01

/* REG_TX_CTRL */
#define BIT_TXC_CQTX			0x80
#define BIT_TXC_CQTXEN			0x40
#define BIT_TXC_CQDRVSEL		0x20
#define BIT_TXC_DOTX			0x10
#define BIT_TXC_DODRVSEL		0x08
#define BIT_TXC_CQDOPAR			0x02
#define BIT_TXC_DO_AV			0x01
	
/* REG_DEV_STAT2 */
#define DEV_STAT2_SET_V24ERR		0x08
#define DEV_STAT2_SET_VMERR		0x04
#define DEV_STAT2_SET_THWARN		0x02
#define DEV_STAT2_SET_TSHD		0x01

/* REG_EVENT_FLAG */
#define EVF_EVENT_FLG			0x01

/* MAX22516 EVENT CODE Masks */
#define REG_EVENT_CODE_MSB_MSK		NO_OS_GENMASK(15, 8)
#define REG_EVENT_CODE_LSB_MSK		NO_OS_GENMASK(7, 0)

/* MAX22516 VID Masks*/
#define PG1_VID1_MSK			NO_OS_GENMASK(15, 8)
#define PG1_VID2_MSK			NO_OS_GENMASK(7, 0)

/* MAX22516 DEVID Masks*/
#define PG1_DEVID1_MSK			NO_OS_GENMASK(23, 16)
#define PG1_DEVID2_MSK			NO_OS_GENMASK(15, 8)
#define PG1_DEVID3_MSK			NO_OS_GENMASK(7, 0)

/* MAX22516 FUNCID1 Masks */
#define PG1_FUNCID1_MSB_MSK		NO_OS_GENMASK(15, 8)
#define PG1_FUNCID1_LSB_MSK		NO_OS_GENMASK(7, 0)

/* MAX22516 LED1_CTRL Masks */
#define REG_LED1_CTRL_MSB_MSK		NO_OS_GENMASK(15, 8)
#define REG_LED1_CTRL_LSB_MSK		NO_OS_GENMASK(7, 0)

/* MAX22516 LED2_CTRL Masks */
#define REG_LED2_CTRL_MSB_MSK		NO_OS_GENMASK(15, 8)
#define REG_LED2_CTRL_LSB_MSK		NO_OS_GENMASK(7, 0)

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

int max22516_build_tcyc(struct max22516_dev *dev, int16_t t, uint8_t *tmr);

int max22516_set_min_ctmr(struct max22516_dev *dev, uint16_t min_t);

/* MAX22516 Initialization */
int max22516_init(struct max22516_dev **device,
		  struct max22516_init_param *init_param);

/** MAX22516 Resources Deallocation */
int max22516_remove(struct max22516_dev *dev);

#endif /* MAX22516_H_ */