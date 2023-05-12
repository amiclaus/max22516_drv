/***************************************************************************//**
 *   @file   max22516.c
 *   @brief  Implementation of MAX22516 Driver.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdlib.h>
#include "max22516.h"
#include "no_os_delay.h"
#include "no_os_error.h"
#include "no_os_alloc.h"

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/**
 * @brief Writes data to max22516 over SPI.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param data - Data value to write.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int max22516_write(struct max22516_dev *dev, uint8_t reg_addr, uint8_t data)
{
	uint8_t buff[MAX22516_BUFF_SIZE_BYTES];

	buff[0] = reg_addr;
	buff[1] = data;

	return no_os_spi_write_and_read(dev->spi_desc, buff,
					MAX22516_BUFF_SIZE_BYTES);
}

/**
 * @brief Reads data from max22516 over SPI.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param data - Data read from the device.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int max22516_read(struct max22516_dev *dev, uint8_t reg_addr, uint8_t *data)
{
	int ret;
	uint8_t buff[MAX22516_BUFF_SIZE_BYTES];

	buff[0] = MAX22516_SPI_READ_CMD | reg_addr;
	buff[1] = MAX22516_SPI_DUMMY_DATA;

	ret = no_os_spi_write_and_read(dev->spi_desc, buff,
				       MAX22516_BUFF_SIZE_BYTES);
	if (ret)
		return ret;

	*data = buff[1];

	return ret;
}

/**
 * @brief Update MAX22516 register.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - Mask for specific register bits to be updated.
 * @param data - Data read from the device.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int max22516_update(struct max22516_dev *dev, uint8_t reg_addr, uint8_t mask,
		    uint8_t data)
{
	uint8_t read_val;
	int ret;

	ret = max22516_read(dev, reg_addr, &read_val);
	if (ret)
		return ret;

	read_val &= ~mask;
	read_val |= data;

	return max22516_write(dev, reg_addr, read_val);
}

/**
 * @brief MAX22516 burst write.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param count - Nr. of bytes to be written.
 * @param data - Data to be written.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int max22516_burst_write_register(struct max22516_dev *dev, uint8_t reg_addr,
				  uint8_t count, uint8_t *data)
{
	uint8_t buff[count+1];
	int i;

	buff[0] = reg_addr;

	for (i = 0; i < count; i++)
		buff[1 + i] = data[0 + i];

	return no_os_spi_write_and_read(dev->spi_desc, buff, count + 1);
}

/**
 * @brief MAX22516 burst read.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param count - Nr. of bytes to be read.
 * @param data - Data read from the device.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int max22516_burst_read_register(struct max22516_dev *dev, uint8_t reg_addr,
				 uint8_t count, uint8_t *data)
{
	uint8_t buff[count+1];
	int i, ret;

	buff[0] = MAX22516_SPI_READ_CMD | reg_addr;

	ret = no_os_spi_write_and_read(dev->spi_desc, buff, count + 1);
	if (ret)
		return ret;

	for (i = 0; i < count; i++)
		data[0 + i] = buff[1 + i];

	return 0;
}

void max22516_build_tcyc(int16_t t, uint8_t *tmr)
{
	uint8_t temp = 0;

	if (t <= 64) { // in 100µs. -> if less than 6.4ms
		temp = t;
	} else if (t <= 316) { // in 100µs. -> if less than 31.6ms
		temp = (t-64) / 4; // offset of 6.4ms and now in 400µs steps
		temp |= 0x40;
	} else if (t < 1328) { // in 100µs. -> if less than 132.8ms
		temp = (t-320) / 16; // offset of 32ms and now in 1.6ms steps
		temp |= 0x80;
	}

	*tmr = temp;
}

void max22516_rebuild_min_cyct_to_us(int16_t t, uint8_t *tmr)
{
	uint8_t temp = 0;

	if ((t & 0xc0) == 0x00) {      // then time is stored in 100µs increments
		temp = t * 100;
	} else if ((t & 0xc0) == 0x40) { // then time is stored in 400µs increments
		temp = 6400 + ((t&0x3f) * 400);
	} else if ((t & 0xc0) == 0x80) { // then time is stored in 1.6ms increments
		temp = 32000 + ((t&0x3f) * 1600);
	}

	*tmr = temp;
}

int max22516_set_min_ctmr(struct max22516_dev *dev,
			  uint16_t min_t) //set min cyc time, in 100us
{
	uint8_t tmr;

	max22516_build_tcyc(min_t, &tmr);

	return max22516_write(dev, REG_PG1_MINCYCTM, tmr);
}

int max22516_set_id(struct max22516_dev *dev, uint16_t vid, uint32_t id,
		    uint16_t fid)
{
	int ret;

	ret = max22516_write(dev, REG_PG1_VID1,
			     no_os_field_prep(PG1_VID1_MSK, vid));
	if (ret)
		return ret;

	ret = max22516_write(dev, REG_PG1_VID2,
			     no_os_field_prep(PG1_VID2_MSK, vid));
	if (ret)
		return ret;

	ret = max22516_write(dev, REG_PG1_DEVID1,
			     no_os_field_prep(PG1_DEVID1_MSK, id));
	if (ret)
		return ret;

	ret = max22516_write(dev, REG_PG1_DEVID2,
			     no_os_field_prep(PG1_DEVID2_MSK, id));
	if (ret)
		return ret;

	ret = max22516_write(dev, REG_PG1_DEVID3,
			     no_os_field_prep(PG1_DEVID3_MSK, id));
	if (ret)
		return ret;

	ret = max22516_write(dev, REG_PG1_FUNCID1,
			     no_os_field_prep(PG1_FUNCID1_MSB_MSK, fid));
	if (ret)
		return ret;

	return max22516_write(dev, REG_PG1_FUNCID1,
			      no_os_field_prep(PG1_FUNCID1_LSB_MSK, fid));
}

void max22516_decode_tcyc(uint8_t tmr, int16_t *t)
{
	uint8_t base = (tmr >> 6);
	int16_t temp = (tmr & 0x3F);

	if (base == 1)
		temp <<= 2;
	else if (base == 2)
		temp <<= 4;
	else
		temp = (0x3F << 4);

	*t = temp;
}

int max22516_get_mst_ctmr(struct max22516_dev *dev, uint16_t min_t,
			  int16_t *c_tmr)  //get master min cyc time, in 100us
{
	int ret;
	uint8_t tmr;

	ret = max22516_read(dev, REG_PG1_MSTCYCTM, &tmr);
	if (ret)
		return ret;

	max22516_decode_tcyc(tmr, c_tmr);

	return 0;
}

int max22516_get_dl_mode(struct max22516_dev *dev, uint8_t *mode)
{
	return max22516_read(dev, REG_PG1_MSTCYCTM, mode);
}

int max22516_get_iol_err_cnt(struct max22516_dev *dev, uint8_t *cnt)
{
	return max22516_read(dev, REG_IOL_ERR_CNT, cnt);
}

int max22516_get_frm_err_cnt(struct max22516_dev *dev, uint8_t *cnt)
{
	return max22516_read(dev, REG_FRM_ERR_CNT, cnt);
}

int max22516_clr_iol_err_cnt(struct max22516_dev *dev)
{
	return max22516_write(dev, REG_IOL_ERR_CNT, 0);
}

int max22516_clr_frm_err_cnt(struct max22516_dev *dev)
{
	return max22516_write(dev, REG_FRM_ERR_CNT, 0);
}

int max22516_set_led1(struct max22516_dev *dev, uint16_t ltmr)
{
	int ret;

	ret = max22516_write(dev, REG_LED1_CTRL_MSB,
			     no_os_field_prep(REG_LED1_CTRL_MSB_MSK, ltmr));
	if (ret)
		return ret;

	return max22516_write(dev, REG_LED1_CTRL_LSB,
			      no_os_field_prep(REG_LED1_CTRL_LSB_MSK, ltmr));
}

int max22516_set_led2(struct max22516_dev *dev, uint16_t ltmr)
{
	int ret;

	ret = max22516_write(dev, REG_LED2_CTRL_MSB,
			     no_os_field_prep(REG_LED2_CTRL_MSB_MSK, ltmr));
	if (ret)
		return ret;

	return max22516_write(dev, REG_LED2_CTRL_LSB,
			      no_os_field_prep(REG_LED2_CTRL_LSB_MSK, ltmr));
}

int max22516_get_v24(struct max22516_dev *dev, uint8_t *status3)
{
	uint8_t temp;
	int ret;

	ret = max22516_read(dev, REG_DEV_STAT2, &temp);
	if (ret)
		return ret;

	if (temp & DEV_STAT2_SET_V24ERR)
		*status3 = 2;
	else if (temp & DEV_STAT2_SET_VMERR)
		*status3 = 1;
	else
		*status3 = 0;

	return 0;
}

int max22516_get_thd(struct max22516_dev *dev, uint8_t *status3)
{
	uint8_t temp;
	int ret;

	ret = max22516_read(dev, REG_DEV_STAT2, &temp);
	if (ret)
		return ret;

	if (temp & DEV_STAT2_SET_TSHD)
		*status3 = 2;
	else if (temp & DEV_STAT2_SET_THWARN)
		*status3 = 1;
	else
		*status3 = 0;

	return 0;
}

int max22516_setup_cq_dis(struct max22516_dev *dev)
{
	return max22516_write(dev, REG_CQ_CTRL1, BIT_CQCTRL1_CQ_PD);
}

int max22516_setup_cq_pp(struct max22516_dev *dev)
{
	int ret;
	ret = max22516_write(dev, REG_CQ_CTRL1, BIT_CQCTRL1_CQ_EN |
			     BIT_CQCTRL1_CQ_PP);
	if (ret)
		return ret;

	return max22516_write(dev, REG_CQ_CTRL2, BIT_CQ_AUTORTY |
			      BIT_CQ_AUTORTY_TIME_200MS |
			      BIT_CQ_CLBL_500US |
			      BIT_CQ_CL_200MA);
}

int max22516_setup_cq_pnp(struct max22516_dev *dev)
{
	int ret;
	ret = max22516_write(dev, REG_CQ_CTRL1, BIT_CQCTRL1_CQ_EN);
	if (ret)
		return ret;

	return max22516_write(dev, REG_CQ_CTRL2, BIT_CQ_AUTORTY |
			      BIT_CQ_AUTORTY_TIME_200MS |
			      BIT_CQ_CLBL_500US |
			      BIT_CQ_CL_200MA);
}

int max22516_setup_cq_npn(struct max22516_dev *dev)
{
	int ret;
	ret = max22516_write(dev, REG_CQ_CTRL1, BIT_CQCTRL1_CQ_EN);
	if (ret)
		return ret;

	return max22516_write(dev, REG_CQ_CTRL2, BIT_CQ_AUTORTY |
			      BIT_CQ_AUTORTY_TIME_200MS |
			      BIT_CQ_CLBL_500US |
			      BIT_CQ_CL_200MA);
}

int max22516_tx_set(struct max22516_dev *dev, uint8_t low_high)
{
	uint8_t tx_ctrl;
	int ret;

	ret = max22516_read(dev, REG_TX_CTRL, &tx_ctrl);
	if (ret)
		return ret;

	if (low_high == 0)
		tx_ctrl &= (~BIT_TXC_CQTX); // set TX to low, keep other bits
	else
		tx_ctrl |= BIT_TXC_CQTX; // set TX to high, keep other bits


	tx_ctrl |=
		BIT_TXC_CQDRVSEL;  // Make sure the register-bits are used (rather then pins)

	return max22516_write(dev, REG_TX_CTRL, tx_ctrl);
}

int max22516_txen_set(struct max22516_dev *dev, uint8_t lvl)
{
	uint8_t tx_ctrl;
	int ret;

	ret = max22516_read(dev, REG_TX_CTRL, &tx_ctrl);
	if (ret)
		return ret;

	if (lvl == 0)
		tx_ctrl &= (~BIT_TXC_CQTXEN); // set TXEN to low, keep other bits
	else
		tx_ctrl |= BIT_TXC_CQTXEN; // set TXEN to high, keep other bits


	tx_ctrl |=
		BIT_TXC_CQDRVSEL;  // Make sure the register-bits are used (rather then pins)

	return max22516_write(dev, REG_TX_CTRL, tx_ctrl);
}

int max22516_set_cq(struct max22516_dev *dev,
		    uint8_t lvl) // set the level of CQ (0: CQ low, 1: CQ high, 2: CQ highZ)
{
	int ret;

	if (lvl < 2)
		ret = max22516_txen_set(dev, 1);
	else
		ret = max22516_txen_set(dev, 0);

	if (ret)
		return ret;

	if(lvl == 0)
		return max22516_tx_set(dev, 1);
	else
		return max22516_tx_set(dev, 0);
}

int max22516_rx_get(struct max22516_dev *dev)
{
	return 0;
}

int max22516_get_cq(struct max22516_dev *dev, uint8_t *cq)
{
	if (max22516_rx_get(dev) == 1)
		*cq = 0;
	else
		*cq = 1;

	return 0;
};

int max22516_get_cq_stat(struct max22516_dev *dev, uint8_t *status3)
{
	uint8_t temp;
	int ret;

	ret = max22516_read(dev, REG_DEV_STAT2, &temp);
	if (ret)
		return ret;

	if (temp & DEV_STAT2_SET_CQFAULT)
		*status3 = 1;
	else
		*status3 = 0;

	return 0;
}

int max22516_setup_do_dis(struct max22516_dev *dev)
{
	return max22516_write(dev, REG_DO_CTRL1, BIT_DOCTRL1_DO_PD);
}

int max22516_setup_do_pp(struct max22516_dev *dev)
{
	int ret;

	ret = max22516_write(dev, REG_DO_CTRL1, BIT_DOCTRL1_DO_EN |
			     BIT_DOCTRL1_DO_PP);
	if (ret)
		return ret;

	return max22516_write(dev, REG_DO_CTRL2, BIT_DO_AUTORTY |
			      BIT_DO_AUTORTY_TIME_200MS |
			      BIT_DO_CLBL_500US |
			      BIT_DO_CL_200MA);
}

int max22516_setup_do_pnp(struct max22516_dev *dev)
{
	int ret;

	ret = max22516_write(dev, REG_DO_CTRL1, BIT_DOCTRL1_DO_EN);
	if (ret)
		return ret;

	return max22516_write(dev, REG_DO_CTRL2, BIT_DO_AUTORTY |
			      BIT_DO_AUTORTY_TIME_200MS |
			      BIT_DO_CLBL_500US |
			      BIT_DO_CL_200MA);
}

int max22516_setup_do_npn(struct max22516_dev *dev)
{
	int ret;

	ret = max22516_write(dev, REG_DO_CTRL1, BIT_DOCTRL1_DO_EN |
			     BIT_DOCTRL1_DO_NPN);
	if (ret)
		return ret;

	return max22516_write(dev, REG_DO_CTRL2, BIT_DO_AUTORTY |
			      BIT_DO_AUTORTY_TIME_200MS |
			      BIT_DO_CLBL_500US |
			      BIT_DO_CL_200MA);
}

int max22516_do_set(struct max22516_dev *dev, uint8_t lvl)
{
	uint8_t tx_ctrl;
	int ret;

	ret = max22516_read(dev, REG_TX_CTRL, &tx_ctrl);
	if (ret)
		return ret;

	if (lvl == 0)
		tx_ctrl &= (~BIT_TXC_DOTX); // set TX to low, keep other bits
	else
		tx_ctrl |= BIT_TXC_DOTX; // set TX to high, keep other bits

	return max22516_write(dev, REG_TX_CTRL, tx_ctrl);
}

int max22516_do_get(struct max22516_dev *dev, uint8_t *lvl)
{
	return 1;
}

int max22516_get_do_stat(struct max22516_dev *dev, uint8_t *status3)
{
	uint8_t temp;
	int ret;

	ret = max22516_read(dev, REG_DEV_STAT2, &temp);
	if (ret)
		return ret;

	if (temp & DEV_STAT2_SET_DOFAULT)
		*status3 = 1;
	else
		*status3 = 0;

	return 0;
}

int max22516_set_event(struct max22516_dev *dev, uint8_t ev_qual,
		       uint16_t ev_code)
{
	int ret;

	ret = max22516_write(dev, REG_STATUS_CODE, 0x81);
	if (ret)
		return ret;

	ret = max22516_write(dev, REG_EVENT_QUAL, ev_qual);
	if (ret)
		return ret;

	ret = max22516_write(dev, REG_EVENT_CODE_MSB,
			     no_os_field_prep(REG_EVENT_CODE_MSB_MSK, ev_code));
	if (ret)
		return ret;

	ret = max22516_write(dev, REG_EVENT_CODE_LSB,
			     no_os_field_prep(REG_EVENT_CODE_LSB_MSK, ev_code));
	if (ret)
		return ret;

	return max22516_write(dev, REG_EVENT_FLAG, EVF_EVENT_FLG);
}

int max22516_setup_watchdog(struct max22516_dev *dev, uint8_t wd_timeout,
			    uint8_t wd_clr, uint8_t wd_event_en,
			    uint8_t wd_event_flag)
{
	int ret;
	uint8_t event_flag;

	if (wd_clr == 0)
		ret = max22516_write(dev, REG_WDGCLR, wd_clr);
	if (wd_clr == 1)
		ret = max22516_write(dev, REG_WDGCLR, wd_clr);

	if (ret)
		return ret;

	ret = max22516_read(dev, REG_EVENT_FLAG, &event_flag);
	if (ret)
		return ret;

	if (wd_event_en == 1) {
		ret = max22516_write(dev, REG_EVENT_FLAG, event_flag | 0x02);
		if (ret)
			return ret;

		return max22516_write(dev, REG_WDG_EVENT, wd_event_flag);
	}

	if (wd_event_en == 0) {
		ret = max22516_write(dev, REG_EVENT_FLAG, event_flag & 0xfd);
		if (ret)
			return ret;

		return max22516_write(dev, REG_WDG_EVENT, wd_event_flag);
	}

	return 0;
}

/**
 * @brief Initialize the MAX22516 device.
 * @param device - The device structure.
 * @param init_param - The structure containing the device initial parameters.
 * @return Returns 0 in case of success or negative error code.
 */
int max22516_init(struct max22516_dev **device,
		  struct max22516_init_param *init_param)
{
	int ret;
	struct max22516_dev *dev;

	dev = (struct max22516_dev *)no_os_calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	ret = no_os_spi_init(&dev->spi_desc, init_param->spi_init);
	if (ret)
		goto error_dev;

	*device = dev;

	return 0;

error_dev:
	free(dev);

	return ret;
}

/**
 * @brief Free resoulces allocated for MAX22516
 * @param dev - The device structure.
 * @return Returns 0 in case of success or negative error code.
 */
int max22516_remove(struct max22516_dev *dev)
{
	int ret;

	ret = no_os_spi_remove(dev->spi_desc);
	if (ret)
		return ret;

	free(dev);

	return ret;
}