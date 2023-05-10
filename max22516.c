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
int max22516_write(struct max22516_dev *dev, uint8_t reg_addr,
		   uint8_t data)
{
	uint8_t buff[MAX22516_BUFF_SIZE_BYTES];

	buff[0] = reg_addr;
	buff[1] = data;

	return no_os_spi_write_and_read(dev->spi_desc, buff, MAX22516_BUFF_SIZE_BYTES);
}

/**
 * @brief Reads data from max22516 over SPI.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param data - Data read from the device.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int max22516_read(struct max22516_dev *dev, uint8_t reg_addr,
		  uint8_t *data)
{
	int ret;
	uint8_t buff[MAX22516_BUFF_SIZE_BYTES];

	buff[0] = MAX22516_SPI_READ_CMD | reg_addr;
	buff[1] = MAX22516_SPI_DUMMY_DATA;

	ret = no_os_spi_write_and_read(dev->spi_desc, buff, MAX22516_BUFF_SIZE_BYTES);
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
int max22516_update(struct max22516_dev *dev, uint8_t reg_addr,
		    uint8_t mask, uint8_t data)
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