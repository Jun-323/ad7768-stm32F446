

/***************************************************************************//**

 *   @file   gpio.c

 *   @author DBogdan (dragos.bogdan@analog.com)

********************************************************************************

 * Copyright 2019(c) Analog Devices, Inc.

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



//#include "error.h"

#include "gpio.h"

#define SUCCESS 0

/******************************************************************************/

/************************ Functions Definitions *******************************/

/******************************************************************************/



/**

 * @brief Obtain the GPIO decriptor.

 * @param desc - The GPIO descriptor.

 * @param gpio_number - The number of the GPIO.

 * @return SUCCESS in case of success, FAILURE otherwise.

 */

int32_t gpio_get(struct gpio_desc **desc,

		 uint8_t gpio_number)

{

	if (desc) {

		// Unused variable - fix compiler warning

	}



	if (gpio_number) {

		// Unused variable - fix compiler warning

	}



	return SUCCESS;

}



/**

 * @brief Free the resources allocated by gpio_get().

 * @param desc - The SPI descriptor.

 * @return SUCCESS in case of success, FAILURE otherwise.

 */

int32_t gpio_remove(struct gpio_desc *desc)

{

	if (desc) {

		// Unused variable - fix compiler warning

	}



	return SUCCESS;

}



/**

 * @brief Enable the input direction of the specified GPIO.

 * @param desc - The GPIO descriptor.

 * @return SUCCESS in case of success, FAILURE otherwise.

 */

int32_t gpio_direction_input(struct gpio_desc *desc)

{

	if (desc) {

		// Unused variable - fix compiler warning

	}



	return SUCCESS;

}



/**

 * @brief Enable the output direction of the specified GPIO.

 * @param desc - The GPIO descriptor.

 * @param value - The value.

 *                Example: GPIO_HIGH

 *                         GPIO_LOW

 * @return SUCCESS in case of success, FAILURE otherwise.

 */

int32_t gpio_direction_output(struct gpio_desc *desc,

			      uint8_t value)

{

	if (desc) {

		// Unused variable - fix compiler warning

	}



	if (value) {

		// Unused variable - fix compiler warning

	}



	return SUCCESS;

}



/**

 * @brief Get the direction of the specified GPIO.

 * @param desc - The GPIO descriptor.

 * @param direction - The direction.

 *                    Example: GPIO_OUT

 *                             GPIO_IN

 * @return SUCCESS in case of success, FAILURE otherwise.

 */

int32_t gpio_get_direction(struct gpio_desc *desc,

			   uint8_t *direction)

{

	if (desc) {

		// Unused variable - fix compiler warning

	}



	if (direction) {

		// Unused variable - fix compiler warning

	}



	return SUCCESS;

}



/**

 * @brief Set the value of the specified GPIO.

 * @param desc - The GPIO descriptor.

 * @param value - The value.

 *                Example: GPIO_HIGH

 *                         GPIO_LOW

 * @return SUCCESS in case of success, FAILURE otherwise.

 */

int32_t gpio_set_value(struct gpio_desc *desc,

		       uint8_t value)

{

	if (desc) {

		// Unused variable - fix compiler warning

	}



	if (value) {

		// Unused variable - fix compiler warning

	}



	return SUCCESS;

}



/**

 * @brief Get the value of the specified GPIO.

 * @param desc - The GPIO descriptor.

 * @param value - The value.

 *                Example: GPIO_HIGH

 *                         GPIO_LOW

 * @return SUCCESS in case of success, FAILURE otherwise.

 */

int32_t gpio_get_value(struct gpio_desc *desc,

		       uint8_t *value)

{

	if (desc) {

		// Unused variable - fix compiler warning

	}



	if (value) {

		// Unused variable - fix compiler warning

	}



	return SUCCESS;

}
