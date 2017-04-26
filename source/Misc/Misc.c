/*
 * Misc.c
 *
 *  Created on: 26 avr. 2017
 *      Author: hanch_000
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "Misc.h"


/*********             Internal macro             *********/

#define PCR_ODE_ENABLED               0x01u   /*!< Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output. */
#define PCR_PE_ENABLED                0x01u   /*!< Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
#define PCR_PS_UP                     0x01u   /*!< Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */

/*********             Internal types             *********/

/*********            Internal variables          *********/

/*********      Internal function declaration     *********/

/*********     Internal function implementation   *********/

/*********      Public function implementation    *********/
void Misc_EnableHv(bool en) {
	if (en) {
		PORTB->PCR[10] =
				((PORTB->PCR[10]
						& (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK
								| PORT_PCR_ISF_MASK))));
	} else {
		PORTB->PCR[10] = ((PORTB->PCR[10]
				& (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))
				| PORT_PCR_PS(PCR_PS_UP) | PORT_PCR_PE(PCR_PE_ENABLED));
	}
}
