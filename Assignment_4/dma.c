/*
 *
 *      File: dma.c
 *
 *      Created on: feb 14,2017
 *      Author: Vignesh Jayaram
 *
 *      Permission is granted to anyone to use this software for any purpose,
 *  	including commercial applications, and to alter it and redistribute it
 *  	freely, subject to the following restrictions:
 *
 *  	1. The origin of this software must not be misrepresented; you must not
 *    	claim that you wrote the original software.
 *  	2. Altered source versions must be plainly marked as such, and must not be
 *   	 misrepresented as being the original software.
 *  	3. This notice may not be removed or altered from any source distribution.
 *
 *  	DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 *  	obligation to support this Software. Silicon Labs is providing the
 *  	Software "AS IS", with no express or implied warranties of any kind,
 *  	including, but not limited to, any implied warranties of merchantability
 *  	or fitness for any particular purpose or warranties against infringement
 *  	of any proprietary rights of a third party.
 *
 *  	Silicon Labs will not be liable for any consequential, incidental, or
 *  	special damages, or any other relief, or for any claim by any third party,
 *  	arising from your use of this Software.
 *
 *      DMA_ADC0_CB(unsigned int channel, bool primary, void *user);
 *		This routine is a call back function which executes on DMA interrupt
 *
 *		DMA_ADC0_Setup(void);
 *		Individual call to setup up the DMA Channel for the ADC0
 *
 *		DMA_Setup(void)
 *		Main call to set up the DMA descriptor location and all of the individual DMA channels
 *
 */

#include "dma.h"


DMA_CB_TypeDef adc0_cb =								// DMA callback structure
{
		.cbFunc = DMA_ADC0_CB,
		.userPtr = NULL,
		.primary = true
};


/*********************************************
 * Call Back function to close off the DMA operation
 *
 * Input variable:  channel , primary, user pointer
 * Global variable: ramBufferAdcData
 * Return type: none
 * MACROS: TOTAL_CONVERSION,HIGH_TEMP_LIMIT_C,LOW_TEMP_LIMIT_C
 *
 *********************************************/
void DMA_ADC0_CB(unsigned int channel, bool primary, void *user)
{
    int32_t sum = 0;
    int32_t average;
    float temp_C = 0;
    INT_Disable();									//Disable interrupts

    DMA_IntClear(DMA_CHANNEL_ADC);					//clear the DMA interrupts
    ADC0->CMD = ADC_CMD_SINGLESTOP;					//Disable ADC
    unblockSleepMode(ADC_EM);

    for (int i=0; i < TOTAL_CONVERSION; i++)		//Add all the 750 samples
    {
            sum += ramBufferAdcData[i];
    }
    average = sum / TOTAL_CONVERSION;				//Take average of the samples

    temp_C = convertToCelsius(average);				//Calculate the temperature in celsius

    if((temp_C > HIGH_TEMP_LIMIT_C) || (temp_C < LOW_TEMP_LIMIT_C))	//check for temperature wrt thresholds
    {
    	LED_ON(1);
    }
    else
    {
    	LED_OFF(1);
    }

    INT_Enable();										//Re-enable the interrupts
}


/*********************************************
 * Individual call to setup up the DMA Channel for the ADC0
 *
 * Input variable:  none
 * Global variable: ramBufferAdcData
 * Return type: none
 * MACROS: ADC0_DMA_ARBITRATE,ADC0_DESTINATION_INCREMENT,ADC_DMA_TRANSFER_SIZE,ADC0_SOURCE_INCREMENT
 * 			DMA_CHANNEL_ADC
 *
 *********************************************/
void DMA_ADC0_Setup(void)
{
    DMA_CfgDescr_TypeDef DMA_ADC0_cfg =
    {
    	.arbRate = ADC0_DMA_ARBITRATE,					//Arbitrate factor set to 1 cycle
		.dstInc = ADC0_DESTINATION_INCREMENT,			//Destination increment set to 2 bytes
		.hprot = 0,										//no protection
		.size = ADC_DMA_TRANSFER_SIZE,					//DMA transfers of 2 bytes
		.srcInc = ADC0_SOURCE_INCREMENT					//Do not increment source
    };

    DMA_CfgDescr(DMA_CHANNEL_ADC, true, &DMA_ADC0_cfg);	//Configure the descriptor

    DMA_CfgChannel_TypeDef DMA_ADC0_channel =
    {
    		.cb = &adc0_cb,								//Address of callback routine
			.enableInt = true,							//enable the interrupts
			.highPri = true,							//assign high priority
			.select = DMAREQ_ADC0_SINGLE				//ADC0 for single scan mode
    };

    DMA_CfgChannel(DMA_CHANNEL_ADC, &DMA_ADC0_channel);	//Configure the DMA channel 0

    DMA_IntClear(DMA_CHANNEL_ADC);						//Clear the interrupts
    DMA_IntEnable(DMA_CHANNEL_ADC);						//Enable channel 0 interrupt

    DMA_ActivateBasic(DMA_CHANNEL_ADC,					//Activate Basic mode for DMA channel 0
                        true,							//use primary descriptor
                        false,							//Burst mode disabled
                        (void *)ramBufferAdcData,		//destination address
                        (void *)&(ADC0->SINGLEDATA),	//source address
                        TOTAL_CONVERSION - 1);			//No of ADC reads required
}

/***************************************************************************************
 * Main call to set up the DMA descriptor location and all of the individual DMA channels
 *
 * Input variable:  none
 * Global variable: ramBufferAdcData
 * Return type: none
 * MACROS: none
 *
 *********************************************/
void DMA_Setup(void)
{
    DMA_Init_TypeDef DMA_init =
    {
    		.controlBlock = dmaControlBlock,
			.hprot = 0
    };
    DMA_Init(&DMA_init);    /* initial DMA dma Control Block descriptor location */

    /* call all the individual DMA setup routines */
    DMA_ADC0_Setup();				//Setup the individual descriptors and channels
    NVIC_EnableIRQ(DMA_IRQn);		//Enable the NVIC for DMA
}


