/*
 * 	File: dma.h
 *
 *  Created on: feb 14,2017
 *  Author: Vignesh Jayaram
 *
 *	DMA_ADC0_CB(unsigned int channel, bool primary, void *user);
 *	This routine is a call back function which executes on DMA interrupt
 *
 *	DMA_ADC0_Setup(void);
 *	Individual call to setup up the DMA Channel for the ADC0
 *
 *	DMA_Setup(void)
 *	Main call to set up the DMA descriptor location and all of the individual DMA channels
 *
 */

#ifndef DMA_H
#define DMA_H


#include "sleep.h"
#include "adc0.h"
#include "led.h"

#define DMA_CHANNEL_ADC 0   	      	//DMA channel 0
#define ADC0_DMA_ARBITRATE dmaArbitrate1 //Macro to set arbitration to 1024
#define ADC0_DESTINATION_INCREMENT dmaDataInc2 //Macro for destination increment
#define ADC0_SOURCE_INCREMENT dmaDataIncNone   //Macro for source increment
#define ADC_DMA_TRANSFER_SIZE dmaDataSize2	   //Macro for hwo many bytes DMA should transfer



#if ( ( DMA_CHAN_COUNT > 0 ) && ( DMA_CHAN_COUNT <= 4 ) )
#define DMACTRL_CH_CNT      4
#define DMACTRL_ALIGNMENT   128

#elif ( ( DMA_CHAN_COUNT > 4 ) && ( DMA_CHAN_COUNT <= 8 ) )
#define DMACTRL_CH_CNT      8
#define DMACTRL_ALIGNMENT   256

#elif ( ( DMA_CHAN_COUNT > 8 ) && ( DMA_CHAN_COUNT <= 12 ) )
#define DMACTRL_CH_CNT      16
#define DMACTRL_ALIGNMENT   256

#else
#error "Unsupported DMA channel count (dmactrl.c)."
#endif

/** DMA control block array, requires proper alignment. */
SL_ALIGN(DMACTRL_ALIGNMENT)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMACTRL_CH_CNT * 2] SL_ATTRIBUTE_ALIGN(DMACTRL_ALIGNMENT);

volatile uint16_t ramBufferAdcData[TOTAL_CONVERSION];		//Array to store the ADC values


void DMA_ADC0_CB(unsigned int channel, bool primary, void *user);
void DMA_ADC0_Setup(void);
void DMA_Setup(void);


#endif /* SRC_DMA_H_ */
