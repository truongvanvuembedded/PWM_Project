/*
 * spi.c
 *
 *  Created on: Nov 6, 2022
 *      Author: hussamaldean
 */


#include "spi.h"

#include "stm32f4xx.h"


void OLED_SPI_Pins_Init()
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN; //enable clock for GPIOA

	//set PA5, PA6 and PA7 to alternate function mode
	GPIOA->MODER|=GPIO_MODER_MODE5_1|GPIO_MODER_MODE6_1|GPIO_MODER_MODE7_1;
	GPIOA->MODER &=~(GPIO_MODER_MODE5_0|GPIO_MODER_MODE6_0|GPIO_MODER_MODE7_0);

	//Set PA9 and PA10 as Output
	GPIOA->MODER|=GPIO_MODER_MODE9_0|GPIO_MODER_MODE10_0;
	GPIOA->MODER&=~(GPIO_MODER_MODE9_1|GPIO_MODER_MODE10_1);

	/*select which AF for PA5, PA6 and PA7*/
	GPIOA->AFR[0]|=(0x05<<20)|(0x05<<24)|(0x05<<28);
}

void OLED_SPI_Configure()
{
	/*Enable clock access to SPI1 module*/
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

		/*Set clock to fPCLK/2*/
		SPI1->CR1 &=~(1U<<3);
		SPI1->CR1 &=~(1U<<4);
		SPI1->CR1 &=~(1U<<5);

		/*Enable full duplex*/
		SPI1->CR1 &=~(1U<<10);

		/*Set MSB first*/
		SPI1->CR1 &= ~(1U<<7);

		/*Set mode to MASTER*/
		SPI1->CR1 |= (1U<<2);

		/*Set 8 bit data mode*/
		SPI1->CR1 &= ~(1U<<11);

		/*Select software slave management by
		 * setting SSM=1 and SSI=1*/
		SPI1->CR1 |= (1<<8);
		SPI1->CR1 |= (1<<9);

		/*Enable SPI module*/
		SPI1->CR1 |= (1<<6);
}

void OLED_SPI_Write(char *data,uint32_t size)
{
	uint32_t i=0;

	while(i<size)
	{
		/*Wait until TXE is set*/
		while(!(SPI1->SR & (SPI_SR_TXE))){}

		/*Write the data to the data register*/
		SPI1->DR =(uint8_t) data[i];
		i++;
	}
	/*Wait until TXE is set*/
	while(!(SPI1->SR & (SPI_SR_TXE))){}

	/*Wait for BUSY flag to reset*/
	while((SPI1->SR & (SPI_SR_BSY))){}

	/*Clear OVR flag*/
	(void)SPI1->DR;
	(void)SPI1->SR;
}



void OLED_Select(void)
{
	GPIOA->BSRR =GPIO_BSRR_BR9;

}

/*Pull high to disable*/
void OLED_Deselect(void)
{
	GPIOA->BSRR =GPIO_BSRR_BS9;
}

void OLED_DataMode()
{
	GPIOA->BSRR=GPIO_BSRR_BS10;
}

void OLED_CommMode()
{
	GPIOA->BSRR=GPIO_BSRR_BR10;
}
