/*
 * motorcycle.c
 *
 *  Created on: 23 сент. 2019 г.
 *      Author: Romich
 */
#include "motorcycle.h"

int search_flag = 0;
int helmet_numbers = 0;
extern int advertize_flag;
uint32_t helmet_address[3];
uint32_t address;
TIM_HandleTypeDef htim4;

void TEST (uint16_t command,uint16_t light, uint16_t status)
{
	//NRF.addr0 = address0;
	NRF.com = command;
	NRF.lght  = light;
	NRF.status0 = status;

	int ok_flag = 0;
	int count = 0;
	green_on;
	while (ok_flag == 0)
	{
		write(&NRF, 20);
		if(isAckPayloadAvailable()) // проверяем пришло ли что-то вместе с ответом
		{
			read(&ANSW, 6);
			if (ANSW.data1 == 0xAA) ok_flag = 1;
		}
		count ++;
		if (count == 20) ok_flag = 1;
	}
	ANSW.data1 = 0x00;
	green_off;
}

void ADVERTIZE ()
{
	blue_on;
	NRF.com = advert;
	HAL_TIM_Base_Start_IT(&htim4);
    while (advertize_flag == 1)
    {
    	write(&NRF, 20);
    	if(isAckPayloadAvailable()) // проверяем пришло ли что-то вместе с ответом
    	{
    		read(&ANSW, 6);
    		if (ANSW.data1 == 0xAA)
    		{
    			advertize_flag=0;
    		    HAL_GPIO_WritePin(GPIOC,BLUE_Pin, GPIO_PIN_SET);
    		    address = ANSW.addr;
    		}

    	}
    	//HAL_Delay(50);
    }
	HAL_TIM_Base_Stop_IT(&htim4);
    blue_off;

}

void HELMET_SEARCH()
{
	search_flag = 0;
	int q = 0;
	helmet_address[0]= 0;
	helmet_address[1]= 0;
	helmet_address[2]= 0;

	red_on;
	NRF.com = search;
	for (int i=0; i<=5; i++)
	{
		write(&NRF, 20);
		if(isAckPayloadAvailable()) // проверяем пришло ли что-то вместе с ответом
		{
			read(&ANSW, 6);
			if (ANSW.data1 == 0xCC)
			{
				if ((ANSW.addr!=helmet_address[0])&&(ANSW.addr!=helmet_address[1])&&(ANSW.addr!=helmet_address[2]))
				{
					helmet_address[q]= ANSW.addr;
					q++;
				}
			}
		}

	}
	helmet_numbers = q;
	q=0;

	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	red_off;
}


void unique_id()
{

		volatile uint32_t *UniqueID = (uint32_t *)0x1FFFF7E8;
		volatile uint32_t __UniqueID[3];
		__UniqueID[0] = UniqueID[0];
		__UniqueID[1] = UniqueID[1];
		__UniqueID[2] = UniqueID[2];

		NRF.addr0 = UniqueID[0];
		NRF.addr1 = UniqueID[1];
		NRF.addr2 = UniqueID[2];
}

void WORK (uint16_t command,uint16_t light, uint16_t status)
{
	NRF.com = command;
	NRF.lght  = light;
	NRF.status0 = status;

	int ok_flag = 0;
	int count = 0;
	green_on;
	if (helmet_numbers == 0) helmet_numbers = 1;
	for (int i = helmet_numbers; i!=0; i--)
	{
		NRF.helmet_addr = helmet_address[i];
		while (ok_flag == 0)
		{
			write(&NRF, 20);
			if(isAckPayloadAvailable()) // проверяем пришло ли что-то вместе с ответом
			{
				read(&ANSW, 6);
				if (ANSW.data1 == 0xAA) ok_flag = 1;
			}
			count ++;
			if (count == 3) ok_flag = 1;
		}
	}
	ANSW.data1 = 0x00;
	green_off;
}
