/*
 * HELMET.c
 *
 *  Created on: 23 сент. 2019 г.
 *      Author: Romich
 */
#include "helmet.h"
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim;

extern int connect_flag;
extern int stop_flag;
int search_flag = 1;
int adc = 0;
int bat_adc = 0;
int gabarit_flag = 0;
int light_check_flag = 0;

extern uint32_t my_addres;

const uint64_t pipe1 = 0xE8E8F0F0E2LL; // адрес первой трубы
const uint64_t pipe2 = 0xE8E8F0F0A2LL;

uint32_t address_0 = 0;
uint32_t address_1 = 0;
uint32_t address_2 = 0;

void TEST (uint8_t pipe_num)
{
	if(pipe_num == 1)
	{
		read(&NRF, 20); // Читаем данные в массив nrf_data и указываем сколько байт читать
		switch (NRF.com)
		{
		case work: 		if ((NRF.addr0 == address_0)&&(NRF.addr1 == address_1)&&(NRF.addr2 == address_2)) WORK(pipe_num);
						break;

		case search:	if ((NRF.addr0 == address_0)&&(NRF.addr1 == address_1)&&(NRF.addr2 == address_2)) HELMET_SEARCH(pipe_num);
						break;

		case advert: 	CONNECT(pipe_num);
						break;
		}
	}
}

void WORK (uint8_t pip_num)
{

	stop_flag = 10;
	if (NRF.lght==stop)
	{
		if(NRF.status0==on) TIM2->CCR1 = 1000;
		if(NRF.status0==off) TIM2->CCR1 = 0;

	}

	if (NRF.lght==gabarit)
	{
	//	if(NRF.status0==on) TIM2->CCR2 = 1000;
	//	if(NRF.status0==off) TIM2->CCR2 = 0;
	}

	if (NRF.lght==left_turn)
	{
		if(NRF.status0==on) TIM2->CCR3 = 1000;
		if(NRF.status0==off) TIM2->CCR3 = 0;
	}

	if (NRF.lght==right_turn)
	{
		if(NRF.status0==on) TIM2->CCR4 = 1000;
		if(NRF.status0==off) TIM2->CCR4 = 0;
	}
	if (NRF.helmet_addr == my_addres)
	{
		ANSW.data1 = 0xAA;
		writeAckPayload(pip_num, &ANSW, 6);
		ANSW.data1 = 0x00;
	}

	ANSW.data1 = 0x00;


}

void CONNECT(uint8_t pip_num)
{
	if (connect_flag == 1)
	{
		address_0 = NRF.addr0;
		address_1 = NRF.addr1;
		address_2 = NRF.addr2;
		ANSW.data1 = 0xAA;
		writeAckPayload(pip_num, &ANSW, 6);
		HAL_Delay(50);
		writeAckPayload(pip_num, &ANSW, 6);
		HAL_Delay(50);
		writeAckPayload(pip_num, &ANSW, 6);
		HAL_Delay(50);
		writeAckPayload(pip_num, &ANSW, 6);
		HAL_Delay(50);
		writeAckPayload(pip_num, &ANSW, 6);
		HAL_Delay(50);
		writeAckPayload(pip_num, &ANSW, 6);

		HAL_TIM_Base_Stop_IT(&htim4);
		TIM2->CCR3 = 0;
		TIM2->CCR4 = 0;

		HAL_Delay(200);
		TIM2->CCR3 = 1000;
		TIM2->CCR4 = 1000;

		HAL_Delay(200);
		TIM2->CCR3 = 0;
		TIM2->CCR4 = 0;

		HAL_Delay(200);
		TIM2->CCR3 = 1000;
		TIM2->CCR4 = 1000;

		HAL_Delay(300);
		TIM2->CCR3 = 0;
		TIM2->CCR4 = 0;

		connect_flag = 0;
		green_off;
	}

	HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_4);

	flash_unlock();
	flash_erase_page(0x08004000);
	flash_write(0x08004000, address_0);
	flash_write(0x08004004, address_1);
	flash_write(0x08004008, address_2);
	flash_lock();

	//HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
//	connect_flag = 0;

	//HAL_GPIO_WritePin(GPIOC, BLUE_Pin,GPIO_PIN_SET);
}
int schet = 0;
void HELMET_SEARCH(uint8_t pip_num)
{

	if (search_flag == 1)
	{
		ANSW.data1 = 0xCC;
		writeAckPayload(pip_num, &ANSW, 6);
		//*
		HAL_Delay(20);
		writeAckPayload(pip_num, &ANSW, 6);
		HAL_Delay(20);
		writeAckPayload(pip_num, &ANSW, 6);
		HAL_Delay(20);
	/*	writeAckPayload(pip_num, &ANSW, 6);
		HAL_Delay(20);
		writeAckPayload(pip_num, &ANSW, 6);
		HAL_Delay(20);
		writeAckPayload(pip_num, &ANSW, 6);
		HAL_Delay(20);
		//*/
		search_flag = 0;
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
		blue_on;
		schet++;
	}
	light_check_flag = 0;
	stop_flag = 120;
//	if (TIM2->CCR1 != 1000) TIM2->CCR1 = 100;
}

void light_check()
{
	HAL_ADC_Start(&hadc2); 						// запускаем преобразование сигнала АЦП
	HAL_ADC_PollForConversion(&hadc2, 100); 	// ожидаем окончания преобразования
	adc = HAL_ADC_GetValue(&hadc2); 			// читаем полученное значение в переменную adc
	HAL_ADC_Stop(&hadc2); 						// останавливаем АЦП (не обязательно)
	if (adc>=4000)
		{
			TIM2->CCR2 = 100;
			red_on;
		}
	else
		{
			TIM2->CCR2 = 400;
			red_off;
		}
}

void unique_id()
{

		volatile uint32_t *UniqueID = (uint32_t *)0x1FFFF7E8;
		volatile uint32_t __UniqueID[3];
		__UniqueID[0] = UniqueID[0];
		__UniqueID[1] = UniqueID[1];
		__UniqueID[2] = UniqueID[2];

		my_addres = UniqueID[2];
		ANSW.addr = UniqueID[2];
}

void battery_check(int flag_bat_led)
{
	HAL_GPIO_WritePin(GPIOB,BAT_CHECK_Pin, GPIO_PIN_SET);
	HAL_ADC_Start(&hadc1); // запускаем преобразование сигнала АЦП
	HAL_ADC_PollForConversion(&hadc1, 100); // ожидаем окончания преобразования
	bat_adc = HAL_ADC_GetValue(&hadc1); // читаем полученное значение в переменную adc
	HAL_ADC_Stop(&hadc1); // останавливаем АЦП (не обязательно)
	HAL_GPIO_WritePin(GPIOB,BAT_CHECK_Pin, GPIO_PIN_RESET);
	if (flag_bat_led == 1)
	{
		if (bat_adc >= 3200)
		{
			led1_4_on;
		}
		if ((bat_adc >= 3000)&&(bat_adc < 3200))
		{
			led1_on;
			led2_on;
			led3_on;
		}
		if ((bat_adc >= 2800)&&(bat_adc < 3200))
		{
			led1_on;
			led2_on;
		}
		if ((bat_adc >= 2600)&&(bat_adc < 2800))
		{
			led1_on;
		}
		if (bat_adc < 2600)
		{
			led1_on;
			HAL_Delay(300);
			led1_off;
			HAL_Delay(300);

			led1_on;
			HAL_Delay(300);
			led1_off;
			HAL_Delay(300);

			led1_on;
			HAL_Delay(300);
			led1_off;
			HAL_Delay(300);

			led1_on;
			HAL_Delay(300);
			led1_off;
			HAL_Delay(300);

			Power_12_Volt_off;
			Power_3_3_Volt_off;
		}
	}
}

void gabarit_btn()
{
	if (gabarit_flag == 0)
	{
		gabarit_flag = 1;
		light_check_flag = 1;
	//	TIM2->CCR2 = 1000;
	}
	else
	{
		gabarit_flag = 0;
		light_check_flag = 1;
		//TIM2->CCR2 = 0;
	}

}
