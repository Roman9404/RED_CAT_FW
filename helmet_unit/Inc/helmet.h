/*
 * HELMET.h
 *
 *  Created on: 23 сент. 2019 г.
 *      Author: Romich
 */
#include "RF24.h"
#include "main.h"

#ifndef HELMET_H_
#define HELMET_H_

/*служебные команды*/
#define advert		0x1111
#define connect		0x2222
#define work		0x3333
#define search		0x4444

/*адрес*/
#define address0	0x12345678

/*огни*/
#define left_turn	0x0001
#define right_turn	0x0002
#define stop		0x0003
#define gabarit		0x0004

/*статус огней*/
#define on			0x4444
#define off			0x5555

/*индикаторные светодиоды*/

#define led1_on		HAL_GPIO_WritePin(GPIOB, LED_1_Pin,GPIO_PIN_RESET);
#define led1_off	HAL_GPIO_WritePin(GPIOB, LED_1_Pin,GPIO_PIN_SET);

#define led2_on		HAL_GPIO_WritePin(GPIOB, LED_2_Pin,GPIO_PIN_RESET);
#define led2_off	HAL_GPIO_WritePin(GPIOB, LED_2_Pin,GPIO_PIN_SET);

#define led3_on		HAL_GPIO_WritePin(GPIOB, LED_3_Pin,GPIO_PIN_RESET);
#define led3_off	HAL_GPIO_WritePin(GPIOB, LED_3_Pin,GPIO_PIN_SET);

#define led4_on		HAL_GPIO_WritePin(GPIOB, LED_4_Pin,GPIO_PIN_RESET);
#define led4_off	HAL_GPIO_WritePin(GPIOB, LED_4_Pin,GPIO_PIN_SET);

#define led1_4_on	HAL_GPIO_WritePin(GPIOB, LED_1_Pin|LED_2_Pin|LED_3_Pin|LED_4_Pin, GPIO_PIN_RESET);
#define led1_4_off	HAL_GPIO_WritePin(GPIOB, LED_1_Pin|LED_2_Pin|LED_3_Pin|LED_4_Pin, GPIO_PIN_SET);


/*управление питанием*/

#define Power_12_Volt_on		HAL_GPIO_WritePin(GPIOA, EN_12V_Pin, GPIO_PIN_SET);
#define Power_12_Volt_off		HAL_GPIO_WritePin(GPIOA, EN_12V_Pin, GPIO_PIN_RESET);

#define Power_3_3_Volt_on		HAL_GPIO_WritePin(GPIOA, EN_3_3V_Pin,GPIO_PIN_SET);
#define Power_3_3_Volt_off 		HAL_GPIO_WritePin(GPIOA, EN_3_3V_Pin,GPIO_PIN_RESET);

/*отладочный светодиод*/
#define blue_on		HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_SET);
#define blue_off	HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
#define green_on	HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_SET);
#define green_off	HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
#define red_on		HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_SET);
#define red_off		HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);
#define red_toggle	HAL_GPIO_TogglePin(GPIOC, BLUE_Pin);

struct DATA_NRF
{
	uint16_t com;		//служебная команда

	uint32_t addr0;		//адрес мотоциклетной части 0
	uint32_t addr1;		//адрес мотоциклетной части 1
	uint32_t addr2;		//адрес мотоциклетной части 2

	uint16_t lght;		//огонь

	uint16_t status0;	//статус огней
	uint16_t status1;	//резерв статуса огней

	uint32_t helmet_addr;	//резерв
	uint32_t rezerv1;	//резерв
	uint32_t rezerv2;	//резерв

}NRF;

struct ANSWER
{
	uint8_t data1;
	uint32_t addr;
	uint8_t data2;
	//uint8_t data3;

}ANSW;


#endif /* HELMET_H_ */
