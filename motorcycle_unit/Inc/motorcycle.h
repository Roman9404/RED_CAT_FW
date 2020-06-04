/*
 * motorcycle.h
 *
 *  Created on: 23 ����. 2019 �.
 *      Author: Romich
 */
#include "RF24.h"
#include "main.h"

#ifndef MOTORCYCLE_H_
#define MOTORCYCLE_H_

/*��������� �������*/
#define advert		0x1111
#define connect		0x2222
#define work		0x3333
#define search		0x4444

/*�����*/
#define address0	0x87654321//	0x12345678

/*����*/
#define left_turn	0x0001
#define right_turn	0x0002
#define stop		0x0003
#define gabarit		0x0004

/*������ �����*/
#define on			0x4444
#define off			0x5555

/*���������� ���������*/
#define red_on		HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_SET);
#define red_off		HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
#define green_on	HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_SET);
#define green_off	HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
#define blue_on		HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_SET);
#define blue_off	HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);

struct DATA_NRF
{
	uint16_t com;		//��������� �������

	uint32_t addr0;		//����� ������������� ����� 0
	uint32_t addr1;		//����� ������������� ����� 1
	uint32_t addr2;		//����� ������������� ����� 2

	uint16_t lght;		//�����

	uint16_t status0;	//������ �����
	uint16_t status1;	//������ ������� �����

	uint32_t helmet_addr;	//������
	uint32_t rezerv1;	//������
	uint32_t rezerv2;	//������

}NRF;

struct ANSWER
{
	uint8_t data1;
	uint32_t addr;
	uint8_t data2;
	//uint8_t data3;

}ANSW;

#endif /* MOTORCYCLE_H_ */
