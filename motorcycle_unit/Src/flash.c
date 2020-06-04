/*
 * flash.c
 *
 *  Created on: 11 ���. 2020 �.
 *      Author: Roman
 */
#include "stm32f1xx_hal.h"

 /*������� ������������� flash ������*/
void flash_unlock(void)
{
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
}
/*������� ���������� flash ������*/
void flash_lock()
{
	FLASH->CR |= FLASH_CR_LOCK;
}

//������� ��������� true ����� ����� ������� ��� ������ ������.
uint8_t flash_ready(void)
{
	return !(FLASH->SR & FLASH_SR_BSY);
}

/*������� ������� ��� ��������. ��� � ������ �������� ����������������*/
void flash_erase_all_pages(void)
{
	FLASH->CR |= FLASH_CR_MER; 		//������������� ��� �������� ���� �������
	FLASH->CR |= FLASH_CR_STRT; 	//������ ��������
	while(!flash_ready()); 			// �������� ����������.
	FLASH->CR &= FLASH_CR_MER;
}

/*������� ������� ���� ��������. � �������� ������ ����� ������������ ����� �����,
*������������� ��������� ������� ��� ��������, ������� ����� ��������.*/
void flash_erase_page(uint32_t address)
{
	FLASH->CR|= FLASH_CR_PER; 		//������������� ��� �������� ����� ��������
	FLASH->AR = address; 			//������ � �����
	FLASH->CR|= FLASH_CR_STRT; 		//��������� ��������
	while(!flash_ready());  		//���� ���� �������� ��������.
	FLASH->CR&= ~FLASH_CR_PER; 		//���������� ��� �������
}

/*������� ������ ���� ���� �� Flash  ������*/
void flash_write(uint32_t address,uint32_t data)
{
	FLASH->CR |= FLASH_CR_PG; 		//��������� ���������������� �����
	while(!flash_ready()); 			//������� ���������� ����� � ������
	*(__IO uint16_t*)address = (uint16_t)data; //����� ������� 2 �����
	while(!flash_ready());
	address+=2;
	data>>=16;
	*(__IO uint16_t*)address = (uint16_t)data; //����� ������� 2 �����
	while(!flash_ready());
	FLASH->CR &= ~(FLASH_CR_PG); 	//��������� ���������������� �����
}

/*������� ������ ������ flash ������*/
uint32_t flash_read(uint32_t address)
{
	return (*(__IO uint32_t*) address);
}



