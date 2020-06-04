/*
 * flash.c
 *
 *  Created on: 11 янв. 2020 г.
 *      Author: Roman
 */
#include "stm32f1xx_hal.h"

 /*функция разблокировки flash памяти*/
void flash_unlock(void)
{
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
}
/*функция блокировки flash памяти*/
void flash_lock()
{
	FLASH->CR |= FLASH_CR_LOCK;
}

//Функция возврщает true когда можно стирать или писать память.
uint8_t flash_ready(void)
{
	return !(FLASH->SR & FLASH_SR_BSY);
}

/*Функция стирает ВСЕ страницы. При её вызове прошивка самоуничтожается*/
void flash_erase_all_pages(void)
{
	FLASH->CR |= FLASH_CR_MER; 		//Устанавливаем бит стирания ВСЕХ страниц
	FLASH->CR |= FLASH_CR_STRT; 	//Начать стирание
	while(!flash_ready()); 			// Ожидание готовности.
	FLASH->CR &= FLASH_CR_MER;
}

/*Функция стирает одну страницу. В качестве адреса можно использовать любой адрес,
*принадлежащий диапазону адресов той странице, которую нужно очистить.*/
void flash_erase_page(uint32_t address)
{
	FLASH->CR|= FLASH_CR_PER; 		//Устанавливаем бит стирания одной страницы
	FLASH->AR = address; 			//Задаем её адрес
	FLASH->CR|= FLASH_CR_STRT; 		//Запускаем стирание
	while(!flash_ready());  		//Ждем пока страница сотрется.
	FLASH->CR&= ~FLASH_CR_PER; 		//Сбрасываем бит обратно
}

/*функция записи двух байт во Flash  память*/
void flash_write(uint32_t address,uint32_t data)
{
	FLASH->CR |= FLASH_CR_PG; 		//Разрешаем программирование флеша
	while(!flash_ready()); 			//Ожидаем готовности флеша к записи
	*(__IO uint16_t*)address = (uint16_t)data; //Пишем младшие 2 байта
	while(!flash_ready());
	address+=2;
	data>>=16;
	*(__IO uint16_t*)address = (uint16_t)data; //Пишем старшие 2 байта
	while(!flash_ready());
	FLASH->CR &= ~(FLASH_CR_PG); 	//Запрещаем программирование флеша
}

/*функция чтения ячейки flash памяти*/
uint32_t flash_read(uint32_t address)
{
	return (*(__IO uint32_t*) address);
}



