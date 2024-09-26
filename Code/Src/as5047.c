#include "as5047.h"
#include "delay.h"

uint16_t as5047_read_angle()
{
    uint16_t data;
    // LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_4);
    // LL_SPI_TransmitData16(SPI1,0x4001);
    // while (LL_SPI_IsActiveFlag_BSY(SPI1));
    // data=LL_SPI_ReceiveData16(SPI1);
    // LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_4);
    // delay_us(1);


    LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_4);
    LL_SPI_TransmitData16(SPI1,0xFFFF);
    // LL_SPI_TransmitData16(SPI1,0xBFFE);
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    delay_us(1);
    LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_4);
	data=LL_SPI_ReceiveData16(SPI1);
    delay_us(1);
    // if((data & 0x400) == 0)
    return (data&0x3FFF);
}