#include "AS5600.h"
#include "delay.h"
#include "usbd_cdc_if.h"
#include "math.h"

#define err()   USBVcom_printf("send_error\r\n")

void AS5600_Init(void)
{
   uint16_t data;
   data = AS_read_reg_u16(0x0C);
   AS_write_reg_u16(0x01, data);
   HAL_Delay(2);
   data = AS_read_reg_u16(0x0C);
   AS_write_reg_u16(0x03, data);
   HAL_Delay(2);

   AS_write_reg_u8(0xFF, 0x80);
   HAL_Delay(2);
   AS_write_reg_u8(0xFF, 0x01);
   HAL_Delay(2);
   AS_write_reg_u8(0xFF, 0x11);
   HAL_Delay(2);
   AS_write_reg_u8(0xFF, 0x10);
   HAL_Delay(2);

}


void I2c_delay(void)
{
    delay_us(2);
}

void I2C_strat(void)
{
    I2C_SDA_UP();
    I2c_delay();
    I2C_SCL_UP();
    I2c_delay();
    I2C_SDA_DOWM();
    I2c_delay();
    I2C_SCL_DOWM();
    I2c_delay();

}

void I2C_end(void)
{
    I2C_SDA_DOWM();
    I2c_delay();
    I2C_SCL_UP();
    I2c_delay();
    I2C_SDA_UP();

}

uint8_t I2C_wait_ACK(void)
{
    uint8_t time = 0;
    
    I2C_SDA_UP();
    I2c_delay();
    I2C_SCL_UP();
    I2c_delay();
    while(I2C_SDA_READ == 1)//if acknowlege isn't come
    {
        time++;
        if(time>250)
        {
            I2C_end();
            err();
            return 1;
        }
    }
    I2c_delay();
    I2C_SCL_DOWM();
    I2c_delay();
    return 0;
}

void I2C_ACK(void)
{
    // I2C_SDA_DOWM();
    // I2c_delay();
    // I2C_SCL_UP();
    // I2c_delay();
    // I2C_SCL_DOWM();
    // I2c_delay();
    I2C_SCL_DOWM();
    I2c_delay();
    I2C_SDA_DOWM();
    I2c_delay();
    I2C_SCL_UP();
    I2c_delay();
    I2C_SCL_DOWM();
    I2C_SDA_UP();

}

void I2C_NACK(void)
{
    // I2C_SDA_UP();
    // I2c_delay();
    // I2C_SCL_UP();
    // I2c_delay();
    // I2C_SCL_DOWM();
    // I2c_delay();
    I2C_SCL_DOWM();
    I2c_delay();
    I2C_SDA_UP();
    I2c_delay();
    I2C_SCL_UP();
    I2c_delay();
    I2C_SCL_DOWM();
    I2C_SDA_UP();
}

//MSB and than LSB
void I2C_write_bus(uint8_t data)
{
    for(uint8_t i=0; i<8; i++)
    {
        // I2C_SCL_DOWM();
        if(data & (0x80>>i))
        {
            I2C_SDA_UP();
        }
        else
        {
            I2C_SDA_DOWM();
        }
        I2c_delay();
        I2C_SCL_UP();
        I2c_delay();
        I2C_SCL_DOWM();
        
        
    }
    // I2C_SCL_DOWM();
    I2c_delay();
}

void I2C_write_data_u8(uint8_t data)
{
    I2C_write_bus(data);

}

void I2C_write_data_u16(uint16_t data)
{
    I2C_write_data_u8(data>>8);
    I2C_wait_ACK();
    I2C_write_data_u8(data);

}

uint8_t I2C_read_bus(void)
{
    uint8_t buff = 0x00;
    for(uint8_t i=0; i<8; i++)
    {
        // I2c_delay();
        I2C_SCL_DOWM();
        I2c_delay();
        I2C_SCL_UP();
        I2c_delay();
        // if(LL_GPIO_GetPinPull(I2C_SDA_PORT,I2C_SDA) == 1)
        // {
        //     buff |= (1<<(i-1));
        // }
        // else
        // {
        //     continue;
        // }
        buff = (buff<<1)|I2C_SDA_READ;
        // I2C_SCL_UP();
        I2c_delay();
    }
    I2C_SCL_DOWM();
    I2c_delay();
    return buff;
}

uint8_t I2C_read_data_u8(void)
{
    return I2C_read_bus();
}

uint16_t I2C_read_data_u16(void)
{
    uint16_t buff = 0x0000;
    buff |= (I2C_read_bus() << 8);
    I2C_ACK();
    buff |= I2C_read_bus();
    // buff<<=8;
    // buff |= I2C_read_bus();

    return buff;
}

uint8_t AS_read_reg_u8(uint8_t reg)
{
    uint8_t buff;
    I2C_strat();
    I2C_write_data_u8(0x36<<1);
    I2C_wait_ACK();
    I2C_write_data_u8(reg);
    I2C_wait_ACK();
    I2C_strat();
    I2C_write_data_u8((0x36<<1)|0x01);
    I2C_wait_ACK();
    buff = I2C_read_data_u8();
    I2C_NACK();
    I2C_end();

    return buff;
}

uint16_t AS_read_reg_u16(uint8_t reg)
{
    uint16_t buff;
    I2C_strat();
    I2C_write_data_u8(0x36<<1);
    I2C_wait_ACK();
    I2C_write_data_u8(reg);
    I2C_wait_ACK();
    I2C_strat();
    I2C_write_data_u8((0x36<<1)|0x01);
    // I2C_wait_ACK();
    buff = I2C_read_data_u16();
    I2C_NACK();
    I2C_end();

    return buff;
}

void AS_write_reg_u8(uint8_t reg, uint8_t data)
{
    I2C_strat();
    I2C_write_data_u8(0x36<<1);
    I2C_wait_ACK();
    I2C_write_data_u8(reg);
    I2C_wait_ACK();
    I2C_write_data_u8(data);
    I2C_wait_ACK();
    // I2C_NACK();
    I2C_end();

    // return buff;
}

void AS_write_reg_u16(uint8_t reg, uint16_t data)
{
    I2C_strat();
    I2C_write_data_u8(0x36<<1);
    I2C_wait_ACK();
    I2C_write_data_u8(reg);
    I2C_wait_ACK();
    I2C_write_data_u16(data);
    I2C_wait_ACK();
    // I2C_NACK();
    I2C_end();

    // return buff;
}

uint16_t AS_angle_read(void)
{   
    uint16_t buff=0x0000;
    float angle = 0;
    buff |= (AS_read_reg_u8(0x0E)<<8);
    I2C_ACK();
    buff |= AS_read_reg_u8(0x0D);
    
    return buff;
}

int16_t angle[3];
uint16_t AS_angle_read_filter(void)
{
    
    uint16_t x_12,x_13,x_23;
    angle[2] =(int16_t)AS_angle_read();
    x_12 = fabs(angle[0]-angle[1]);
    x_13 = fabs(angle[0]-angle[2]);
    x_23 = fabs(angle[1]-angle[2]);

    if((fabs(x_23-x_12)>=150&&fabs(x_23-x_12)<=3000) ||(fabs(x_12-x_13>=150&&fabs(x_13-x_12)<=3000)))
    {
        angle[2] = angle[1];
    }
    angle[0] = angle[1];
    angle[1] = angle[2];
    return angle[2];

}



