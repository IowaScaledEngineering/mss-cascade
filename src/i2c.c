#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "mss-cascade.h"


static inline void sda_low() { I2C_SDA_DDR |= _BV(I2C_SDA); _delay_us(10); }
static inline void sda_high() { I2C_SDA_DDR &= ~_BV(I2C_SDA); _delay_us(10); }
static inline void scl_low() { I2C_SCL_PORT &= ~_BV(I2C_SCL); _delay_us(10); }
static inline void scl_high() { I2C_SCL_PORT |= _BV(I2C_SCL); _delay_us(10); }

void i2cStart(void)
{
	scl_high();
	sda_low();
	scl_low();
	sda_high();
}

void i2cStop(void)
{
	scl_low();
	sda_low();
	scl_high();
	sda_high();
}

uint8_t i2cWriteByte(uint8_t byte)
{
	uint8_t i = 0x80, ack;

	do
	{
		if(byte & i)
		{
			sda_high();
		}
		else
		{
			sda_low();
		}
		
		scl_high();
		scl_low();
		
		i >>= 1;
	} while(i);

	sda_high();  // Release SDA
	
	scl_high();
	if(I2C_SDA_PIN & _BV(I2C_SDA))
		ack = 0;
	else
		ack = 1;
	scl_low();

	return ack;
}

uint8_t i2cReadByte(uint8_t ack)
{
	uint8_t i, data = 0;

	for(i=0; i<8; i++)
	{
		data = data << 1;
		scl_high();
		if(I2C_SDA_PIN & _BV(I2C_SDA))
			data |= 0x01;
		scl_low();
	}
	
	if(ack)
		sda_low();
	scl_high();
	scl_low();
	sda_high();

	return data;
}

uint8_t writeByte(uint8_t addr, uint8_t cmd, uint16_t writeVal)
{
	uint8_t ack;
	
	i2cStart();
	
	i2cWriteByte(addr << 1);
	i2cWriteByte(cmd);
	ack = i2cWriteByte(writeVal);

	i2cStop();

	return ack;
}

uint8_t readWord(uint8_t addr, uint8_t cmd, uint16_t* data)
{
	uint8_t ack = 1;
	*data = 0xFFFF;
	
	i2cStart();
	
	ack &= i2cWriteByte(addr << 1);
	ack &= i2cWriteByte(cmd);

	i2cStart();

	ack &= i2cWriteByte((addr << 1) | 0x01);
	*data = i2cReadByte(1);
	*data |= ((uint16_t)i2cReadByte(0) << 8);
	
	i2cStop();

	return ack;
}

