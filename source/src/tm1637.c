#include <stddef.h>
#include "stm32f30x.h"
#include "tm1637.h"
#include "cortexm_delay.h"

#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80

//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t digitToSegment[] = {
    // XGFEDCBA
    0b00111111,    // 0
    0b00000110,    // 1
    0b01011011,    // 2
    0b01001111,    // 3
    0b01100110,    // 4
    0b01101101,    // 5
    0b01111101,    // 6
    0b00000111,    // 7
    0b01111111,    // 8
    0b01101111,    // 9
    0b01110111,    // A
    0b01111100,    // b
    0b00111001,    // C
    0b01011110,    // d
    0b01111001,    // E
    0b01110001     // F
};

GPIO_InitTypeDef GPIO_InitStruct_CLK;
GPIO_InitTypeDef GPIO_InitStruct_DIO;

static const uint8_t minusSegments = 0b01000000;
static uint8_t m_brightness = 0;
static struct tp_tm1637 * m_tm1637 = NULL;

void tm1637_start();
void tm1637_stop();
void tm1637_clear();
uint8_t tm1637_writeByte(uint8_t b);
void tm1637_showDots(uint8_t dots, uint8_t* digits);
uint8_t tm1637_encodeDigit(uint8_t digit);

void tm1637_init(struct tp_tm1637 * tm1637)
{
    m_tm1637 = tm1637;

    if ((m_tm1637->clk_port == GPIOA) || (m_tm1637->dio_port == GPIOA)) {
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    }
    else if ((m_tm1637->clk_port == GPIOB) || (m_tm1637->dio_port == GPIOB)) {
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    }
    else if ((m_tm1637->clk_port == GPIOC) || (m_tm1637->dio_port == GPIOC)) {
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    }

	GPIO_InitStruct_CLK.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct_CLK.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct_CLK.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct_CLK.GPIO_OType = GPIO_OType_OD;

	GPIO_InitStruct_CLK.GPIO_Pin = m_tm1637->clk_pin;
	GPIO_Init(m_tm1637->clk_port, &GPIO_InitStruct_CLK);

	GPIO_InitStruct_DIO.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct_DIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct_DIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct_DIO.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct_DIO.GPIO_Pin = m_tm1637->dio_pin;
	GPIO_Init(m_tm1637->dio_port, &GPIO_InitStruct_DIO);

	// Set the pin direction and default value.
	// Both pins are set as inputs, allowing the pull-up resistors to pull them up
    m_tm1637->clk_port->ODR &= ~m_tm1637->clk_pin;
    m_tm1637->dio_port->ODR &= ~m_tm1637->dio_pin;
}

void tm1637_setBrightness(uint8_t brightness, uint8_t on)
{
	m_brightness = (brightness & 0x7) | (on? 0x08 : 0x00);
}

void tm1637_setSegments(const uint8_t * segments, uint8_t length, uint8_t pos)
{
    // Write COMM1
	tm1637_start();
	tm1637_writeByte(TM1637_I2C_COMM1);
	tm1637_stop();

	// Write COMM2 + first digit address
	tm1637_start();
	tm1637_writeByte(TM1637_I2C_COMM2 + (pos & 0x03));

	// Write the data bytes
	for (uint8_t k=0; k < length; k++)
	    tm1637_writeByte(segments[k]);

	tm1637_stop();

	// Write COMM3 + brightness
	tm1637_start();
	tm1637_writeByte(TM1637_I2C_COMM3 + (m_brightness & 0x0f));
	tm1637_stop();
}

void tm1637_clear()
{
    uint8_t data[] = { 0, 0, 0, 0 };
	tm1637_setSegments(data, sizeof(data), 0);
}

void tm1637_showNumberDec(int num, uint8_t leading_zero, uint8_t length, uint8_t pos)
{
    tm1637_showNumberDecEx(num, 0, leading_zero, length, pos);
}

void tm1637_showNumberDecEx(int num, uint8_t dots, uint8_t leading_zero,
                                    uint8_t length, uint8_t pos)
{
    tm1637_showNumberBaseEx(num < 0? -10 : 10, num < 0? -num : num, dots, leading_zero, length, pos);
}

void tm1637_showNumberHexEx(uint16_t num, uint8_t dots, uint8_t leading_zero,
                                    uint8_t length, uint8_t pos)
{
    tm1637_showNumberBaseEx(16, num, dots, leading_zero, length, pos);
}

void tm1637_showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots, uint8_t leading_zero,
                                    uint8_t length, uint8_t pos)
{
    uint8_t negative = 0;
	if (base < 0) {
	    base = -base;
		negative = 1;
	}

    uint8_t digits[4];

	if (num == 0 && !leading_zero) {
		// Singular case - take care separately
		for(uint8_t i = 0; i < (length-1); i++)
			digits[i] = 0;
		digits[length-1] = tm1637_encodeDigit(0);
	}
	else {
		//uint8_t i = length-1;
		//if (negative) {
		//	// Negative number, show the minus sign
		//    digits[i] = minusSegments;
		//	i--;
		//}
		
		for(int i = length-1; i >= 0; --i)
		{
		    uint8_t digit = num % base;
			
			if (digit == 0 && num == 0 && leading_zero == 0)
			    // Leading zero is blank
				digits[i] = 0;
			else
			    digits[i] = tm1637_encodeDigit(digit);
				
			if (digit == 0 && num == 0 && negative) {
			    digits[i] = minusSegments;
				negative = 0;
			}
			num /= base;
		}
		if(dots != 0)
		{
			tm1637_showDots(dots, digits);
		}
    }
    tm1637_setSegments(digits, length, pos);
}

void tm1637_bitDelay()
{
    delay_us(m_tm1637->delay_us);
}

void tm1637_start()
{
	GPIO_InitStruct_DIO.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(m_tm1637->dio_port, &GPIO_InitStruct_DIO);
    tm1637_bitDelay();
}

void tm1637_stop()
{
	GPIO_InitStruct_DIO.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(m_tm1637->dio_port, &GPIO_InitStruct_DIO);
	tm1637_bitDelay();
	GPIO_InitStruct_CLK.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(m_tm1637->clk_port, &GPIO_InitStruct_CLK);
	tm1637_bitDelay();
	GPIO_InitStruct_DIO.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(m_tm1637->dio_port, &GPIO_InitStruct_DIO);
	tm1637_bitDelay();
}

uint8_t tm1637_writeByte(uint8_t b)
{
    uint8_t data = b;

    // 8 Data Bits
    for(uint8_t i = 0; i < 8; i++) {
        // CLK low
        GPIO_InitStruct_CLK.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_Init(m_tm1637->clk_port, &GPIO_InitStruct_CLK);
        tm1637_bitDelay();

        // Set data bit
        if (data & 0x01) {
            GPIO_InitStruct_DIO.GPIO_Mode = GPIO_Mode_IN;
        }
        else {
            GPIO_InitStruct_DIO.GPIO_Mode = GPIO_Mode_OUT;
        }
        GPIO_Init(m_tm1637->dio_port, &GPIO_InitStruct_DIO);
        tm1637_bitDelay();

        // CLK high
        GPIO_InitStruct_CLK.GPIO_Mode = GPIO_Mode_IN;
        GPIO_Init(m_tm1637->clk_port, &GPIO_InitStruct_CLK);
        tm1637_bitDelay();
        data = data >> 1;
    }

    // Wait for acknowledge
    // CLK to zero
    GPIO_InitStruct_CLK.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(m_tm1637->clk_port, &GPIO_InitStruct_CLK);

	GPIO_InitStruct_DIO.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(m_tm1637->dio_port, &GPIO_InitStruct_DIO);
    tm1637_bitDelay();

    // CLK to high
    GPIO_InitStruct_CLK.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(m_tm1637->clk_port, &GPIO_InitStruct_CLK);
    tm1637_bitDelay();
    uint8_t ack = m_tm1637->dio_port->IDR & m_tm1637->dio_pin; //digitalRead(m_pinDIO);
    if (ack == 0) {
        GPIO_InitStruct_DIO.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_Init(m_tm1637->dio_port, &GPIO_InitStruct_DIO);
        tm1637_bitDelay();
    }

    GPIO_InitStruct_CLK.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(m_tm1637->clk_port, &GPIO_InitStruct_CLK);
    tm1637_bitDelay();

    return ack;
}

void tm1637_showDots(uint8_t dots, uint8_t* digits)
{
    for(int i = 0; i < 4; ++i)
    {
        digits[i] |= (dots & 0x80);
        dots <<= 1;
    }
}

uint8_t tm1637_encodeDigit(uint8_t digit)
{
	return digitToSegment[digit & 0x0f];
}
