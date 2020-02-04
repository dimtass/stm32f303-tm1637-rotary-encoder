#ifndef __TM1637_H
#define __TM1637_H

#ifdef __cplusplus
 extern "C" {
#endif 

#define DECLARE_TM1637(NAME, CLK_PORT, CLK_PIN, DIO_PORT, DIO_PIN, DELAY_US) \
	struct tp_tm1637 NAME = { \
		.clk_port = CLK_PORT, \
		.clk_pin = CLK_PIN, \
		.dio_port = DIO_PORT, \
		.dio_pin = DIO_PIN, \
		.delay_us = DELAY_US, \
    }

struct tp_tm1637 {
	GPIO_TypeDef*   clk_port;
    uint16_t        clk_pin;
	GPIO_TypeDef*   dio_port;
    uint16_t        dio_pin;
    uint16_t        delay_us;
};

void tm1637_init(struct tp_tm1637 * tm1637);
void tm1637_setBrightness(uint8_t brightness, uint8_t on);
void tm1637_showNumberDec(int num, uint8_t leading_zero, uint8_t length, uint8_t pos);
void tm1637_showNumberDecEx(int num, uint8_t dots, uint8_t leading_zero,
                                    uint8_t length, uint8_t pos);

void tm1637_showNumberHexEx(uint16_t num, uint8_t dots, uint8_t leading_zero,
                                    uint8_t length, uint8_t pos);
void tm1637_showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots, uint8_t leading_zero,
                                    uint8_t length, uint8_t pos);


#ifdef __cplusplus
}
#endif

#endif /* __TM1637_H */