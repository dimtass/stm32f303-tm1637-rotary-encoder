#include <stdio.h>
#include "stm32f30x.h"
#include "debug_trace.h"
#ifdef USE_DBGUART
#include "dev_uart.h"
#endif
#ifdef USE_STTERM
#include "stlinky.h"
#endif
#include "mod_led.h"
#include "timer_sched.h"
#include "tm1637.h"
#include "cortexm_delay.h"
#include "rotary_enc_pot.h"

#define LED_TIMER_MS 500
#define LED_PORT GPIOC
#define LED_PIN GPIO_Pin_13

#define ROT_ENC_PORT	GPIOB
#define ROT_ENC_PINA	GPIO_Pin_0
#define ROT_ENC_PINB	GPIO_Pin_1

struct tp_pot_values {
	uint8_t a;
	uint8_t b;
};

volatile uint32_t glb_tmr_1ms;
uint32_t trace_levels;
struct rep_pot pot1;
volatile uint8_t read_pot = 0;
volatile struct tp_pot_values pot_values;

/* Create the list head for the timer */
static LIST_HEAD(obj_timer_list);

// Declare uart
#ifdef USE_DBGUART
DECLARE_UART_DEV(dbg_uart, USART1, 115200, 256, 10, 1);
#endif

#ifdef USE_SEMIHOSTING
extern void initialise_monitor_handles(void);
#endif

#ifdef USE_OVERCLOCKING
extern uint32_t overclock_stm32f303(void);
#endif

static inline void main_loop(void)
{
	/* 1 ms timer */
	if (glb_tmr_1ms) {
		glb_tmr_1ms = 0;
		mod_timer_polling(&obj_timer_list);
	}
}

void led_on(void *data)
{
	LED_PORT->ODR |= LED_PIN;
}

void led_off(void *data)
{
	LED_PORT->ODR &= ~LED_PIN;
}

void led_init(void *data)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);

	LED_PORT->ODR |= LED_PIN;
	TRACE(("init\n"));
}

void rotary_encoder_init()
{	
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure Button pin as input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure rotary encoder */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/* Enable and set EXTI15_10 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	read_pot = 0;
	pot_values.a = 0;
	pot_values.b = 0;
}

void rotary_encoder_cbk(rep_val_t value)
{
	printf("POT1: %d\n", value);
    tm1637_showNumberDec(value, 1, 4, 0);  // Expect: 0000
}

void EXTI1_IRQHandler(void)
{
	if((EXTI_GetITStatus(EXTI_Line1) != RESET))
	{
		/* Do not handle the display here, just read the pins
			and trigger a flag for post-processing (see main() while loop).
			This flag will block faster updates than the display handling.
		*/
		if (!read_pot) {
			read_pot = 1;
			pot_values.a = (uint8_t)(ROT_ENC_PORT->IDR & ROT_ENC_PINA);
			pot_values.b = (uint8_t)(ROT_ENC_PORT->IDR & ROT_ENC_PINB);
		}
		/* Clear the EXTI line 6 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

int main(void)
{
#ifdef USE_OVERCLOCKING
    SystemCoreClock = overclock_stm32f303();
#endif
	if (SysTick_Config(SystemCoreClock / 1000)) {
		/* Capture error */
		while (1);
	}
	delay_init(SystemCoreClock);

    trace_levels_set(
			0
			| TRACE_LEVEL_DEFAULT
			,1);

#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
#elif USE_STTERM
	stlinky_init();
#elif USE_DBGUART
	// setup uart port
	dev_uart_add(&dbg_uart);
	// set callback for uart rx
 	dbg_uart.fp_dev_uart_cb = NULL;
 	mod_timer_add((void*) &dbg_uart, 5, (void*) &dev_uart_update, &obj_timer_list);
#endif

	/* Declare LED module and initialize it */
	DECLARE_MODULE_LED(led_module, 8, 250);
	mod_led_init(&led_module);
	mod_timer_add((void*) &led_module, led_module.tick_ms, (void*) &mod_led_update, &obj_timer_list);

	/* Declare LED */
	DECLARE_DEV_LED(def_led, &led_module, 1, NULL, &led_init, &led_on, &led_off);
	dev_led_add(&def_led);
	dev_led_set_pattern(&def_led, 0b11001100);

	/* Configure TM1637 */
	DECLARE_TM1637(tm1637, GPIOB, GPIO_Pin_9, GPIOB, GPIO_Pin_8, 80);
	tm1637_init(&tm1637);
    tm1637_setBrightness(0x0f, 1);
    tm1637_showNumberDec(0, 1, 4, 0);  // Expect: 0000

	/* Initialize and configure rotary encoder */
	rotary_encoder_init();
	rep_init(&pot1, 0, -100, 100, 1, 0, &rotary_encoder_cbk);

	TRACE(("Program started\n"));

	/* main loop */
	while (1) {
		main_loop();
		if (read_pot) {
			/* First process the display and then clear the flag */
			rep_set_update_values(&pot1, pot_values.a, pot_values.b);
			read_pot = 0;
		}
	}
}