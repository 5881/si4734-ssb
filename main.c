/* 9 мая 2021 работает настройка, переключение ам/fm
 * начата разработка интерфейса
 * 10 мая 2021 SI4734 поддерживает SSB с тем же патчем что и SI4735!!! 
 * В качестве контроллера использован stm32f030
 */



/**********************************************************************
 * Секция include и defines
**********************************************************************/
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/timer.h>
#include "oledi2c.h"
#include "oled_printf.h"
#include "si4734.h"
#include "patch_init.h"

#define AM_MODE 0
#define FM_MODE 1
#define SSB_MODE 2

uint16_t MIN_LIMIT=200;
uint16_t  MAX_LIMIT=30000;
//#define IF_FEQ 455

uint16_t encoder=7055;
uint16_t pwm1=750;
uint16_t coef=5;
uint8_t encoder_mode=2;
int16_t bfo=0;

/*
11 метров, 25.60 — 26.10 МГц (11,72 — 11,49 метра).
13 метров, 21.45 — 21.85 МГц (13,99 — 13,73 метра).
15 метров, 18.90 — 19.02 МГц (15,87 — 15,77 метра).
16 метров, 17.55 — 18.05 МГц (17,16 — 16,76 метра).
19 метров, 15.10 — 15.60 МГц (19,87 — 18,87 метра).
22 метра, 13.50 — 13.87 МГц (22,22 — 21,63 метра).
25 метров 11.60 — 12.10 МГц (25,86 — 24,79 метра).
31 метр, 9.40 — 9.99 МГц (31,91 — 30,03 метра).
41 метр, 7.20 — 7.50 МГц (41,67 — 39,47 метра).
49 метров, 5.85 — 6.35 МГц (52,36 — 47,66 метра).
60 метров, 4.75 — 5.06 МГц (63,16 — 59,29 метра).
75 метров, 3.90 — 4.00 МГц (76,92 — 75 метров).
90 метров, 3.20 — 3.40 МГц (93,75 — 88,24 метров).
120 метров (средние волны), 2.30 — 2.495 МГц (130,43 — 120,24 метра).
*/
uint16_t bands[]={200,1000,3000,5800,7200,9300,11200,13500,14200,15100,17450,21500,27000};
uint8_t steps[]={1,5,10,50};
uint8_t reciver_mode=0;
//0 - am, 1 -fm, 2 - ssb

void reciver_set_mode(uint8_t rec_mod){
si4734_powerdown();
if(!rec_mod) {
	//o_printf("AM mode\n");
	reciver_mode=0;
	si4734_am_mode();
	si4734_set_prop(AM_CHANNEL_FILTER, 0x0100);
	si4734_set_prop(AM_SEEK_BAND_TOP, 23000);
	MIN_LIMIT=200;
	encoder=17450;
	MAX_LIMIT=30000;
	coef=5;
	encoder_mode=0;
	} else if(rec_mod==FM_MODE){
	//oled_clear();
	//o_printf("FM mode\n");
	reciver_mode=FM_MODE;
	si4734_fm_mode();
	si4734_set_prop(FM_DEEMPHASIS,0x0001);//01 = 50 µs. Used in Europe, Australia, Japan
	MIN_LIMIT=6000;
	MAX_LIMIT=11100;
	coef=5;
	encoder=8910;
	encoder_mode=0;
	} else {
	reciver_mode=SSB_MODE;
	si4734_ssb_patch_mode(ssb_patch_content);
	si4734_set_prop(0x0101,((1<<15)|(1<<12)|(1<<4)|2));//ssb man page 24
	//o_printf_at(0,0,1,0,"ssb status: x%x",status);
	//si4734_set_prop(AM_SEEK_BAND_TOP, 23000);
	MIN_LIMIT=200;
	encoder=7100;
	MAX_LIMIT=30000;
	coef=1;
	encoder_mode=0;
	}
	
}

void select_band(int8_t direction){
	static int8_t band=5;
	band+=direction;
	if(band<0)band=0;
	if(band>12) band=12;
	encoder=bands[band];
	}
void select_step(int8_t direction){
	static int8_t step=1;
	step+=direction;
	if(step<0)step=0;
	if(step>4)step=4;
	coef=steps[step];
	}





void exti_encoder_init(){
		/*	энкодер с подтяжкой к питанию.
	 * PA0 энкодер 1
	 * PA1 энклдер 2
	 * PA2 кнопка энкодера
	 */
	//Enable GPIOA clock.
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT,
					GPIO_PUPD_NONE, GPIO0|GPIO1|GPIO2);
	exti_select_source(EXTI0,GPIOA);
	//Прерывания по ноге PA0
	exti_set_trigger(EXTI0,EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI0);
	nvic_enable_irq(NVIC_EXTI0_1_IRQ);
	}


	
void exti0_1_isr(void){
	//indicate(2);
	//o_printf_at(0,6,1,0,"ISR_8_pin9=%x",((gpio_get(GPIOA,GPIO7))>>7));
	int8_t encoder_direction;
	if(gpio_get(GPIOA,GPIO1))encoder_direction=1;else encoder_direction=-1;
	//o_printf_at(0,6,1,0,"ISR_8_pin9=%x",((gpio_get(GPIOA,GPIO7))>>7));
	if(encoder_mode==0){
		encoder+=coef*encoder_direction;
		if(encoder<MIN_LIMIT)encoder=MIN_LIMIT;
		if(encoder>MAX_LIMIT)encoder=MAX_LIMIT;
		}
	//if(encoder_mode==1){
	//	pwm1+=10*encoder_direction;
	//	if(pwm1<100)pwm1=100;
	//	if(pwm1>3300)pwm1=3300;
	//	timer_set_oc_value(TIM3, TIM_OC4, pwm1);
	//	}
	if(encoder_mode==1)select_band(encoder_direction);
	if(encoder_mode==2)select_step(encoder_direction);
	if(encoder_mode==3)bfo+=50*encoder_direction;
	exti_reset_request(EXTI0);
	}	


static void i2c_setup(void){
	/* Enable clocks for I2C1 and AFIO. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_I2C1);
	/* Set alternate functions for the SCL and SDA pins of I2C1.
	 * SDA PA10
	 * SCL PA9
	 *  */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9|GPIO10);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD,
                    GPIO_OSPEED_2MHZ, GPIO9|GPIO10);
    gpio_set_af(GPIOA,GPIO_AF4,GPIO9|GPIO10);//ремапинг на i2c
	
	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(I2C1);
	i2c_set_speed(I2C1,i2c_speed_fm_400k,8);
	i2c_peripheral_enable(I2C1);
}	



void rcc_clock_setup_in_hsi_out_64mhz(void)
 {
         rcc_osc_on(RCC_HSI);
         rcc_wait_for_osc_ready(RCC_HSI);
         rcc_set_sysclk_source(RCC_HSI);
  
         rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
         rcc_set_ppre(RCC_CFGR_PPRE_NODIV);
  
         flash_prefetch_enable();
         flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);
  
         /* 8MHz * 12 / 2 = 48MHz */
         rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL16);
         rcc_set_pll_source(RCC_CFGR_PLLSRC_HSI_CLK_DIV2);
  
         rcc_osc_on(RCC_PLL);
         rcc_wait_for_osc_ready(RCC_PLL);
         rcc_set_sysclk_source(RCC_PLL);
  
         rcc_apb1_frequency = 64000000;
         rcc_ahb_frequency = 64000000;
 }

void rcc_clock_setup_in_hsi_out_8mhz(void)
 {
         rcc_osc_on(RCC_HSI);
         rcc_wait_for_osc_ready(RCC_HSI);
         rcc_set_sysclk_source(RCC_HSI);
  
         rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
         rcc_set_ppre(RCC_CFGR_PPRE_NODIV);
  
         flash_prefetch_enable();
         flash_set_ws(FLASH_ACR_LATENCY_000_024MHZ);
  
         /* 8MHz * 4 / 2 = 16MHz */
         rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL2);
         rcc_set_pll_source(RCC_CFGR_PLLSRC_HSI_CLK_DIV2);
  
         rcc_osc_on(RCC_PLL);
         rcc_wait_for_osc_ready(RCC_PLL);
         rcc_set_sysclk_source(RCC_PLL);
  
         rcc_apb1_frequency = 8000000;
         rcc_ahb_frequency = 8000000;
 }

void led_setup(){
	//отладочный светодиод
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT,
					GPIO_PUPD_NONE, GPIO0);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP,
                    GPIO_OSPEED_2MHZ, GPIO0);
                    gpio_set(GPIOB, GPIO0);
    }


void gpio_setup(){
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT,
					GPIO_PUPD_NONE, GPIO11);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP,
                    GPIO_OSPEED_2MHZ, GPIO11);
                    gpio_clear(GPIOA, GPIO11);
    //rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT,
					GPIO_PUPD_NONE, GPIO3);
					
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT,
					GPIO_PUPD_NONE, GPIO5|GPIO6);
	}
	
	

void print_prop(uint16_t prop){
	o_printf("prop %x is 0x%x\n",prop, si4734_get_prop(prop));
	
}

uint8_t get_recivier_signal_status(uint8_t *snr,uint8_t *rssi,uint8_t *freq_of){
	uint8_t status,resp1,resp2;
	switch(reciver_mode){
		case AM_MODE: status=si4734_am_signal_status(&resp1,&resp2,rssi,snr);
			break;
		case FM_MODE: status=si4734_fm_signal_status(rssi,snr,freq_of);
			break;
		case SSB_MODE: status=si4734_am_signal_status(&resp1,&resp2,rssi,snr);
			break;
		}
	return status;
	}
/********************************************************************
 * Секция функций оформления дисплея
 ********************************************************************/
void logo(void){
	o_printf_at(0,0,1,0,"The best AM/FM tuner!");
	}
void show_freq(uint16_t freq){
	//NB колонки в пиксилях, а строки по 8 пикселей, отсчёт с нуля из
	//верхнего левого угла
	
	o_printf_at(0,1,3,0,"%5d",freq);
	if(reciver_mode==FM_MODE)o_printf_at(18*5,1,1,0,"x10");
		else o_printf_at(18*5,1,1,0,"   ");
	o_printf_at(18*5,2,1,0,"KHz");
	
	}
void show_reciver_status(uint8_t snr, uint8_t rssi, uint8_t status){
	o_printf_at(1,4,1,0,"SNR:%2ddB SI: %2duVdB",snr,rssi);
	o_printf_at(1,5,1,0,"status x%x",status);
	}

void show_reciver_full_status(uint16_t freq,
				uint8_t snr, uint8_t rssi, uint8_t status){
	show_freq(freq);
	show_reciver_status(snr,rssi,status);
	}

/*********************************************************************
 * ssb mode test
 *********************************************************************/





void main(){
	rcc_clock_setup_in_hsi_out_48mhz();
	gpio_setup();
	exti_encoder_init();
	i2c_setup();
	oled_init();
	oled_clear();
	//o_printf("oled init\n");
	//logo();
	uint32_t old_enc=0;
	int16_t old_bfo=0;
	uint32_t count=0;
	uint8_t status,rev,snr,rssi,resp1,resp2;
	int8_t freq_of;
	uint16_t j=0,freq;
	uint32_t old_encoder=0;
	//test();
	//o_printf("si4734 init start");
	si4734_reset();
	for(uint32_t i=0;i<0x5ff;i++)__asm__("nop");
	//status=si4734_am_mode();
	//status=si4734_fm_mode();
	//si4734_set_prop(AM_CHANNEL_FILTER, 0x0100);
	//o_printf("init status: %x\n",status);
	reciver_set_mode(AM_MODE);
	
	while(1){
		logo();
		//spi_send8(SPI1,j++);
		if(!gpio_get(GPIOB,GPIO5) && reciver_mode==AM_MODE){
			while(!gpio_get(GPIOB,GPIO5))__asm__("nop");
			si4734_am_seek(1);
			status=si4734_get_freq(&freq, &snr, &rssi);
			encoder=freq;
			old_encoder=freq;
			}
		if(!gpio_get(GPIOA,GPIO3)){
			while(!gpio_get(GPIOA,GPIO3))__asm__("nop");
			reciver_mode++;
			if (reciver_mode>2) reciver_mode=0;
			reciver_set_mode(reciver_mode);
			}
		
		if(!gpio_get(GPIOA,GPIO2)){
			while(!gpio_get(GPIOA,GPIO2))__asm__("nop");
			if(reciver_mode==1)reciver_set_mode(0); else encoder_mode++; 
			if(encoder_mode>2 && !reciver_mode) encoder_mode=0;
			if(encoder_mode>3) encoder_mode=0;
			o_printf_at(1,7,1,0,"encoder mode: %d",encoder_mode);
			}
				
		if(old_encoder!=encoder){
			old_encoder=encoder;
			if(reciver_mode==1) status=si4734_fm_set_freq(encoder);
			else if(reciver_mode==2) status=si4734_ssb_set_freq(encoder);
				else status=si4734_am_set_freq(encoder);
				}
		show_freq(encoder);	
		
		if(old_bfo!=bfo && reciver_mode==2){si4734_set_prop(SSB_BFO, bfo);
						o_printf_at(1,6,1,0,"BFO: %5d",bfo);
						//si4734_ssb_set_freq(encoder);
						old_bfo=bfo;
						}
		
		//if(!reciver_mode) status=si4734_am_signal_status(&resp1,&resp2,&rssi,&snr);
		//else if(reciver_mode==1) status=si4734_fm_signal_status(&rssi,&snr,&freq_of);
		//	else if(reciver_mode==2) status=si4734_am_signal_status(&resp1,&resp2,&rssi,&snr);
		get_recivier_signal_status(&snr,&rssi,&freq_of);
		show_reciver_full_status(encoder,snr,rssi,status);
		for(uint32_t i=0;i<0x5ffff;i++)__asm__("nop");
		
		
		}
	}

