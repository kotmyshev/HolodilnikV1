// STM32F407VGT6, 12 MHz, LCD1602, ADC for thermoresistors

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_exti.h"
#include "misc.h"



#define ICE_PINS (GPIO_Pin_0 | GPIO_Pin_1);   			// pins for ice detectors

#define COMAND GPIO_ResetBits(GPIOC, GPIO_Pin_0); 	// comand mode for display
#define WRDATA GPIO_SetBits(GPIOC, GPIO_Pin_0); 		// data mode for display

#define LEDON GPIO_SetBits(GPIOD, GPIO_Pin_15);	 		// blue Led LD6 light On
#define LEDOFF GPIO_ResetBits(GPIOD, GPIO_Pin_15); 	// blue Led LD6 light Off


unsigned int koefficient = 29600 / 0xfff; 					// Koeff 29600/4095 depends by selected resistors
unsigned int allow_err = 5; 												// allowable error in percentage

void Delay(unsigned int Val) { 
	for (; Val != 0; Val--); 
}

void SEND (unsigned char SDAT	)								// Display Write Function with strobe
{	
GPIOD->ODR |= SDAT;									// WR DATA to ODR Registr Port D
	
Delay(10);
GPIO_SetBits(GPIOC, GPIO_Pin_2); 		// Generation STROB signal on PC02
Delay(10);
GPIO_ResetBits(GPIOC, GPIO_Pin_2);	
Delay(10);	
GPIOD->ODR &= ~0xFF;								// Reset Data Bits (Port D)
}

char ConvertChar_LCD ( unsigned int Zifra) 					// Convert Decimal 0-9 Number into Display CODE
 {
		char DIG = 0x00;
	 
	  if (Zifra > 9) Zifra = 9;
	  if (Zifra < 1) Zifra = 0;
		if (Zifra == 0) DIG = 0x30;
		if (Zifra == 1) DIG = 0x31;
	 	if (Zifra == 2) DIG = 0x32;
	 	if (Zifra == 3) DIG = 0x33;
	 	if (Zifra == 4) DIG = 0x34;
	 	if (Zifra == 5) DIG = 0x35;
	 	if (Zifra == 6) DIG = 0x36;
	 	if (Zifra == 7) DIG = 0x37;
	 	if (Zifra == 8) DIG = 0x38;
	 	if (Zifra == 9) DIG = 0x39;
		
	 return DIG;
}
 

void gpioa_init() {
	// init gpios to Analog Mode for working with ADC1 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
	
  GPIO_InitTypeDef  gpioA; 	
	gpioA.GPIO_Mode = GPIO_Mode_AN;  	
	gpioA.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4; 
	GPIO_Init(GPIOA, &gpioA); 
}  

void lcd_init(void) { 

	GPIO_InitTypeDef gpio_c_i; 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  	
	gpio_c_i.GPIO_Mode = GPIO_Mode_OUT;  	
	gpio_c_i.GPIO_Speed = GPIO_Speed_100MHz;  	
	gpio_c_i.GPIO_PuPd = GPIO_PuPd_NOPULL;  	
	gpio_c_i.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_0; 
	GPIO_Init(GPIOC, &gpio_c_i); 
	
	GPIO_InitTypeDef gpio_d_i; 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  	
	gpio_d_i.GPIO_Mode = GPIO_Mode_OUT;  	
	gpio_d_i.GPIO_Speed = GPIO_Speed_100MHz;  	
	gpio_d_i.GPIO_PuPd = GPIO_PuPd_NOPULL;  	
	gpio_d_i.GPIO_Pin = GPIO_Pin_All; 
	GPIO_Init(GPIOD, &gpio_d_i); 

	GPIO_ResetBits(GPIOC, GPIO_Pin_1);	 // Pre-Setup Write Only State Display
	
	Delay(700000); 						// Display Start Time >20 ms
	COMAND; 												
	SEND(0x38) ; 										// Function Set: 8-bit, 2-line
	Delay(1500);							// Wait >37 us
	SEND(0x0C) ; 										// Display Set (No Cursor & No Blinking)
	Delay(1500);							// Wait >37 us
	SEND(0x6) ; 										// Entery Mode Set (Right-Moving Cursor)
	Delay(1500);							// Wait >37 us
	SEND(0x1) ; 										// Display Clear
	Delay(51000);							// Wait >1.53 ms
} 


void enable_interrupt(IRQn_Type irq) 
	{  	
		NVIC_InitTypeDef nvic_init;
  	nvic_init.NVIC_IRQChannel = irq; 
		nvic_init.NVIC_IRQChannelCmd = ENABLE; 
		nvic_init.NVIC_IRQChannelSubPriority = 1; 
		nvic_init.NVIC_IRQChannelPreemptionPriority = 1; 
		
	  NVIC_Init(&nvic_init); 
} 


void init_ice_buttons(void) { 
	GPIO_InitTypeDef gpio_init; 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  	
	gpio_init.GPIO_Mode = GPIO_Mode_IN;  	
	gpio_init.GPIO_Speed = GPIO_Speed_100MHz;  	
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;  	
	gpio_init.GPIO_Pin = ICE_PINS; 
	GPIO_Init(GPIOB, &gpio_init); 
	
	enable_interrupt(EXTI0_IRQn);  	
	enable_interrupt(EXTI1_IRQn);  
} 


void adc_init() { 
	ADC_InitTypeDef ADC_InitStructure; 
	ADC_CommonInitTypeDef adc_init; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	ADC_DeInit(); 

	ADC_StructInit(&ADC_InitStructure);  	
	adc_init.ADC_Mode = ADC_Mode_Independent;  	
	adc_init.ADC_Prescaler = ADC_Prescaler_Div2; 

	ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;  
	ADC_CommonInit(&adc_init); 
	ADC_Init(ADC1, &ADC_InitStructure); 
	ADC_Cmd(ADC1, ENABLE); 
}  
unsigned int readADC1(unsigned short channel) { 
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_3Cycles); 
	ADC_SoftwareStartConv(ADC1); 
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);  	
	return ADC_GetConversionValue(ADC1); 
}  

void EXTI0_IRQHandler(void) { 
	EXTI_ClearITPendingBit(EXTI_Line0);  
		LEDON;
}

void EXTI1_IRQHandler(void) { 
	EXTI_ClearITPendingBit(EXTI_Line1);  
		LEDON;
}
  
int main(void) { 
	gpioa_init(); 
	init_ice_buttons();
	adc_init(); 
	lcd_init();
	
	
	do { 
		unsigned int holod_setup = readADC1(ADC_Channel_1);
		unsigned int moroz_setup = readADC1(ADC_Channel_2);
		unsigned int holod_sense = readADC1(ADC_Channel_3);
		unsigned int moroz_sense = readADC1(ADC_Channel_4);
		
		unsigned int Temp_holod_set = holod_setup *  koefficient;
		unsigned int Temp_moroz_set = moroz_setup *  koefficient;
		unsigned int Temp_holod_now = holod_sense *  koefficient;
		unsigned int Temp_moroz_now = moroz_sense *  koefficient;
		
		
		unsigned int h_allow_min = Temp_holod_set - Temp_holod_set * allow_err/100;
		unsigned int h_allow_max = Temp_holod_set + Temp_holod_set * allow_err/100;
		
		unsigned int m_allow_min = Temp_moroz_set - Temp_holod_set * allow_err/100;
		unsigned int m_allow_max = Temp_moroz_set + Temp_holod_set * allow_err/100;
		
		
		if ( Temp_holod_now > h_allow_max | Temp_holod_now < h_allow_min) 
		{LEDON} else {LEDOFF};
		
		if ( Temp_moroz_now > m_allow_max | Temp_holod_now < m_allow_min) 
		{LEDON} else {LEDOFF};
		
	
		COMAND; 
		SEND(0x80);
		WRDATA;
		SEND(ConvertChar_LCD(Temp_holod_now / 1000));
    SEND(ConvertChar_LCD((Temp_holod_now % 1000) / 100));
    SEND(ConvertChar_LCD((Temp_holod_now % 1000) % 100));
    SEND(0x2E);
    SEND(ConvertChar_LCD(((Temp_holod_now % 1000) % 100) / 10));
		
		COMAND; 
		SEND(0xC0);
		WRDATA;
		SEND(ConvertChar_LCD(Temp_moroz_now / 1000));
    SEND(ConvertChar_LCD((Temp_moroz_now % 1000) / 100));
    SEND(ConvertChar_LCD((Temp_moroz_now % 1000) % 100));
    SEND(0x2E);
    SEND(ConvertChar_LCD(((Temp_moroz_now % 1000) % 100) / 10));
		
		
		
		Delay(500000); 
	} while (1); 
}
