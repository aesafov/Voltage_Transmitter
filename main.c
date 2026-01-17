/*
Пример взял тут:
https://narodstream.ru/urok-178-cmsis-stm32f1-spi-dma/
Урок 178. CMSIS. STM32F1. SPI. DMA
Переделал под себя.
*/
//#include "stm32f3xx.h"
#include "main.h"

// Макросы для работы с CYCCNT
#define DWT_CTRL    (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)
#define SCB_DEMCR   (*(volatile uint32_t*)0xE000EDFC)

void SystemClock_Config(void);
void TIM6_Init(void);
void ADC1_Init(void);
void SPI1_Init(void);
void DMA1_Init(void);
void GPIO_Init(void);
void UART_Init(void);
void SPI1_SendByte(uint8_t data);
void Delay(__IO uint32_t nCount);

uint16_t tx_data[] = {0xF000, 0x0000, 0x0000, 0x000F};
//uint16_t aTxBuff[32] = {0x5555, 0xffff, 0xffff, 0x555a, 0xffff, 0xffff, 0xffff, 0xffff, 
//						0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 
//						0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 
//						0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff};
//uint16_t aTxBuff[4] = {0x1999, 0x8666, 0x6679, 0x9878}; // 2018->0x07E2
//uint16_t aTxBuff[4] = {0x1999, 0x9867, 0x9879, 0x9998}; // 800->0x320
uint16_t aTxBuff[4] = {0x1999, 0x9867, 0x9879, 0x9999}; // 800->0x320 + 1 бит в конце "1"
//const uint16_t aTxBuff[4] = {0x1999, 0x9867, 0x9879, 0x9987}; // 801->0x321 + 1 бит в конце "1"
//const uint16_t aTxBuff[4] = {0x1998, 0x6799, 0x8678, 0x6679};
//const uint16_t aTxBuff[4] = {0x1998, 0x6799, 0x8666, 0x6667};
//const uint16_t aTxBuff[4] = {0x1998, 0x6799, 0x8679, 0x8799};
//const uint16_t aTxBuff[4] = {0x1998, 0x6799, 0x8667, 0x9867};
//const uint16_t aTxBuff[4] = {0x1998, 0x6799, 0x8678, 0x7999};

__IO uint32_t tmpreg;
uint8_t fl=0;
volatile uint16_t adc_value = 0;
uint32_t multiplier;
uint32_t result;
uint32_t result_11bit, result_10bit, result_8bit;

#define COUNT 1000
uint8_t fl_ADC_start;
uint16_t buff_data_ADC[COUNT];
uint32_t count_conversion;
uint16_t min_ADC_value;
uint16_t max_ADC_value;

uint32_t elapsed_cycles;
uint64_t test_data[100];
uint8_t test_data_count = 0;
uint8_t y = 0;

union Out_t
{
    //uint8_t output[8];
    uint16_t out_data[4];
    uint64_t output_64;
};
union Out_t data;

//-----------------------------------------------------------------------------

int main(void)
{
    SystemClock_Config();  // Настройка тактирования
    
    // Разрешить доступ к DWT
    SCB_DEMCR |= 0x01000000; // Set bit 24: Enable the debug monitor
    // Включить Cycle Counter
    DWT_CYCCNT = 0;          // Reset cycle counter
    DWT_CTRL |= 1;           // Set bit 0: Enable the cycle counter
    
    multiplier = (OUTPUT_MAX * SCALE_FACTOR) / (INPUT_MAX - INPUT_MIN);
    
    GPIO_Init();
    DMA1_Init();
	SPI1_Init();    
    TIM6_Init();
    ADC1_Init();
    UART_Init();
    
    // Запустить таймер
    TIM6->CR1 |= TIM_CR1_CEN;

    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR);     // Периферийный адрес (SPI DR)
    DMA1_Channel3->CMAR = (uint32_t)aTxBuff;         // Адрес буфера данных
    DMA1_Channel3->CNDTR = 4;
    DMA1_Channel3->CCR |= DMA_CCR_EN; // Запустить DMA
//    SPI1->CR1 |= SPI_CR1_CPHA;
    SPI1->CR1 |= SPI_CR1_SPE;     // Включить SPI
    SET_BIT(SPI1->CR2, SPI_CR2_TXDMAEN);
    
    tmpreg = 0;
    
    while (1)
    {        
        if(fl_ADC_start)	  
        {
//            for(volatile int i = 0; i < 5; i++) { __NOP(); }
            ADC1->CR2 |= ADC_CR2_SWSTART | ADC_CR2_EXTTRIG;
            
            // Ждать завершения преобразования
            while(!(ADC1->SR & ADC_SR_EOC));
            
            // Чтение результата
            //adc_value = ADC1->DR & 0xFFF; 
            
            buff_data_ADC[count_conversion] = ADC1->DR & 0xFFF; 
            count_conversion++;
			if(count_conversion >= COUNT)
			{
				fl_ADC_start = 0;
				count_conversion = 0;
				
				min_ADC_value = buff_data_ADC[COUNT/2];
				max_ADC_value = min_ADC_value;
				
				for (count_conversion = 0; count_conversion <= COUNT-1; count_conversion++)
				{
					if(min_ADC_value < buff_data_ADC[count_conversion]) min_ADC_value = buff_data_ADC[count_conversion];
					else if(max_ADC_value > buff_data_ADC[count_conversion]) max_ADC_value = buff_data_ADC[count_conversion];
				}
				
				// RS-485 на передачу
				GPIOF->ODR |= GPIO_ODR_7;
				
				for (int count_conversion = 0; count_conversion < COUNT - 1; count_conversion++) 
				{
					while (!(USART1->ISR & USART_ISR_TXE));
					USART1->TDR = (buff_data_ADC[count_conversion] >> 8) & 0xFF;
					
					while (!(USART1->ISR & USART_ISR_TXE));
					USART1->TDR = buff_data_ADC[count_conversion] & 0xFF;

				}	
                // RS-485 на передачу
				GPIOF->ODR &= ~GPIO_ODR_7;                
			}            
        }

//        // Если преобразование завершено
//        if(ADC1->SR & ADC_SR_EOC)
//        {
//            uint32_t start_cycles = DWT_CYCCNT;
//            
//            // Чтение результата
//            adc_value = ADC1->DR & 0xFFF;        
//            if (adc_value < INPUT_MIN) adc_value = INPUT_MIN;
//            if (adc_value > INPUT_MAX) adc_value = INPUT_MAX;
//            uint32_t temp = (adc_value - INPUT_MIN) * multiplier;
//            result = (int)(temp >> 16);
//            
//            uint16_t input = result;
//            
//            int8_t bit_pos;// Позиция в битовой последовательности
//            uint8_t bit;
//            uint8_t pattern;

//            data.output_64 = 0;
//            data.output_64 |= 0xCCC;
//            for(bit_pos = 11; bit_pos > 0; bit_pos--)
//            {
//                bit = (input >> bit_pos) & 1;
//                pattern = bit ? 0b0011 : 0b1100;  // если 1 → "0011", если 0 → "1100"
//                data.output_64 = data.output_64 << 4;
//                data.output_64 = data.output_64 | pattern;
//            }
//            bit = (input >> bit_pos) & 1;
//            pattern = bit ? 0b0011 : 0b1100;  // если 1 → "0011", если 0 → "1100"
//            data.output_64 = data.output_64 << 4;
//            data.output_64 = data.output_64 | pattern;
//            data.output_64 = data.output_64 << 1;
//            //Добавил при отладке на плате
//            data.output_64 = data.output_64 | 1;
//            
////            test_data[test_data_count] = data.output_64;
////            test_data_count++;
////            
////            if(test_data_count >= 100)
////            {
////                test_data_count = 0;
////                __NOP();
////            }
//            
//            aTxBuff[3] = sendData[y][0];
//            aTxBuff[2] = sendData[y][1];
//            aTxBuff[1] = sendData[y][2];
//            aTxBuff[0] = sendData[y][3];
//            y++;
//            if(y >=9) y = 0;
//            
//            
////            aTxBuff[3] = data.out_data[0];
////            aTxBuff[2] = data.out_data[1];
////            aTxBuff[1] = data.out_data[2];
////            aTxBuff[0] = data.out_data[3];
//            
//            uint32_t end_cycles = DWT_CYCCNT;
//            elapsed_cycles = end_cycles - start_cycles;
////            result_11bit = result >> 1;
////            result_10bit = result >> 2;
////            result_8bit = result >> 4;
//            GPIOF->ODR &= ~GPIO_ODR_7;
//        }
            
        
//      
            
//        GPIOA->ODR ^= GPIO_ODR_12;
        tmpreg++;
		
//        for(volatile int i = 0; i < 83; i++) { __NOP(); }
//        if(READ_BIT(DMA2->LISR, DMA_LISR_TCIF3) == (DMA_LISR_TCIF3))
//        {
//            WRITE_REG(DMA2->LIFCR, DMA_LIFCR_CTCIF3);
//        }
		__NOP();
    }
}
//-----------------------------------------------------------------------------
void SystemClock_Config(void)
{
    // Включить HSE
    RCC->CR |= RCC_CR_HSEON; 
    while(!(RCC->CR & RCC_CR_HSERDY));  // Ждать готовности HSE

    // Включить флэш
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR |= FLASH_ACR_LATENCY_1; // Two wait states, if 48 < SYSCLK ≤ 72 MHz

    // Настройка PLL
    // PLL: 12 / M * N
    // HSE = 12 MHz, M = 3, N = 16 → 64 MHz
    RCC->CFGR = 0;
//    RCC->CFGR |= (16U << RCC_CFGR_PLLMUL_Pos); // PLL input clock x 16   
//    RCC->CFGR2 = 2U << RCC_CFGR2_PREDIV_Pos; // HSE input to PLL divided by 3
    RCC->CFGR |= RCC_CFGR_PLLMUL16;    
    RCC->CFGR2 = RCC_CFGR2_PREDIV_DIV3;

    // Включить PLL
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));  // Ждать готовности PLL

    // Переключить системный клок на PLL
    RCC->CFGR &= ~RCC_CFGR_SW;          // Сбросить SW
    RCC->CFGR |= RCC_CFGR_SW_PLL;       // Выбрать PLL как источник
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Ждать переключения
}
//-----------------------------------------------------------------------------
void GPIO_Init(void)
{
    // Тактирование GPIOA и GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOFEN;

    // PA12 (SCK) - AF6
    GPIOA->MODER   &= ~GPIO_MODER_MODER12;
    GPIOA->MODER   |=  GPIO_MODER_MODER12_1;
//    GPIOA->AFR[1]  |=  (6U << (5*4));
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH4_Msk); // Сбрасываем биты AFRH12
    GPIOA->AFR[1] |= (0x6UL << GPIO_AFRH_AFRH4_Pos); // Устанавливаем AF6

    // PB5 (MOSI) - AF5
    GPIOB->MODER   &= ~GPIO_MODER_MODER5;
    GPIOB->MODER   |=  GPIO_MODER_MODER5_1;
    GPIOB->AFR[0]  |=  (5U << (5*4));
    
    // Настроить PB7 как Output (Push-Pull, 50 MHz)
    GPIOB->MODER   &= ~GPIO_MODER_MODER7;      // Очистить биты
    GPIOB->MODER   |=  GPIO_MODER_MODER7_0;    // Output mode
    GPIOB->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR7; // High speed
    GPIOB->OTYPER  &= ~GPIO_OTYPER_OT_7;        // Push-pull
    GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPDR7;      // No pull-up/pull-down    
    GPIOB->ODR |= GPIO_ODR_7;
    
//    // Настроить PA12 как Output (Push-Pull, 50 MHz)
//    GPIOA->MODER   &= ~GPIO_MODER_MODER12;      // Очистить биты
//    GPIOA->MODER   |=  GPIO_MODER_MODER12_0;    // Output mode
//    GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR12; // High speed
//    GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_12;        // Push-pull
//    GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPDR12;      // No pull-up/pull-down
    
    // Настроить PB1 как Analog Input
    GPIOB->MODER   |=  GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1;  // Analog mode
    GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPDR1;  // No pull-up/pull-down
    
    // Настроить PF7 как Output (Push-Pull, 50 MHz)
    GPIOF->MODER   &= ~GPIO_MODER_MODER7;      // Очистить биты
    GPIOF->MODER   |=  GPIO_MODER_MODER7_0;    // Output mode
    GPIOF->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR7; // High speed
    GPIOF->OTYPER  &= ~GPIO_OTYPER_OT_7;        // Push-pull
    GPIOF->PUPDR   &= ~GPIO_PUPDR_PUPDR7;      // No pull-up/pull-down    
    GPIOF->ODR |= GPIO_ODR_7;
}
//-----------------------------------------------------------------------------
void ADC1_Init(void)
{
    // Настройка тактирования ADC: PCLK2 (64 МГц) / 8 = 8 МГц
    RCC->CFGR &= ~RCC_CFGR_ADCPRE_DIV8;  // Очистить биты предделителя
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV8;  // Установить деление на 8
    
    // Тактирование ADC1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
    // Настройка канала ADC_IN9 (PB1)
    ADC1->SQR1 = 0; // L[3:0] = 0 т.е 1 преобразование  
    ADC1->SQR3 = 0;
    ADC1->SQR3 |= (9U << 0);  // ADC_IN9    
    
    // Время сэмплирования
    ADC1->SMPR2 = 0;     
//    ADC1->SMPR2 |= (0U << ADC_SMPR2_SMP0_Pos); // 1.5 cycles    
    ADC1->SMPR2 |= (3U << ADC_SMPR2_SMP0_Pos); // 28.5 cycles
//    ADC1->SMPR2 |= (7U << ADC_SMPR2_SMP0_Pos); // 239.5 cycles
    
    ADC1->CR2 |= (7U << ADC_CR2_EXTSEL_Pos); // SWSTART
    
    ADC1->CR2 |= ADC_CR2_ADON;
    
    // Калибровка АЦП
    // Рекомендуется выполнять калибровку после каждого включения питания.
    // Перед началом калибровки АЦП должен находиться в состоянии 
    // включенного питания (бит ADON = 1) не менее двух тактовых циклов АЦП.
    for(volatile int i = 0; i < 10; i++);    
    
    ADC1->CR2 |= ADC_CR2_CAL;                // Запустить калибровку
    while(ADC1->CR2 & ADC_CR2_CAL);          // Дождаться окончания калибровки      
}
//-----------------------------------------------------------------------------
void TIM6_Init(void)
{
    // Тактирование TIM6 (под APB1, частота 64 МГц)
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Настройка таймера
    TIM6->PSC = 0;            // Предделитель: 0 → деление на 1
    TIM6->ARR = 1279;         // Период: 1280 отсчётов → 64'000'000 / 1280 = 50'000 Гц → 20 мкс

    // Включить прерывание по Update Event
    TIM6->DIER |= TIM_DIER_UIE;

    // Настроить NVIC
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    NVIC_SetPriority(TIM6_DAC_IRQn, 1);

    
}
//-----------------------------------------------------------------------------
void TIM6_DAC_IRQHandler(void)
{
    if (TIM6->SR & TIM_SR_UIF)
    {
        TIM6->SR &= ~TIM_SR_UIF;  // Сбросить флаг прерывания
        
        // Запуск ADC
        ADC1->CR2 |= ADC_CR2_SWSTART | ADC_CR2_EXTTRIG;

        DMA1_Channel3->CCR &= ~DMA_CCR_EN;
        DMA1_Channel3->CNDTR = 4;
        DMA1_Channel3->CCR |= DMA_CCR_EN; // Запустить DMA
        
////        GPIOF->ODR ^= GPIO_ODR_7;  // Переключить PF7       
//        GPIOF->ODR |= GPIO_ODR_7;
        
    }
}
//-----------------------------------------------------------------------------
void DMA1_Init()
{
    SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);
    tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);
    (void)tmpreg;
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
 }
//-----------------------------------------------------------------------------
void SPI1_Init(void)
{
    // Тактирование SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    
    DMA1_Channel3->CCR = 0;
    SET_BIT(DMA1_Channel3->CCR, DMA_CCR_MINC | DMA_CCR_DIR);
    MODIFY_REG(DMA1_Channel3->CCR, DMA_CCR_PSIZE_1, DMA_CCR_PSIZE_0);
    MODIFY_REG(DMA1_Channel3->CCR, DMA_CCR_MSIZE_1, DMA_CCR_MSIZE_0);
    CLEAR_BIT(SPI1->CR1, 0x00FF);
    SET_BIT(SPI1->CR1, SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_MSTR);
    CLEAR_BIT(SPI1->CR2, 0x00FF);
    SET_BIT(SPI1->CR2, SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_DS_3);
}
//-----------------------------------------------------------------------------
void UART_Init(void) 
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Тактирование USART1
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Тактирование порта A
    
    GPIOA->MODER &= ~(GPIO_MODER_MODER9_Msk); // Сброс битов моды для PA9
    GPIOA->MODER |= (2UL << GPIO_MODER_MODER9_Pos); // Установка 10b (Alternate Function)

    // Тип выхода - Push-Pull (по умолчанию)
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_9);

    // Скорость вывода - High Speed
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9_Msk);

    // Подтяжка - нет (No pull-up, No pull-down)
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9_Msk);

    // Выбрать AF7 для PA9
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH1_Msk); // Сброс битов альтернативной функции для PA9
    GPIOA->AFR[1] |= (7UL << GPIO_AFRH_AFRH1_Pos); // Установка AF7
	
	USART1->CR1 = 0x00000000U;
    USART1->CR2 = 0x00000000U;
    USART1->CR3 = 0x00000000U;
	
	// Установить делитель скорости на 115200 @ 64MHz
    USART1->BRR = 0x22CU;

    // Включить USART
    USART1->CR1 |= USART_CR1_UE;

    // Включить передатчик
    USART1->CR1 |= USART_CR1_TE;
}
//-----------------------------------------------------------------------------
/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
    while(nCount--)
    {
    }
}
