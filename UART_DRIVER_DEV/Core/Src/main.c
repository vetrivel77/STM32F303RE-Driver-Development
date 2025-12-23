/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define SIZE   50
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum Mode{FullDuplex = 0,HalfDuplex};
enum Parity{odd = 1,Even};enum Stop{Bit1=1,Bit2};enum data_M{Interrupt = 1,DMA};

typedef struct{
	char uRx_buff[SIZE];
	char *uTx_buff;
	volatile uint16_t urx_iter , utx_iter;
}uart_isr_;

char u1Rx_dma_buff[SIZE];
char u3Rx_dma_buff[SIZE];

char u1[] = "HELLO I AM UART1 IN\r\n";
char u2[] = "HI I AM UART2 IN\r\n";
char u3[] = "HELLO I AM UART3 IN\r\n";
char u4[] = "HI I AM UART4 IN\r\n";
char u5[] = "HELLO I AM UART5 IN\r\n";

char dma_u1[] = "HELLO I AM UART1 DMA\r\n";
char dma_u2[] = "HI I AM UART2 DMA\r\n";
char dma_u3[] = "HELLO I AM UART3 DMA\r\n";
char dma_u4[] = "HI I AM UART4 DMA\r\n";


char u1Rx_Buff[SIZE] = {0x00};
char *u1Tx_Buff = NULL;
volatile uint16_t u1rx_iter = 0, u1tx_iter  = 0 ,u1_tx_len = 0;

char u3Rx_Buff[SIZE] = {0x00};
char *u3Tx_Buff = NULL;
volatile uint16_t u3rx_iter = 0, u3tx_iter  = 0 ,u3_tx_len = 0;

char u4Rx_Buff[SIZE] = {0x00};
char *u4Tx_Buff = NULL;
volatile uint16_t u4rx_iter = 0, u4tx_iter  = 0 ,u4_tx_len = 0;

char u5Rx_Buff[SIZE] = {0x00};
char *u5Tx_Buff = NULL;
volatile uint16_t u5rx_iter = 0, u5tx_iter  = 0 ,u5_tx_len = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void EVL_UART_GPIO_Init(GPIO_TypeDef *gpio,USART_TypeDef *uart,uint16_t Mode,uint16_t TX_PIN,uint16_t RX_PIN);
void EVL_UART_Init(USART_TypeDef *uart,uint8_t FullDuplex,uint32_t BaudRate,uint8_t DataBits,uint16_t ParityBit,uint16_t StopBit);
void EVL_UART_Transmit(USART_TypeDef *uart,char *Data);
void EVL_UART_Receive(USART_TypeDef *uart);
void EVL_UART_Transmit_IT(USART_TypeDef *uart,char *Data,uint16_t len);
void EVL_UART_Receive_IT(USART_TypeDef *uart);
void EVL_UART_Transmit_DMA(USART_TypeDef *uart,char Data[]);
void EVL_UART_Receive_DMA(USART_TypeDef *uart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RX_BUF_SIZE 1024

char DMArxBuff[RX_BUF_SIZE];
volatile uint16_t rxWr = 0 , txIdx =0;


int str_len(char *str)
{
	uint8_t i = 0 , len = 0;

	for(i = 0 ; str[i] != '\0' ;i++)
	{
		len++;
	}
	return len;
}

void EVL_UART_Receive_DMA(USART_TypeDef *uart){

	IRQn_Type irq;

	switch ((uint32_t)uart)
	{
		case ((uint32_t)USART3):{
			irq = DMA1_Channel3_IRQn;
			DMA1_Channel3->CCR &= ~DMA_CCR_EN; //disable
			DMA1_Channel3->CPAR = (uint32_t)&uart->RDR; //base address
			DMA1_Channel3->CMAR = (uint32_t)u3Rx_dma_buff;
			DMA1_Channel3->CNDTR = SIZE;
			DMA1_Channel3->CCR =
			      DMA_CCR_MINC      |   // Memory increment
			      DMA_CCR_TCIE      |   // Transfer complete interrupt
			      DMA_CCR_PL_1      |   // High priority
			      0;                    // Peripheral → Memory (DIR = 0)
			uart->CR3 |= USART_CR3_DMAR; //dma enable transmit the data
			DMA1_Channel3->CCR |= DMA_CCR_EN;
			break;}
		case ((uint32_t)USART1):{
			irq = DMA1_Channel5_IRQn;
			DMA1_Channel5->CCR &= ~DMA_CCR_EN; //disable
			DMA1_Channel5->CPAR = (uint32_t)&uart->RDR; //base address
			DMA1_Channel5->CMAR = (uint32_t)u1Rx_dma_buff;
			DMA1_Channel5->CNDTR = SIZE;
			DMA1_Channel5->CCR =
					DMA_CCR_MINC      |   // Memory increment
					DMA_CCR_TCIE      |   // Transfer complete interrupt
					DMA_CCR_PL_1      |   // High priority
					0;                    // Peripheral → Memory (DIR = 0)
			uart->CR3 |= USART_CR3_DMAR; //dma enable transmit the data
			DMA1_Channel5->CCR |= DMA_CCR_EN;
			break;}
		default: break;
	}

	NVIC_SetPriority(irq,1);
	NVIC_EnableIRQ(irq);
}

void EVL_UART_Transmit_DMA(USART_TypeDef *uart,char Data[]){

	//assign the tx into buffer
	IRQn_Type irq;

	switch((uint32_t)uart){
		case (uint32_t)USART1:{
			irq = DMA1_Channel4_IRQn;
			DMA1_Channel4->CCR &= ~DMA_CCR_EN; //ccr disable
			DMA1_Channel4->CPAR = (uint32_t)&uart->TDR; //uart base address storing
			DMA1_Channel4->CMAR = (uint32_t)Data; //buffer storing into system memory
			DMA1_Channel4->CNDTR = str_len(Data); //number of bytes to send -> decrements the size every tx
			DMA1_Channel4->CCR =
					DMA_CCR_MINC      |   // Memory increment
				    DMA_CCR_DIR       |   // Memory → Peripheral
				    DMA_CCR_TCIE      |   // Transfer complete interrupt
				    DMA_CCR_PL_1;         // High priority
			uart->CR3 |= USART_CR3_DMAT; //dma enable transmit the data
			DMA1_Channel4->CCR |= DMA_CCR_EN; //atlast dma ccr enable
			break;}
		case (uint32_t)USART3:{
			irq = DMA1_Channel2_IRQn;
			DMA1_Channel2->CCR &= ~DMA_CCR_EN; //ccr disable
			DMA1_Channel2->CPAR = (uint32_t)&uart->TDR; //uart base address storing
			DMA1_Channel2->CMAR = (uint32_t)Data; //buffer store into memory
			DMA1_Channel2->CNDTR = str_len(Data); //number of bytes to send
			DMA1_Channel2->CCR =
					DMA_CCR_MINC  |   // Memory increment
					DMA_CCR_DIR   |   // Memory → Peripheral
					DMA_CCR_TCIE  |   // Transfer complete interrupt
					DMA_CCR_PL_1;     // High priority
		    uart->CR3 |= USART_CR3_DMAT; //dma enable transmit the data
		    DMA1_Channel2->CCR |= DMA_CCR_EN; //atlast dma ccr enable
			break;}
		default: break;
	}

	NVIC_SetPriority(irq,1);
	NVIC_EnableIRQ(irq);

}

void DMA1_Channel2_IRQHandler(void)
{
  if(DMA1->ISR & DMA_ISR_TCIF2)
  {
	  DMA1->IFCR |= DMA_IFCR_CTCIF2;
  }
}

void DMA1_Channel3_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF3)
    {
        DMA1->IFCR |= DMA_IFCR_CTCIF3;   // Clear flag
    }
}

void DMA1_Channel4_IRQHandler(void)
{
  if(DMA1->ISR & DMA_ISR_TCIF4) //TC COMPLETE AND 4	 DENOTING THE CHANNEL
  {
	  DMA1->IFCR |= DMA_IFCR_CTCIF4; //clear flag CTCIF4: Transfer complete flag clear for channel 4
  }
}

void DMA1_Channel5_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF5)
    {
        DMA1->IFCR |= DMA_IFCR_CTCIF5;   // Clear flag
    }
}

void USART1_IRQHandler(void)
{
	/* TXE interrupt */
	if ((USART1->ISR & USART_ISR_TXE) && (USART1->CR1 & USART_CR1_TXEIE)) //TXE = 1,TXEIE = 1
	{
		if (u1tx_iter < u1_tx_len)
		{
			USART1->TDR = u1Tx_Buff[u1tx_iter++];   // Load next byte
		}
		else
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;  // Stop the interrupt tx mode
			USART1->CR1 |= USART_CR1_TCIE;   // Enable TC interrupt
		}

	}

	/* TC interrupt control */
	if ((USART1->ISR & USART_ISR_TC) && (USART1->CR1 & USART_CR1_TCIE)) //TC-1,
	{
		USART1->ICR |= USART_ICR_TCCF;       // Clear TC flag

		USART1->CR1 &= ~USART_CR1_TCIE;      // Disable TC interrupt
	        // Transmission fully completed
	}

	//* RXIE interrupt */
    if (USART1->ISR & USART_ISR_RXNE )
    {
    	if(u1rx_iter < SIZE){
			u1Rx_Buff[u1rx_iter++] = USART1->RDR;

			if((u1rx_iter >= 2) && u1Rx_Buff[u1rx_iter - 2] == 0x0D && u1Rx_Buff[u1rx_iter - 1] == 0x0A)
			{
				//u1rx_iter = 0;
				//USART1->CR1 &= ~USART_CR1_RXNEIE;  // Stop the interrupt tx mode
			}
    	}
    }

}

void USART3_IRQHandler(void)
{
    /* TXE interrupt */
    if ((USART3->ISR & USART_ISR_TXE) && (USART3->CR1 & USART_CR1_TXEIE))
    {
        if (u3tx_iter < u3_tx_len)
        {
            USART3->TDR = u3Tx_Buff[u3tx_iter++];   // Load next byte
        }
        else
        {
            USART3->CR1 &= ~USART_CR1_TXEIE;  // Disable TXE interrupt
            USART3->CR1 |= USART_CR1_TCIE;   // Enable TC interrupt
        }
    }

    /* TC interrupt */
    if ((USART3->ISR & USART_ISR_TC) && (USART3->CR1 & USART_CR1_TCIE))
    {
        USART3->ICR |= USART_ICR_TCCF;       // Clear TC flag
        USART3->CR1 &= ~USART_CR1_TCIE;      // Disable TC interrupt
        // Transmission fully completed
    }

    //* RXIE interrupt */
    //EVL_UART_Transmit(USART2, "outside\r\n");
    if (USART3->ISR & USART_ISR_RXNE)
    {
    	//EVL_UART_Transmit(USART2, "inside\r\n");
    	if(u3rx_iter < SIZE){
			u3Rx_Buff[u3rx_iter++] = USART3->RDR;

			if((u3rx_iter >= 2)&& u3Rx_Buff[u3rx_iter - 2] == 0x0D && u3Rx_Buff[u3rx_iter - 1] == 0x0A)
			{
				u3rx_iter = 0;
			}
    	}
    }

}

void UART4_IRQHandler(void)
{
	/* TXE interrupt */
	if ((UART4->ISR & USART_ISR_TXE) && (UART4->CR1 & USART_CR1_TXEIE))
	    {
	        if (u4tx_iter < u4_tx_len)
	        {
	        	UART4->TDR = u4Tx_Buff[u4tx_iter++];   // Load next byte
	        }
	        else
	        {
	        	UART4->CR1 &= ~USART_CR1_TXEIE;  // Disable TXE interrupt
	        	UART4->CR1 |= USART_CR1_TCIE;   // Enable TC interrupt
	        }
	    }

	    /* TC interrupt */
	    if ((UART4->ISR & USART_ISR_TC) && (UART4->CR1 & USART_CR1_TCIE))
	    {
	    	UART4->ICR |= USART_ICR_TCCF;       // Clear TC flag
	    	UART4->CR1 &= ~USART_CR1_TCIE;      // Disable TC interrupt
	        // Transmission fully completed
	    }

	    //* RXIE interrupt */
	    //EVL_UART_Transmit(USART2, "outside\r\n");
	    if (UART4->ISR & USART_ISR_RXNE)
	    {
	    	//EVL_UART_Transmit(USART2, "inside\r\n");
	    	if(u4rx_iter < SIZE){
				u4Rx_Buff[u4rx_iter++] = UART4->RDR;

				if((u4rx_iter >= 2)&& u4Rx_Buff[u4rx_iter - 2] == 0x0D && u4Rx_Buff[u4rx_iter - 1] == 0x0A)
				{
					u4rx_iter = 0;
				}
	    	}
	    }

}

void UART5_IRQHandler(void)
{
		/* TXE interrupt */
		if ((UART5->ISR & USART_ISR_TXE) && (UART5->CR1 & USART_CR1_TXEIE))
		{
		        if (u5tx_iter < u5_tx_len)
		        {
		        	UART5->TDR = u5Tx_Buff[u5tx_iter++];   // Load next byte
		        }
		        else
		        {
		        	UART5->CR1 &= ~USART_CR1_TXEIE;  // Disable TXE interrupt
		        	UART5->CR1 |= USART_CR1_TCIE;   // Enable TC interrupt
		        }
		    }

		    /* TC interrupt */
		    if ((UART5->ISR & USART_ISR_TC) && (UART5->CR1 & USART_CR1_TCIE))
		    {
		    	UART5->ICR |= USART_ICR_TCCF;       // Clear TC flag
		    	UART5->CR1 &= ~USART_CR1_TCIE;      // Disable TC interrupt
		        // Transmission fully completed
		    }

		    //* RXIE interrupt */
		    //EVL_UART_Transmit(USART2, "outside\r\n");
		    if (UART5->ISR & USART_ISR_RXNE)
		    {
		    	//EVL_UART_Transmit(USART2, "inside\r\n");
		    	if(u5rx_iter < SIZE){
					u5Rx_Buff[u5rx_iter++] = UART5->RDR;

					if((u5rx_iter >= 2)&& u5Rx_Buff[u5rx_iter - 2] == 0x0D && u5Rx_Buff[u5rx_iter - 1] == 0x0A)
					{
						u5rx_iter = 0;
					}
		    	}
		    }


}
void EVL_UART_Receive_IT(USART_TypeDef *uart){

	//uart interrupt
	IRQn_Type irq;

	switch((uint32_t)uart)
	{
		case (uint32_t)USART1:irq = USART1_IRQn;break;
		case (uint32_t)USART3:irq = USART3_IRQn;break;
		case (uint32_t)UART4:irq = UART4_IRQn;break;
		case (uint32_t)UART5:irq = UART5_IRQn;break;
		default:return;
	}
	NVIC_SetPriority(irq,1);
	NVIC_EnableIRQ(irq);
	uart->CR1 |= USART_CR1_RXNEIE;
}
void EVL_UART_Transmit_IT(USART_TypeDef *uart,char *Data,uint16_t len){

		IRQn_Type irq;

		switch((uint32_t)uart)
		{
			case (uint32_t)USART1:{irq = USART1_IRQn;u1Tx_Buff = Data;u1_tx_len = len;u1tx_iter = 0;break;}
			case (uint32_t)USART3:{irq = USART3_IRQn;u3Tx_Buff = Data;u3_tx_len = len;u3tx_iter = 0;break;}
			case (uint32_t)UART4:{irq = UART4_IRQn;u4Tx_Buff = Data;u4_tx_len = len;u4tx_iter = 0;break;}
			case (uint32_t)UART5:{irq = UART5_IRQn;u5Tx_Buff = Data;u5_tx_len = len;u5tx_iter = 0;break;}
			default:return;
		}
		NVIC_EnableIRQ(irq);
		uart->CR1 |= USART_CR1_TXEIE;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

	SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();


  /* USER CODE BEGIN 2 */

    //char Data[] = "HELLO I AM UART3\r\n";
    EVL_UART_GPIO_Init(GPIOA,USART2,Interrupt,2,3); //PA2,PA3
    EVL_UART_Init(USART2,FullDuplex,9600,8,0,Bit1);
    EVL_UART_Transmit(USART2,"UART2 Start...!\r\n");

    EVL_UART_GPIO_Init(GPIOC,USART1,DMA,4,5); //PC4,PC5
    EVL_UART_Init(USART1,FullDuplex,115200,8,0,Bit1);

    EVL_UART_GPIO_Init(GPIOB,USART3,DMA,10,11); //PB10,PB11
    EVL_UART_Init(USART3,FullDuplex,115200,8,0,Bit1);

    //DMA
    EVL_UART_Receive_DMA(USART3);
    EVL_UART_Transmit_DMA(USART1,dma_u1);
    EVL_UART_Receive_DMA(USART1);
    EVL_UART_Transmit_DMA(USART3,dma_u3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //EVL_UART_Transmit(USART2,Data);
	 // EVL_UART_Transmit(USART3,Data);
	 // HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EVL_UART_GPIO_Init(GPIO_TypeDef *gpio,USART_TypeDef *uart,uint16_t Mode,uint16_t tx_pin,uint16_t rx_pin)
{
	//DMA CLOCK ENABLE
	if(Mode == 0x02){
		switch((uint32_t)uart)
		{
			case (uint32_t)USART1:
			case (uint32_t)USART2:
			case (uint32_t)USART3:RCC->AHBENR |= RCC_AHBENR_DMA1EN;break;
			case (uint32_t)UART4:RCC->AHBENR |= RCC_AHBENR_DMA2EN; break;
			default:break;
		}
	}
	//PORT CLOCK ENABLE
	if(gpio == GPIOA){
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	}else if(gpio == GPIOB){
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	}else if(gpio == GPIOC){
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	}else if(gpio == GPIOD){
		RCC->AHBENR |= RCC_AHBENR_GPIODEN;
	}else if(gpio == GPIOE) {
		RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	}else if(gpio == GPIOF){
		RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
	}

	//UART CLOCK ENABLE
	if(uart == USART1){
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	}else if(uart == USART2){
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	}else if(uart == USART3){
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	}else if(uart == UART4){
		RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
	}else if(uart == UART5){
		RCC->AHBENR |= RCC_AHBENR_GPIODEN;
		RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
	}


	//PIN CONFIGURATION
	//pin modes
	//Tx AF function
	if(((uart == UART5)) && (RCC->AHBENR & RCC_AHBENR_GPIOCEN) && (RCC->AHBENR & RCC_AHBENR_GPIODEN))
	{
		/* PC12 → UART5_TX (AF5) */
		GPIOC->MODER &= ~(3U << (12 * 2));
		GPIOC->MODER |=  (2U << (12 * 2));

		GPIOC->AFR[1] &= ~(0xF << ((12 - 8) * 4));
		GPIOC->AFR[1] |=  (5U  << ((12 - 8) * 4));

		/* PD2 → UART5_RX (AF5) */
		GPIOD->MODER &= ~(3U << (2 * 2));
		GPIOD->MODER |=  (2U << (2 * 2));

		GPIOD->AFR[0] &= ~(0xF << (2 * 4));
		GPIOD->AFR[0] |=  (5U  << (2 * 4));
	}
	else
	{
		gpio->MODER &= ~(3U << (tx_pin * 2));
		gpio->MODER |= (2U << (tx_pin * 2));

		//rx AF Function
		gpio->MODER &= ~(3U << (rx_pin * 2));
		gpio->MODER |= (2U << (rx_pin * 2));

		//speeder
		gpio->OSPEEDR &= ~((3U << (tx_pin * 2)) | (3U << (rx_pin * 2)));
		gpio->OSPEEDR |=  ((3U << (tx_pin * 2)) | (3U << (rx_pin * 2)));

		// Set AF for TX AF[0] -> 0...7 AF[1] -> 8...15
		if ((uart == USART1 ) || (uart == USART2)) //AF7
		{
			//tx pins
			gpio->AFR[0] &= ~(0xF << (tx_pin * 4)); //clear
			gpio->AFR[0] |= (7U << (tx_pin * 4)); //set

			//rx pins
			gpio->AFR[0] &= ~(0xF << (rx_pin * 4)); //clear
			gpio->AFR[0] |= (7U << (rx_pin * 4)); //set

		}else if(uart == USART3){

			gpio->AFR[1] &= ~(0xF << ((tx_pin - 8)* 4));
			gpio->AFR[1] |= (7U << ((tx_pin - 8) * 4));

			gpio->AFR[1] &= ~(0xF << ((rx_pin - 8)* 4));
			gpio->AFR[1] |= (7U << ((rx_pin - 8) * 4));

		}
		else if(uart == UART4) //AF5 pc10 , 11
		{
			gpio->AFR[1] &= ~(0xF << ((tx_pin - 8) * 4));
			gpio->AFR[1] |= (5U << ((tx_pin - 8) * 4));

			gpio->AFR[1] &= ~(0xF << ((rx_pin - 8) * 4));
			gpio->AFR[1] |= (5U << ((rx_pin - 8) * 4));
		}

		//pullup the rx pin
		gpio->PUPDR &= ~(3U << (rx_pin * 2));
		gpio->PUPDR |= (1U << (rx_pin * 2));
	}
}

void EVL_UART_Init(USART_TypeDef *uart,uint8_t FullDuplex,uint32_t BaudRate,uint8_t DataBits,uint16_t ParityBit,uint16_t StopBit)
{
	//baud rate EX 115200 calculation
	uint16_t val = (SystemCoreClock + (BaudRate/2))/BaudRate;

	//uart configuration
	uart->CR1 = 0x00; //clear all . by default 8bits data
	uart->BRR = 0x0000;

	//baud rate set
	uart->BRR = val;

	//RX and TX enable
	uart->CR1 |= USART_CR1_RE;
	uart->CR1 |= USART_CR1_TE;


#if 0
	//parity bit
	if(ParityBit == 0x01) //0dd
	{
		uart->CR1 |= USART_CR1_PCE;
		uart->CR1 |= USART_CR1_PS;
		uart->CR1 |= USART_CR1_M;

	}else if(ParityBit == 0x02)
	{
		uart->CR1 |= USART_CR1_PCE;    // Enable parity
		uart->CR1 &= ~USART_CR1_PS;   // Even parity
		uart->CR1 |= USART_CR1_M;     // 8 data bits (parity uses MSB)

	}

	//stop bits
	if(StopBit == 0x01)
	{
		uart->CR2 &= ~(3U << 12);
	}else if(StopBit == 0x02)
	{
		uart->CR2 &= ~(3U << 12);
		uart->CR2 |= (2U << 12);
	}
#endif

	uart->CR1 |= USART_CR1_UE;//uart enable
}

void EVL_UART_Transmit(USART_TypeDef *uart,char *Data){

	uint8_t i = 0;
	while(Data[i] != '\0')
	{
		while(!(uart->ISR & USART_ISR_TXE)); //wait for tc set indicates tc completed.. 1 , 0
		uart->TDR = Data[i]; //LOAD THE DATA
		i++;
	}
	while (!(uart->ISR & USART_ISR_TC));  // TC flag (Transmission Complete)
}

#if 0
void EVL_UART_Receive(USART_TypeDef *uart){

		/* Check if data is available (NON-BLOCKING) */
	    if (uart == USART1)
	    {
	    	if (i >= RX_BUF_SIZE){
	    	    	i = 0;
	    	}
	    	while(!(uart->ISR & USART_ISR_RXNE));
	        rxData[i++] = (uint8_t)(uart->RDR & 0xFF);

	        //if(rxData[i-2] == 0x0D && rxBuf[i-1] == 0x0A)
	        {
	             i = 0;
	        }
	    }
}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
