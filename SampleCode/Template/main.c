/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#define LED_R					(PH0)
#define LED_Y					(PH1)
#define LED_G					(PH2)

#define BUF_LEN					(1024)

#define FIFO_THRESHOLD 			(4)
#define RX_BUFFER_SIZE 			(256)
#define RX_TIMEOUT_CNT 			(60) //40~255

#define UART_RX_IDEL(uart) (((uart)->FIFOSTS & UART_FIFOSTS_RXIDLE_Msk )>> UART_FIFOSTS_RXIDLE_Pos)

typedef struct {
	uint8_t RX_Buffer[RX_BUFFER_SIZE];
	uint16_t Length;
	uint8_t RDA_Trigger_Cnt;
	uint8_t RXTO_Trigger_Cnt;
	
//	uint8_t end;
}UART_BUF_t;

UART_BUF_t uart0Dev;

/*----------------------------------------------------*/
typedef enum{
	flag_DEFAULT = 0 ,
		
	flag_UART0_Received_Data ,	

	flag_reverse ,
	
	flag_END	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

/*----------------------------------------------------*/
#define dPDMA_TEST_LENGTH 						(1)
#define PDMAchannel_TIMER_EXT 					(1)
uint32_t g_au32CAPValue[dPDMA_TEST_LENGTH] = {0};

#define PDMATIMEOUT(ms)						(ms*(CLK_GetCPUFreq()>>15)/1000)

#define TIMER_PSC	 							(0xFF)		// 0 ~ 0xFF

//#define USE_INT
#define USE_FLOAT

/*----------------------------------------------------*/
#define LED_TOGGLE								(LED_G = ~LED_G)
#define TIMER_1S									(10000)	//(4302)	//(6)
uint32_t counter_target = 66;	//TIMER_1S;

/*----------------------------------------------------*/

void PDMA_TimerCapture_Start(void)
{
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 8 bits(one byte) */
    PDMA_SetTransferCnt(PDMA, PDMAchannel_TIMER_EXT, PDMA_WIDTH_32, dPDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, Source increment size is 8 bits(one byte), destination address is PA->DOUT (no increment)*/
    PDMA_SetTransferAddr(PDMA, PDMAchannel_TIMER_EXT, (uint32_t) &(TIMER0->CAP), PDMA_SAR_FIX, (uint32_t)g_au32CAPValue, PDMA_DAR_INC);
    /* Request source is timer 1 */
    PDMA_SetTransferMode(PDMA, PDMAchannel_TIMER_EXT, PDMA_TMR0, FALSE,(uint32_t) NULL);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA, PDMAchannel_TIMER_EXT, PDMA_REQ_SINGLE,(uint32_t) PDMA_BURST_128);
}


/*
	due to TIMER_SET_CMP_VALUE 2nd parameter limit : 0xFFF FFF
	=> able to detect lowest freq = 192000000 / 0xFFF FFF = 11
	=> able to detect lowest freq = 192000000(PSC + 1) / 0xFFF FFF = 0.044... , PSC = 0 ~ 0xFF
*/

void PDMA_TimerCapture_Init(void)	//PB15 : TM0_EXT 
{
    /* Enable Timer0 external capture function */
    TIMER_Open(TIMER0, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER0, TIMER_PSC);
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_FALLING);
    TIMER_SetTriggerSource(TIMER0, TIMER_TRGSRC_CAPTURE_EVENT);
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_PDMA);

	SYS_ResetModule(PDMA_RST);

    /* Open Channel 1 */
    PDMA_Open(PDMA, 1 << PDMAchannel_TIMER_EXT);
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 8 bits(one byte) */
    PDMA_SetTransferCnt(PDMA, PDMAchannel_TIMER_EXT, PDMA_WIDTH_32, dPDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, Source increment size is 8 bits(one byte), destination address is PA->DOUT (no increment)*/
    PDMA_SetTransferAddr(PDMA, PDMAchannel_TIMER_EXT, (uint32_t) &(TIMER0->CAP), PDMA_SAR_FIX, (uint32_t)g_au32CAPValue, PDMA_DAR_INC);
    /* Request source is timer 1 */
    PDMA_SetTransferMode(PDMA, PDMAchannel_TIMER_EXT, PDMA_TMR0, FALSE,(uint32_t) NULL);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA, PDMAchannel_TIMER_EXT, PDMA_REQ_BURST,(uint32_t) PDMA_BURST_128);

	PDMA_SetTimeOut(PDMA,PDMAchannel_TIMER_EXT, TRUE, PDMATIMEOUT(1));
	PDMA_EnableTimeout(PDMA,(1 << PDMAchannel_TIMER_EXT));

    /* Enable interrupt */
    PDMA_EnableInt(PDMA, PDMAchannel_TIMER_EXT, PDMA_INT_TRANS_DONE);
    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);

    TIMER_Start(TIMER0);

	PDMA_TimerCapture_Start();
	
}


void PDMA_TimerCapture_Process(void)
{
//    uint32_t u32InitCount = 0;	
//	uint16_t i = 0;

    volatile uint32_t u32CAPDiff = 0;
    volatile uint32_t clock = 0;
    volatile uint32_t inputclock = 0;		

	static uint32_t u32Temp = 0;
	
	#if defined (USE_INT)
    uint32_t Freq = 0;
	#elif defined (USE_FLOAT)
    double Freq = 0;
	#endif

	#if 0	//debug
	printf("\r\n");
	for ( i = 0 ; i < dPDMA_TEST_LENGTH; i++)
	{
		printf("%8d , ",g_au32CAPValue[i]);
	}
	printf("\r\n");
	#endif

	
    u32CAPDiff = g_au32CAPValue[0] - u32Temp;

	//Timer input clock or event source is divided by (PSC+1) before it is fed to the timer up counter
	inputclock = TIMER_GetModuleClock(TIMER0);
	clock = (inputclock / (TIMER_PSC + 1));
	LED_Y = ~LED_Y;


	if (g_au32CAPValue[0] > u32Temp)
	{
		#if defined (USE_INT)
		Freq = (clock/u32CAPDiff);		
		printf("clock : %6d (counter:%5d) , Diff: %9d. Freq: %6d Hz\r\n",clock,counter_target,u32CAPDiff, Freq);
		
		#elif defined (USE_FLOAT) 
		Freq = ((double )clock/(double )u32CAPDiff);		
		printf("clock : %6d (counter:%5d) , Diff: %9d. Freq: %.6f Hz\r\n",clock,counter_target,u32CAPDiff, Freq);
		
		#endif
	}

	u32Temp = g_au32CAPValue[0];


	PDMA_TimerCapture_Start();//PDMA_TimerCapture_Init();
}

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & PDMA_INTSTS_REQTOF1_Msk)	//(1 << (PDMA_INTSTS_REQTOF1_Pos + UART_RX_PDMA_CH) /* Request Time-out */
    {
		PDMA_SetTimeOut(PDMA,PDMAchannel_TIMER_EXT, 0, 0);
	
        /* Clear PDMA timeout interrupt flag */
        PDMA_CLR_TMOUT_FLAG(PDMA, PDMAchannel_TIMER_EXT);

		PDMA_SetTimeOut(PDMA,PDMAchannel_TIMER_EXT, TRUE, PDMATIMEOUT(1));
        /* Disable and enable timeout function to restart the timeout counter */
		PDMA_DisableTimeout(PDMA,(1 << PDMAchannel_TIMER_EXT) );
		PDMA_EnableTimeout(PDMA,(1 << PDMAchannel_TIMER_EXT) );
		
        /* Set transfer count and trigger again */
        PDMA_SetTransferCnt(PDMA, PDMAchannel_TIMER_EXT, PDMA_WIDTH_32, dPDMA_TEST_LENGTH);
        /* Get the latest status for SPI PDMA again */
        status = PDMA_GET_INT_STATUS(PDMA);
    }


    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        /* Check if channel 1 has abort error */
        if (PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF1_Msk)
        {
			//abort
			printf("abort\r\n");
        }

        /* Clear abort flag of channel 1 */
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF1_Msk);
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        /* Check transmission of channel 1 has been transfer done */
        if (PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF1_Msk)
        {
			//done
			TIMER_ClearCaptureIntFlag(TIMER0);
//			TIMER_Stop(TIMER0);
			
			PDMA_TimerCapture_Process();
			LED_R = ~LED_R;
        }

        /* Clear transfer done flag of channel 1 */
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void TMR0_IRQHandler(void)
{
    if(TIMER_GetCaptureIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 capture trigger interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER0);
    }
}
void TMR3_IRQHandler(void)
{
	static uint32_t CNT = 0;

    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);

		if (CNT++ >= (counter_target >> 1))
		{		
			CNT = 0;
			LED_G ^= 1;
		}					
    }
}

void TIMER3_Init(void)		// use 100 us timer
{
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 10000);
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);	
    TIMER_Start(TIMER3);
}

void UART0_Process(uint8_t rx)
{
	switch(rx)
	{
		case '1':				
			counter_target = 40000;
			PDMA_TimerCapture_Start();
		break;

		case '2':
			counter_target = 20000;
			PDMA_TimerCapture_Start();
		break;

		case '3':
			counter_target = 10000;
			PDMA_TimerCapture_Start();
		break;	
		
		case '4':
			counter_target = 1000;
			PDMA_TimerCapture_Start();
		break;	

		case '5':
			counter_target = 40;
			PDMA_TimerCapture_Start();
		break;	

		case '6':
			counter_target = 20;
			PDMA_TimerCapture_Start();
		break;	

		case '7':
			counter_target = 10;
			PDMA_TimerCapture_Start();
		break;	

		case 'Z':
		case 'z':
			NVIC_SystemReset();
		break;
		
	}
	
}

void UART0_IRQHandler(void)
{
	uint8_t i;
	static uint16_t u16UART_RX_Buffer_Index = 0;

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk))    
    {
        /* UART receive data available flag */
        
        /* Record RDA interrupt trigger times */
        uart0Dev.RDA_Trigger_Cnt++;
        
        /* Move the data from Rx FIFO to sw buffer (RAM). */
        /* Every time leave 1 byte data in FIFO for Rx timeout */
        for(i = 0 ; i < (FIFO_THRESHOLD - 1) ; i++)
        {
            uart0Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;

            if (u16UART_RX_Buffer_Index >= RX_BUFFER_SIZE) 
                u16UART_RX_Buffer_Index = 0;
        }
    }
    else if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RXTOINT_Msk)) 
    {
        /* When Rx timeout flag is set to 1, it means there is no data needs to be transmitted. */

        /* Record Timeout times */
        uart0Dev.RXTO_Trigger_Cnt++;

        /* Move the last data from Rx FIFO to sw buffer. */
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            uart0Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;
        }

        /* Clear UART RX parameter */
        UART_DISABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
        u16UART_RX_Buffer_Index = 0;

//		set_flag(flag_UART0_Received_Data , ENABLE);

		#if 1
		printf("UART RX : 0x%2X \r\n" , uart0Dev.RX_Buffer[0]);
		
		UART0_Process(uart0Dev.RX_Buffer[0]);
		#else

        printf("\nUART0 Rx Received Data : %s\n",uart0Dev.RX_Buffer);
        printf("UART0 Rx RDA (Fifofull) interrupt times : %d\n",uart0Dev.RDA_Trigger_Cnt);
        printf("UART0 Rx RXTO (Timeout) interrupt times : %d\n",uart0Dev.RXTO_Trigger_Cnt);
		#endif

        /* Reset UART interrupt parameter */
        UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
		memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));

    }
	
}

/*
	EC_M451_UART_Timerout_V1.00.zip
	https://www.nuvoton.com/hq/resource-download.jsp?tp_GUID=EC0120160728090754
*/

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, RX_TIMEOUT_CNT);

	/* Set UART FIFO RX interrupt trigger level to 4-bytes*/
    UART0->FIFO = ((UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES);

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);
	
	memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));

	UART_WAIT_TX_EMPTY(UART0);
	
//	set_flag(flag_UART0_Received_Data , DISABLE);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);

    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

	/*----------------------------------------------------*/
    /* Set multi-function pin for Timer0 external capture pin */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) \
                    | SYS_GPB_MFPH_PB15MFP_TM0_EXT;
	
    /* Lock protected registers */
    SYS_LockReg();
}
/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();
	
    UART0_Init();
	
	LED_Init();
	
	TIMER3_Init();

	PDMA_TimerCapture_Init();	//PB15 : TM0_EXT
	
    /* Got no where to go, just loop forever */
    while(1)
    {
		
    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
