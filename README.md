# M480BSP_TIMER_EXT_PDMA
 M480BSP_TIMER_EXT_PDMA

update @ 2020/07/10

1. use TM0_EXT (PB15) with PDMA , to capture external signal freq.

	- test signal : LED (PG2)

2. increase TIMER0 PSC value to 0xFF , to increase detect freq range under 0 (check PDMA_TimerCapture_Process) 

	- ex : if TIMER0 PSC set 0 , lowest freq. is TIMER0 CLOCK 96000000/0xFFF FFF = 5

	- ex : if TIMER0 PSC set 0xFF , lowest freq. is TIMER0 CLOCK (96000000/(0xFF + 1))/0xFFF FFF = 0.022351

3. Connect LED (PG2) to TM0_EXT (PB15) , to simulate external signal
	
	- with UART terminal , press 1 to 7 , to set different signal freq
	
4. below is waveform with log capture screen (from digit 1 to 7)

0.25Hz

![image](https://github.com/released/M480BSP_TIMER_EXT_PDMA/blob/master/SimulateFreq1.jpg)

0.5Hz

![image](https://github.com/released/M480BSP_TIMER_EXT_PDMA/blob/master/SimulateFreq2.jpg)

1Hz

![image](https://github.com/released/M480BSP_TIMER_EXT_PDMA/blob/master/SimulateFreq3.jpg)

9.79Hz

![image](https://github.com/released/M480BSP_TIMER_EXT_PDMA/blob/master/SimulateFreq4.jpg)

161Hz

![image](https://github.com/released/M480BSP_TIMER_EXT_PDMA/blob/master/SimulateFreq5.jpg)

