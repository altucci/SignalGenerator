#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <dos.h>
#include <math.h>
#include "ae.h"

#include "ser1.h"

#define BUF_LENGTH								4096

#define SIGNAL_1_ADDR							0x01
#define SIGNAL_2_ADDR							0x02
#define SIGNAL_3_ADDR							0x04
#define SIGNAL_4_ADDR							0x08

#define PARAM_ID_SIGNAL_1_PULSE_DURATION		100
#define PARAM_ID_SIGNAL_2_PULSE_DURATION		101
#define PARAM_ID_SIGNAL_3_PULSE_DURATION		102
#define PARAM_ID_SIGNAL_4_PULSE_DURATION		103

#define PARAM_ID_NUMBER_OF_CYCLES				104

extern COM ser1_com;

typedef struct
{
	COM *m_comPC1;

	unsigned char serPC1_in_buf[BUF_LENGTH];
	unsigned char serPC1_out_buf[BUF_LENGTH];

} COMM_VALS;

COMM_VALS CommVals;

typedef struct
{
	unsigned char DigitalOutputs;

	int TriggerMode;

	int PulseDuration[4];
	int PulseDurationHigh[4];
	int NumberOfCycles;
	
	int PulseDurationCounter[4];
	int CycleCounter[4];

	int PulseWidthsPerPeriod;

	int TimerFrequency;
	int TimerMaxCount;

	int TimerCounter;
	unsigned long TimerCounterSeconds;

} VARS;

VARS Vars;

void initialize(void);
void interrupt far int_timer_isr(void);
int CheckSerialComm(void);

void main(void)
{
	initialize();

	while (1)
	{
		while (CheckSerialComm() != -1)
			hitwd();

		if (Vars.TimerCounter >= Vars.TimerFrequency)
		{
			Vars.TimerCounter = 0;
			Vars.TimerCounterSeconds++;
			hitwd();
		}
	}
}

void initialize(void)
{
	ae_init();
	pio_init(11,1);
	pio_init(18, 0);	//	P18=CTS1 for U24 CS
	pio_init(3,0);

	// Configure & Initialize H5 Ports (82C55)
	outportb(0x0103, 0x82);
	outportb(0x0100, 0x00);

	// Initialize Comm Ports
	memset(&CommVals, 0, sizeof(COMM_VALS));

	CommVals.m_comPC1 = &ser1_com;
	s1_init(9, CommVals.serPC1_in_buf, BUF_LENGTH, CommVals.serPC1_out_buf, BUF_LENGTH, CommVals.m_comPC1);  	// 19,200 baud
	
	clean_ser1(CommVals.m_comPC1);
	
	memset(&Vars, 0, sizeof(VARS));

	Vars.PulseDuration[0] = 200;
	Vars.PulseDuration[1] = 210;
	Vars.PulseDuration[2] = 220;
	Vars.PulseDuration[3] = 240;
	Vars.PulseDurationHigh[0] = 40;
	Vars.PulseDurationHigh[1] = 42;
	Vars.PulseDurationHigh[2] = 44;
	Vars.PulseDurationHigh[3] = 48;

	Vars.NumberOfCycles = 1000;

	Vars.PulseWidthsPerPeriod = 5;

	// Initialize Timer 2 Interrupt
	Vars.TimerMaxCount = 2000;			//TimerFrequency = 10,000,000 / TimerMaxCount, so if TimerMaxCount = 5000, then TimerFrequency = 2000 ints/sec = 1 int/500 microseconds
	Vars.TimerFrequency = (int)(10000000L / (long)Vars.TimerMaxCount);

	t2_init(0xE001, Vars.TimerMaxCount, int_timer_isr);
}

void interrupt far int_timer_isr(void)
{
	if (Vars.TriggerMode)
	{
		if (Vars.CycleCounter[0] < Vars.NumberOfCycles)
		{
			if (++Vars.PulseDurationCounter[0] <= Vars.PulseDurationHigh[0])
			{
				Vars.DigitalOutputs |= SIGNAL_1_ADDR;
			}
			else
			{
				Vars.DigitalOutputs &= 0xFF ^ SIGNAL_1_ADDR;

				if (Vars.PulseDurationCounter[0] == Vars.PulseDuration[0])
				{
					Vars.PulseDurationCounter[0] = 0;
					Vars.CycleCounter[0]++;
				}
			}
		}
		
		if (Vars.CycleCounter[1] < Vars.NumberOfCycles)
		{
			if (++Vars.PulseDurationCounter[1] <= Vars.PulseDurationHigh[1])
			{
				Vars.DigitalOutputs |= SIGNAL_2_ADDR;
			}
			else
			{
				Vars.DigitalOutputs &= 0xFF ^ SIGNAL_2_ADDR;

				if (Vars.PulseDurationCounter[1] == Vars.PulseDuration[1])
				{
					Vars.PulseDurationCounter[1] = 0;
					Vars.CycleCounter[1]++;
				}
			}
		}
		
		if (Vars.CycleCounter[2] < Vars.NumberOfCycles)
		{
			if (++Vars.PulseDurationCounter[2] <= Vars.PulseDurationHigh[2])
			{
				Vars.DigitalOutputs |= SIGNAL_3_ADDR;
			}
			else
			{
				Vars.DigitalOutputs &= 0xFF ^ SIGNAL_3_ADDR;

				if (Vars.PulseDurationCounter[2] == Vars.PulseDuration[2])
				{
					Vars.PulseDurationCounter[2] = 0;
					Vars.CycleCounter[2]++;
				}
			}
		}
		
		if (Vars.CycleCounter[3] < Vars.NumberOfCycles)
		{
			if (++Vars.PulseDurationCounter[3] <= Vars.PulseDurationHigh[3])
			{
				Vars.DigitalOutputs |= SIGNAL_4_ADDR;
			}
			else
			{
				Vars.DigitalOutputs &= 0xFF ^ SIGNAL_4_ADDR;

				if (Vars.PulseDurationCounter[3] == Vars.PulseDuration[3])
				{
					Vars.PulseDurationCounter[3] = 0;
					Vars.CycleCounter[3]++;
				}
			}
		}

		if (Vars.CycleCounter[0] == Vars.NumberOfCycles && Vars.CycleCounter[1] == Vars.NumberOfCycles && Vars.CycleCounter[2] == Vars.NumberOfCycles && Vars.CycleCounter[3] == Vars.NumberOfCycles)
		{
			Vars.TriggerMode = 0;
		}

		outportb(0x0100, Vars.DigitalOutputs);
	}

	Vars.TimerCounter++;

	outport(0xff22, 0x8000);
	
}

int CheckSerialComm(void)
{
	char ch;
	int i;
	char param_buf[10];
	char value_buf[10];
	int param;
    int val;
	unsigned char check_sum;
	unsigned char pc_check_sum;

	check_sum = 0;

	if (!serhit1(CommVals.m_comPC1)) return -1;
	ch = getser1(CommVals.m_comPC1);
	if (ch != '~') return -1;
	check_sum += ch;

	while (!serhit1(CommVals.m_comPC1)) hitwd();
	ch = getser1(CommVals.m_comPC1);
	if (ch != '~') return -1;
	check_sum += ch;

	while (!serhit1(CommVals.m_comPC1)) hitwd();
	ch = getser1(CommVals.m_comPC1);
	if (ch != '2') return -1;
	check_sum += ch;

	while (!serhit1(CommVals.m_comPC1)) hitwd();
	ch = getser1(CommVals.m_comPC1);
	check_sum += ch;
	
	if (ch == '1')
	{
		while (!serhit1(CommVals.m_comPC1)) hitwd();
		ch = getser1(CommVals.m_comPC1);
		check_sum += ch;
		if (ch == '1') // Start
		{
			Vars.PulseDurationCounter[0] = 0;
			Vars.PulseDurationCounter[1] = 0;
			Vars.PulseDurationCounter[2] = 0;
			Vars.PulseDurationCounter[3] = 0;
			Vars.CycleCounter[0] = 0;
			Vars.CycleCounter[1] = 0;
			Vars.CycleCounter[2] = 0;
			Vars.CycleCounter[3] = 0;
			Vars.TriggerMode = 1;
		}
		else if (ch == '0') // Stop
		{
			Vars.TriggerMode = 0;
			outportb(0x0100, 0x00);
		}
	}
	else if (ch == '2')	// Configuration Parameters
	{
		memset(param_buf, 0, sizeof(param_buf));

		for (i=0; i<3; i++)
		{
			while (!serhit1(CommVals.m_comPC1)) hitwd();
  			param_buf[i] = getser1(CommVals.m_comPC1);
			check_sum += param_buf[i];
		}
		param = atoi(param_buf);

		memset(value_buf, 0, sizeof(value_buf));

		for (i=0; i<6; i++)
		{
			while (!serhit1(CommVals.m_comPC1)) hitwd();
  			value_buf[i] = getser1(CommVals.m_comPC1);
			check_sum += value_buf[i];
		}

		while (!serhit1(CommVals.m_comPC1)) hitwd();
		pc_check_sum = getser1(CommVals.m_comPC1);

		if (pc_check_sum != check_sum)
			return -1;

		val = atoi(value_buf);

		switch (param)
		{
			case PARAM_ID_SIGNAL_1_PULSE_DURATION :
				Vars.PulseDuration[0] = val * (Vars.TimerFrequency / 1000);
				Vars.PulseDurationHigh[0] = (int)((float)Vars.PulseDuration[0] / (float)Vars.PulseWidthsPerPeriod);
				break;
			case PARAM_ID_SIGNAL_2_PULSE_DURATION :
				Vars.PulseDuration[1] = val * (Vars.TimerFrequency / 1000);
				Vars.PulseDurationHigh[1] = (int)((float)Vars.PulseDuration[1] / (float)Vars.PulseWidthsPerPeriod);
				break;
			case PARAM_ID_SIGNAL_3_PULSE_DURATION :
				Vars.PulseDuration[2] = val * (Vars.TimerFrequency / 1000);
				Vars.PulseDurationHigh[2] = (int)((float)Vars.PulseDuration[2] / (float)Vars.PulseWidthsPerPeriod);
				break;
			case PARAM_ID_SIGNAL_4_PULSE_DURATION :
				Vars.PulseDuration[3] = val * (Vars.TimerFrequency / 1000);
				Vars.PulseDurationHigh[3] = (int)((float)Vars.PulseDuration[3] / (float)Vars.PulseWidthsPerPeriod);
				break;

			case PARAM_ID_NUMBER_OF_CYCLES :	Vars.NumberOfCycles = val; break;

			default : break;
		}
	}
	else
		return -1;

	return 0;
}
