/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65.2
        Device            :  PIC16LF18875
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/tmr1.h"
#include <xc.h>
#include "mcc_generated_files/mcc.h"
#include "I2C/i2c.h"
#include "Serial/definitions.h"

#define EEAddr 0xF000
#define NREG 20
#define RING_BUFFER_NUM_ADDRESS 100
#define LIMITS_ADDR 0xF064
#define CLOCK_STAMP_ADDR 0xF078
#define PARAM_ADDR 0xF07A

typedef struct data_register
{
    //timestamp
    uint8_t h;
    uint8_t m;
    uint8_t s;
    //temp
    uint8_t t;
    //lum
    uint8_t l;
}DataRegister;
 
typedef struct ring_buffer
{
    uint8_t write_pointer;
    uint8_t read_pointer;
    uint8_t nr;
    bool empty;
}RingBuffer;

enum State {NORMAL, CLOCK, ALARM, TEMPERATURE, LUMINOSITY, HT, HU, MT, MU, AF, TT, TU, LL}; //UI State
enum State M_STATE = NORMAL;
uint8_t hours = 0, ht = 0, hu = 0;
uint8_t minutes = 0, mt = 0, mu = 0;
uint8_t seconds = 0;
bool second_changed = false, minute_changed = false, hour_changed = false;
bool alarm = false, alaf = false; //alarm- if alarm is on alaf- if alarms are allowed
uint8_t tala = 2; //time that pwm should be on
uint8_t alarm_seconds = 0; //number of seconds elapsed with alarm on
uint8_t tt = 0, tu = 0 , alat = 30; //tt- temperature tens(alat), tu- temperature units(alat), alat - temperature threshold
unsigned char t = 0; //temperature reading from sensor
uint8_t luminosity = 0, alal = 2; // luminosity- reading from potenciometer, alal- lum threshold
uint8_t pmon = 3; //reading interval
uint8_t tmax = 0, tmin = 50, lmax = 0, lmin = 3, tlast = 0, llast = 0; //max, min and last values of readings
adc_result_t convertedValue = 0;
RingBuffer Rb = { 0x00, 0x00, 0, 1}; // buffer to store periodic readings if one of lum or temp changes
bool mhf = false;

bool RC5_LastValue = 1; // 1 means that RC5 is not being pressed

bool running = false;

void updateChecksum(void)
{
    uint8_t checkmsb = 0, checklsb = 0;
    uint16_t checksum = 0;
    
    checksum += tmin + tmax + lmin + lmax;
    checksum += hours + minutes;
    checksum += pmon + tala + alat + alal + alaf;
    checksum += Rb.write_pointer + Rb.read_pointer + Rb.nr + Rb.empty;
    
    checkmsb = ((checksum & 0xFF00) >> 8);
    checklsb = (checksum & 0x00FF);
    
    DATAEE_WriteByte(PARAM_ADDR + 0x0009, checkmsb);
    DATAEE_WriteByte(PARAM_ADDR + 0x000A, checklsb);
    
    return;
}

void loadParams(void)
{
    uint16_t checksum = 0, check = 0;
    uint8_t  checklsb = 0, checkmsb = 0;
    uint8_t pmon_t = 0, tala_t = 0, alat_t = 0, alal_t = 0, alaf_t = 0;  
    uint8_t wp_t = 0, rp_t = 0, nr_t = 0, empty_t = 0;
    uint8_t h_t = 0, m_t = 0;
    uint8_t tmin_t = 0, tmax_t = 0, lmin_t = 0, lmax_t = 0;
    
    tmin_t  = DATAEE_ReadByte(LIMITS_ADDR + 0x03);
    tmax_t  = DATAEE_ReadByte(LIMITS_ADDR + 0x05 + 0x03);
    lmin_t  = DATAEE_ReadByte(LIMITS_ADDR + 0x0A + 0x04);
    lmax_t  = DATAEE_ReadByte(LIMITS_ADDR + 0x0F + 0x04);
    
    h_t     = DATAEE_ReadByte(CLOCK_STAMP_ADDR);
    m_t     = DATAEE_ReadByte(CLOCK_STAMP_ADDR + 0x0001);
    
    pmon_t  = DATAEE_ReadByte(PARAM_ADDR);
    tala_t  = DATAEE_ReadByte(PARAM_ADDR + 0x0001);
    alat_t  = DATAEE_ReadByte(PARAM_ADDR + 0x0002);
    alal_t  = DATAEE_ReadByte(PARAM_ADDR + 0x0003);
    alaf_t  = DATAEE_ReadByte(PARAM_ADDR + 0x0004);
    
    wp_t    = DATAEE_ReadByte(PARAM_ADDR + 0x0005);
    rp_t    = DATAEE_ReadByte(PARAM_ADDR + 0x0006);
    nr_t 	= DATAEE_ReadByte(PARAM_ADDR + 0x0007);
    empty_t = DATAEE_ReadByte(PARAM_ADDR + 0x0008);
    
    checksum += tmin_t + tmax_t + lmin_t + lmax_t;
    checksum += h_t + m_t;
    checksum += pmon_t + tala_t + alat_t + alal_t + alaf_t;
    checksum += wp_t + rp_t + nr_t + empty_t;
    
    checkmsb = DATAEE_ReadByte(PARAM_ADDR + 0x0009);
    checklsb = DATAEE_ReadByte(PARAM_ADDR + 0x000A);
    
    check = ((checkmsb & 0x00FF) << 8) + (checklsb & 0x00FF);
    
    if(checksum == check && checksum != 65536)
    {
        tmin = tmin_t;
        tmax = tmax_t;
        lmin = lmin_t;
        lmax = lmax_t;
        
        hours = h_t;
        ht = hours/10;
        hu = hours%10;
        minutes = m_t;
        mt = minutes/10;
        mu = minutes%10;
        
        pmon = pmon_t;
        tala = tala_t;
        alat = alat_t;
        tt = alat/10;
        tu = alat%10;
        alal = alal_t;
        alaf = alaf_t;
        
        Rb.write_pointer = wp_t;
        Rb.read_pointer  = rp_t;
		Rb.nr = nr_t;
        Rb.empty = empty_t;
    }
    else
    {
        
        DATAEE_WriteByte(LIMITS_ADDR + 0x03, tmin);
        DATAEE_WriteByte(LIMITS_ADDR + 0x05 + 0x03, tmax);
        DATAEE_WriteByte(LIMITS_ADDR + 0x0A + 0x04, lmin);
        DATAEE_WriteByte(LIMITS_ADDR + 0x0F + 0x04, lmax);
        
        DATAEE_WriteByte(CLOCK_STAMP_ADDR, hours);
        DATAEE_WriteByte(CLOCK_STAMP_ADDR + 0x0001, minutes);
        
        DATAEE_WriteByte(PARAM_ADDR, pmon);
        DATAEE_WriteByte(PARAM_ADDR + 0x0001, tala);
        DATAEE_WriteByte(PARAM_ADDR + 0x0002, alat);
        tt = alat/10;
        tu = alat%10;
        DATAEE_WriteByte(PARAM_ADDR + 0x0003, alal);
        DATAEE_WriteByte(PARAM_ADDR + 0x0004, alaf);
        
        DATAEE_WriteByte(PARAM_ADDR + 0x0005, Rb.write_pointer);
        DATAEE_WriteByte(PARAM_ADDR + 0x0006, Rb.read_pointer);
        DATAEE_WriteByte(PARAM_ADDR + 0x0007, Rb.nr);
        DATAEE_WriteByte(PARAM_ADDR + 0x0008, Rb.empty);
        
        updateChecksum();
    }
}

void saveWritePointer(void)
{
    DATAEE_WriteByte(PARAM_ADDR + 0x0005, Rb.write_pointer);
    updateChecksum();
    return;
}

void saveReadPointer(void)
{
    DATAEE_WriteByte(PARAM_ADDR + 0x0006, Rb.read_pointer);
    updateChecksum();
    return;
}

void saveValidRegisters(void)
{
	DATAEE_WriteByte(PARAM_ADDR + 0x0007, Rb.nr);
	updateChecksum();
	return;
}

void saveEmptyFlag(void)
{
    DATAEE_WriteByte(PARAM_ADDR + 0x0008, Rb.empty);
    updateChecksum();
    return;
}

void saveLumMax(void)
{
    DATAEE_WriteByte(LIMITS_ADDR + 0x0F, hours);
    DATAEE_WriteByte(LIMITS_ADDR + 0x0F + 0x01, minutes);
    DATAEE_WriteByte(LIMITS_ADDR + 0x0F + 0x02, seconds);
    DATAEE_WriteByte(LIMITS_ADDR + 0x0F + 0x03, t);
    DATAEE_WriteByte(LIMITS_ADDR + 0x0F + 0x04, lmax);
    updateChecksum();
    return;
}

void saveLumMin(void)
{
    DATAEE_WriteByte(LIMITS_ADDR + 0x0A, hours);
    DATAEE_WriteByte(LIMITS_ADDR + 0x0A + 0x01, minutes);
    DATAEE_WriteByte(LIMITS_ADDR + 0x0A + 0x02, seconds);
    DATAEE_WriteByte(LIMITS_ADDR + 0x0A + 0x03, t);
    DATAEE_WriteByte(LIMITS_ADDR + 0x0A + 0x04, lmin);
    updateChecksum();
    return;
}

void saveTempMax(void)
{
    DATAEE_WriteByte(LIMITS_ADDR + 0x05, hours);
    DATAEE_WriteByte(LIMITS_ADDR + 0x05 + 0x01, minutes);
    DATAEE_WriteByte(LIMITS_ADDR + 0x05 + 0x02, seconds);
    DATAEE_WriteByte(LIMITS_ADDR + 0x05 + 0x03, tmax);
    DATAEE_WriteByte(LIMITS_ADDR + 0x05 + 0x04, luminosity);
    updateChecksum();
    return;
}

void saveTempMin(void)
{
    DATAEE_WriteByte(LIMITS_ADDR, hours);
    DATAEE_WriteByte(LIMITS_ADDR + 0x01, minutes);
    DATAEE_WriteByte(LIMITS_ADDR + 0x02, seconds);
    DATAEE_WriteByte(LIMITS_ADDR + 0x03, tmin);
    DATAEE_WriteByte(LIMITS_ADDR + 0x04, luminosity);
    updateChecksum();
    return;
}

bool RingBuffer_MHF(void)
{
    uint8_t n = 0, pointer = Rb.read_pointer;
    
    while(pointer != Rb.write_pointer)
    {
        pointer+= 0x05;
        if(pointer > RING_BUFFER_NUM_ADDRESS - 5 )
            pointer = 0x00;
        n++;
    }
    
    return (n > NREG/2) ? true:false;
    
}

void RingBuffer_WriteRegister(uint8_t h, uint8_t m, uint8_t s, uint8_t t, uint8_t l)
{
    if (Rb.empty)
    {
        Rb.empty = 0;
        saveEmptyFlag();
    }
    
    DATAEE_WriteByte(EEAddr + Rb.write_pointer, h);
    DATAEE_WriteByte(EEAddr + Rb.write_pointer + 0x01, m);
    DATAEE_WriteByte(EEAddr + Rb.write_pointer + 0x02, s);
    DATAEE_WriteByte(EEAddr + Rb.write_pointer + 0x03, t);
    DATAEE_WriteByte(EEAddr + Rb.write_pointer + 0x04, l);

    Rb.write_pointer += 0x05;
    if(Rb.nr < NREG)
        Rb.nr++;
		saveValidRegisters();
    if (Rb.write_pointer > RING_BUFFER_NUM_ADDRESS - 5)
        Rb.write_pointer = 0x00;
    
    saveWritePointer();
    
    return;
}

DataRegister RingBuffer_ReadRegisterFromIndex(uint8_t index)
{
    DataRegister result;
    uint8_t oldest_pointer;
    
    if(Rb.nr < NREG)
        oldest_pointer = 0x00;
    else
        oldest_pointer = Rb.write_pointer;

    if (oldest_pointer > RING_BUFFER_NUM_ADDRESS - 5)
        oldest_pointer = 0x00;

    while(index > 0)
    {
        oldest_pointer += 0x05;
        if (oldest_pointer > RING_BUFFER_NUM_ADDRESS - 5)
            oldest_pointer = 0x00;
        index--;
    }

    result.h = DATAEE_ReadByte(EEAddr + oldest_pointer);
    result.m = DATAEE_ReadByte(EEAddr + oldest_pointer + 0x01);
    result.s = DATAEE_ReadByte(EEAddr + oldest_pointer + 0x02);
    result.t = DATAEE_ReadByte(EEAddr + oldest_pointer + 0x03);
    result.l = DATAEE_ReadByte(EEAddr + oldest_pointer + 0x04);
 
    return result;
}

DataRegister RingBuffer_ReadRegister(void)
{
    DataRegister result;
 
    result.h = DATAEE_ReadByte(EEAddr + Rb.read_pointer);
    result.m = DATAEE_ReadByte(EEAddr + Rb.read_pointer + 0x01);
    result.s = DATAEE_ReadByte(EEAddr + Rb.read_pointer + 0x02);
    result.t = DATAEE_ReadByte(EEAddr + Rb.read_pointer + 0x03);
    result.l = DATAEE_ReadByte(EEAddr + Rb.read_pointer + 0x04);
 
    Rb.read_pointer += 0x05;
   
    if (Rb.read_pointer > RING_BUFFER_NUM_ADDRESS - 5)
        Rb.read_pointer = 0x00;
    
    saveReadPointer();
    
    return result;
}

unsigned char tsttc(void)
{
	unsigned char value;
do{
	IdleI2C();
	StartI2C(); IdleI2C();
    
	WriteI2C(0x9a | 0x00); IdleI2C();
	WriteI2C(0x01); IdleI2C();
	RestartI2C(); IdleI2C();
	WriteI2C(0x9a | 0x01); IdleI2C();
	value = ReadI2C(); IdleI2C();
	NotAckI2C(); IdleI2C();
	StopI2C();
} while (!(value & 0x40));

	IdleI2C();
	StartI2C(); IdleI2C();
	WriteI2C(0x9a | 0x00); IdleI2C();
	WriteI2C(0x00); IdleI2C();
	RestartI2C(); IdleI2C();
	WriteI2C(0x9a | 0x01); IdleI2C();
	value = ReadI2C(); IdleI2C();
	NotAckI2C(); IdleI2C();
	StopI2C();

	return value;
}

void PWM_Output_D4_Enable(void)
{
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x00;
    
    RA6PPS = 0x0E;
    
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x01;
}

void PWM_Output_D4_Disable(void)
{
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x00;
    
    RA6PPS = 0x00;
    
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x01;
}

void processS1(void)
{
	switch (M_STATE)
	{
		case NORMAL:
			if (!IO_RB4_GetValue())
            {
                if(alarm)
                {
                    alarm = !alarm;
                    if(alarm_seconds <= tala)
                    {
                         TMR2_StopTimer();
                         PWM_Output_D4_Disable();
                    }
                    alarm_seconds = 0;
                }
                else
                {
                    M_STATE = CLOCK;
                }
            }
			break;
		case CLOCK:
			if (!IO_RB4_GetValue())
				M_STATE = ALARM;
			break;
		case ALARM:
			if (!IO_RB4_GetValue())
				M_STATE = TEMPERATURE;
			break;
		case TEMPERATURE:
			if (!IO_RB4_GetValue())
				M_STATE = LUMINOSITY;
			break;
		case LUMINOSITY:
			if (!IO_RB4_GetValue())
				M_STATE = NORMAL;
			break;
		case HT:
			if (!IO_RB4_GetValue())
				M_STATE = HU;
			break;
		case HU:
			if (!IO_RB4_GetValue())
				M_STATE = MT;
			break;
		case MT:
			if (!IO_RB4_GetValue())
				M_STATE = MU;
			break;
		case MU:
			if (!IO_RB4_GetValue())
				M_STATE = NORMAL;
			break;
		case AF:
			if (!IO_RB4_GetValue())
				M_STATE = NORMAL;
			break;
		case TT:
			if (!IO_RB4_GetValue())
				M_STATE = TU;
			break;
		case TU:
			if (!IO_RB4_GetValue())
				M_STATE = NORMAL;
			break;
		case LL:
			if (!IO_RB4_GetValue())
				M_STATE = NORMAL;
			break;
		default:
			break;
	}
}

void processS2(void)
{
    switch (M_STATE)
	{
		case CLOCK:
			M_STATE = HT;
			break;
		case ALARM:
			M_STATE = AF;
			break;
		case TEMPERATURE:
			M_STATE = TT;
			break;
		case LUMINOSITY:
			M_STATE = LL;
			break;
		case HT:
			ht++;
			ht = ht % 3;
            if (ht == 2)
			{
				hu = hu % 4;
			}
            hours = ht * 10 + hu;
            DATAEE_WriteByte(CLOCK_STAMP_ADDR, hours);
            updateChecksum();
			break;
		case HU:
			hu++;
			if (ht != 2)
			{
				hu = hu % 10;
			}
			if (ht == 2)
			{
				hu = hu % 4;
			}
            hours = ht * 10 + hu;
            DATAEE_WriteByte(CLOCK_STAMP_ADDR, hours);
            updateChecksum();
			break;
		case MT:
			mt++;
			mt = mt % 6;
            minutes = mt * 10 + mu;
            DATAEE_WriteByte(CLOCK_STAMP_ADDR + 0x0001, minutes);
            updateChecksum();
			break;
		case MU:
			mu++;
			mu = mu % 10;
            minutes = mt * 10 + mu;
            DATAEE_WriteByte(CLOCK_STAMP_ADDR + 0x0001, minutes);
            updateChecksum();
			break;
		case AF:
            if(!alarm)
            {
                alaf = !alaf;
                DATAEE_WriteByte(PARAM_ADDR + 0x0004, alaf);
                updateChecksum();
            }
			break;
		case TT:
			tt++;
			tt = tt % 6;
            if(tt == 5)
                tu = 0;
            alat = tt * 10 + tu;
            DATAEE_WriteByte(PARAM_ADDR + 0x0002, alat);
            updateChecksum();
			break;
		case TU:
			tu++;
			if (tt != 5)
			{
				tu = tu % 10;
			}
			if (tt == 5)
				tu = 0;
            alat = tt * 10 + tu;
            DATAEE_WriteByte(PARAM_ADDR + 0x0002, alat);
            updateChecksum();
            
			break;
		case LL:
			alal++;
			alal = alal%4;
            DATAEE_WriteByte(PARAM_ADDR + 0x0003, alal);
            updateChecksum();
			break;
		default:
			break;
	}
}

void displayUI(void)
{
	unsigned char bit_t = 1;
	switch (M_STATE)
	{
		case NORMAL:
            
            if(seconds%2)
                IO_RA7_SetHigh();
            else
                IO_RA7_SetLow();
                
			if(alarm && alarm_seconds > 2)
                IO_RA6_SetHigh();
            else
                IO_RA6_SetLow();
			if(luminosity & (bit_t << 1))
                IO_RA5_SetHigh();
            else
                IO_RA5_SetLow();
			if(luminosity & bit_t)
                IO_RA4_SetHigh();
            else
                IO_RA4_SetLow();
			break;
		case CLOCK:
            if(seconds%2 == 0)
                IO_RA7_SetHigh();
            else
                IO_RA7_SetLow();
			break;
		case ALARM:
			if(seconds%2 == 0)
                IO_RA6_SetHigh();
            else
                IO_RA6_SetLow();
			break;
		case TEMPERATURE:
			if(seconds%2 == 0)
                IO_RA5_SetHigh();
            else
                IO_RA5_SetLow();
			break;
		case LUMINOSITY:
			if(seconds%2 == 0)
                IO_RA4_SetHigh();
            else
                IO_RA4_SetLow();
			break;
		case HT:
			if(ht & (bit_t << 3))
                IO_RA7_SetHigh();
            else
                IO_RA7_SetLow();
			if(ht & (bit_t << 2))
                IO_RA6_SetHigh();
            else
                IO_RA6_SetLow();
			if(ht & (bit_t << 1))
                IO_RA5_SetHigh();
            else
                IO_RA5_SetLow();
			if(ht & (bit_t ))
                IO_RA4_SetHigh();
            else
                IO_RA4_SetLow();
			break;
		case HU:
			if(hu & (bit_t << 3))
                IO_RA7_SetHigh();
            else
                IO_RA7_SetLow();
			if(hu & (bit_t << 2))
                IO_RA6_SetHigh();
            else
                IO_RA6_SetLow();
			if(hu & (bit_t << 1))
                IO_RA5_SetHigh();
            else
                IO_RA5_SetLow();
			if(hu & (bit_t ))
                IO_RA4_SetHigh();
            else
                IO_RA4_SetLow();
			break;
		case MT:
			if(mt & (bit_t << 3))
                IO_RA7_SetHigh();
            else
                IO_RA7_SetLow();
			if(mt & (bit_t << 2))
                IO_RA6_SetHigh();
            else
                IO_RA6_SetLow();
			if(mt & (bit_t << 1))
                IO_RA5_SetHigh();
            else
                IO_RA5_SetLow();
			if(mt & (bit_t ))
                IO_RA4_SetHigh();
            else
                IO_RA4_SetLow();
			break;
		case MU:
			if(mu & (bit_t << 3))
                IO_RA7_SetHigh();
            else
                IO_RA7_SetLow();
			if(mu & (bit_t << 2))
                IO_RA6_SetHigh();
            else
                IO_RA6_SetLow();
			if(mu & (bit_t << 1))
                IO_RA5_SetHigh();
            else
                IO_RA5_SetLow();
			if(mu & (bit_t ))
                IO_RA4_SetHigh();
            else
                IO_RA4_SetLow();
			break;
		case AF:
			if(alaf)
                IO_RA4_SetHigh();
            else
                IO_RA4_SetLow();
			break;
		case TT:
			if(tt & (bit_t << 3))
                IO_RA7_SetHigh();
            else
                IO_RA7_SetLow();
			if(tt & (bit_t << 2))
                IO_RA6_SetHigh();
            else
                IO_RA6_SetLow();
			if(tt & (bit_t << 1))
                IO_RA5_SetHigh();
            else
                IO_RA5_SetLow();
			if(tt & (bit_t ))
                IO_RA4_SetHigh();
            else
                IO_RA4_SetLow();
			break;
		case TU:
			if(tu & (bit_t << 3))
                IO_RA7_SetHigh();
            else
                IO_RA7_SetLow();
			if(tu & (bit_t << 2))
                IO_RA6_SetHigh();
            else
                IO_RA6_SetLow();
			if(tu & (bit_t << 1))
                IO_RA5_SetHigh();
            else
                IO_RA5_SetLow();
			if(tu & (bit_t ))
                IO_RA4_SetHigh();
            else
                IO_RA4_SetLow();
			break;
		case LL:
			
			if(alal & (bit_t << 1))
                IO_RA5_SetHigh();
            else
                IO_RA5_SetLow();
			if(alal & (bit_t ))
                IO_RA4_SetHigh();
            else
                IO_RA4_SetLow();
			break;
		default:
			break;
	}

	return;
}

void resetUI(void)
{
	IO_RA4_SetLow();
	IO_RA5_SetLow();
	IO_RA6_SetLow();
	IO_RA7_SetLow();

	return;
}

void count(void)
{
    second_changed = true;
    
    
    seconds += 1;
    if(seconds >= 60)
    {
        seconds = 0;
        minutes++;
        mt = minutes/10;
        mu = minutes%10;
        minute_changed = true;
    }
    if(minutes >= 60)
    {
        minutes = 0;
        mt = 0;
        mu = 0;
        hours++;
        ht = hours/10;
        hu = hours%10;
        minute_changed = true;
        hour_changed = true;
    }
    if(hours >= 24)
    {
        hours = 0;
        ht = 0;
        hu = 0;
        hour_changed = true;
    }
    
    TMR1IF = 0;
    TMR1_Reload();
}

void processCommand(uint8_t com)
{
    DataRegister r;
    uint8_t n, i;
    
    switch(com)
    {
        case RCLK:
            EUSART_Write(SOM);
            EUSART_Write(RCLK);
            EUSART_Write(hours);
            EUSART_Write(minutes);
            EUSART_Write(seconds);
            EUSART_Write(EOM);
            break;
        case SCLK:
			if(com_args[0] >= 0 && com_args[0] <= 23 && com_args[1] >= 0 && com_args[1] <= 59 && com_args[2] >= 0 && com_args[2] <= 59)
			{
				hours = com_args[0];
				ht = hours/10;
				hu = hours%10;
				minutes = com_args[1];
				mt = minutes/10;
				mu = minutes%10;
				seconds = com_args[2];
				
				DATAEE_WriteByte(CLOCK_STAMP_ADDR, hours);
				DATAEE_WriteByte(CLOCK_STAMP_ADDR + 0x0001, minutes);
				updateChecksum();

				EUSART_Write(SOM);
				EUSART_Write(SCLK);
				EUSART_Write(CMD_OK);
			}
			else
			{
				EUSART_Write(SOM);
				EUSART_Write(SCLK);
				EUSART_Write(CMD_ERROR);
			}
            EUSART_Write(EOM);
            break;
        case RTL:
            EUSART_Write(SOM);
            EUSART_Write(RTL);
            EUSART_Write(t);
            EUSART_Write(luminosity);
            EUSART_Write(EOM);
            break;
        case RPAR:
            EUSART_Write(SOM);
            EUSART_Write(RPAR);
            EUSART_Write(pmon);
            EUSART_Write(tala);
            EUSART_Write(EOM);
            break;
        case MMP:
			if(com_args[0] >= 0 && com_args[0] <= 99)
			{
				pmon = com_args[0];
				DATAEE_WriteByte(PARAM_ADDR, pmon);
				updateChecksum();
				EUSART_Write(SOM);
				EUSART_Write(MMP);
				EUSART_Write(CMD_OK);
			}
			else
			{
				EUSART_Write(SOM);
				EUSART_Write(MMP);
				EUSART_Write(CMD_ERROR);
			}
            EUSART_Write(EOM);
            break;
        case MTA:
			if(com_args[0] >= 0 && com_args[0] <= 60)
			{
				tala = com_args[0];
				DATAEE_WriteByte(PARAM_ADDR + 0x0001, tala);
				updateChecksum();
				EUSART_Write(SOM);
				EUSART_Write(MTA);
				EUSART_Write(CMD_OK);
			}
			else
			{
				EUSART_Write(SOM);
				EUSART_Write(MTA);
				EUSART_Write(CMD_ERROR);
			}
            EUSART_Write(EOM);
            break;
        case RALA:
            EUSART_Write(SOM);
            EUSART_Write(RALA);
            EUSART_Write(alat);
            EUSART_Write(alal);
            EUSART_Write(alaf);
            EUSART_Write(EOM);
            break;
        case DATL:
			if(com_args[0] >= 0 && com_args[0] <= 50 && com_args[1] >= 0 && com_args[1] <= 3)
			{
				alat = com_args[0];
				DATAEE_WriteByte(PARAM_ADDR + 0x0002, alat);
				alal = com_args[1];
				DATAEE_WriteByte(PARAM_ADDR + 0x0003, alal);
				updateChecksum();
				EUSART_Write(SOM);
				EUSART_Write(DATL);
				EUSART_Write(CMD_OK);
			}
			else
			{
				EUSART_Write(SOM);
				EUSART_Write(DATL);
				EUSART_Write(CMD_ERROR);
			}
            EUSART_Write(EOM);
            break;
        case AALA:
			if(com_args[0] == 0 || com_args[0] == 1)
			{
				alaf = com_args[0];
				DATAEE_WriteByte(PARAM_ADDR + 0x0004, alaf);
				updateChecksum();
				EUSART_Write(SOM);
				EUSART_Write(AALA);
				EUSART_Write(CMD_OK);
			}
			else
			{
				EUSART_Write(SOM);
				EUSART_Write(AALA);
				EUSART_Write(CMD_ERROR);
			}
            EUSART_Write(EOM);
            break;
        case RMM:
            EUSART_Write(SOM);
            EUSART_Write(RMM);
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x01));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x02));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x03));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x04));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x05));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x06));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x07));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x08));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x09));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x0A));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x0B));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x0C));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x0D));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x0E));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x0F));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x10));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x11));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x12));
            EUSART_Write(DATAEE_ReadByte(LIMITS_ADDR + 0x13));            
            EUSART_Write(EOM);
            break;
        case CMM:
            EUSART_Write(SOM);
            EUSART_Write(CMM);
            DATAEE_WriteByte(LIMITS_ADDR, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x01, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x02, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x03, 50);
            DATAEE_WriteByte(LIMITS_ADDR + 0x04, 0);
            tmax = 0;
            DATAEE_WriteByte(LIMITS_ADDR + 0x05, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x06, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x07, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x08, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x09, 0);
            tmin = 50;
            DATAEE_WriteByte(LIMITS_ADDR + 0x0A, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x0B, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x0C, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x0D, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x0E, 3);
            lmax = 0;
            DATAEE_WriteByte(LIMITS_ADDR + 0x0F, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x10, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x11, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x12, 0);
            DATAEE_WriteByte(LIMITS_ADDR + 0x13, 0);
            lmin = 3;
            updateChecksum();
            EUSART_Write(CMD_OK);
            EUSART_Write(EOM);
            break;
        case IREG:
            EUSART_Write(SOM);
            EUSART_Write(IREG);
            EUSART_Write(NREG);
            EUSART_Write(Rb.nr);
            EUSART_Write(Rb.read_pointer);
            EUSART_Write(Rb.write_pointer);
            EUSART_Write(EOM);
            break;
        case TRGC:
            EUSART_Write(SOM);
            EUSART_Write(TRGC);
			if(com_args[0] >= 0 && com_args[0] <= 10)
			{
				while(com_args[0] > 0)
				{
					if(Rb.read_pointer != Rb.write_pointer)
					{
						r = RingBuffer_ReadRegister();
						EUSART_Write(r.h);
						EUSART_Write(r.m);
						EUSART_Write(r.s);
						EUSART_Write(r.t);
						EUSART_Write(r.l);
						com_args[0]--;
					}
					else
						break;
				}
			}            
            EUSART_Write(EOM);
            break;
        case TRGI:
            EUSART_Write(SOM);
            EUSART_Write(TRGI);
			if(com_args[0] >= 0 && com_args[0] <= 10 && com_args[1] >= 0 && com_args[1] <= NREG - 1)
			{
				if(com_args[1] < Rb.nr)
				{
					if(Rb.nr < NREG)
                    {
                        com_args[0] = Rb.nr - com_args[1];
                    }

					while(com_args[0] > 0)
					{
						r = RingBuffer_ReadRegisterFromIndex(com_args[1]);
						EUSART_Write(r.h);
						EUSART_Write(r.m);
						EUSART_Write(r.s);
						EUSART_Write(r.t);
						EUSART_Write(r.l);
						com_args[1]++;
						if(com_args[1] > NREG-1)
							com_args[1] = 0;
						com_args[0]--;
					}
				}
			}
            EUSART_Write(EOM);
            break;
        default:
            break;
    }
    
    return;
}

void main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    TMR1_SetInterruptHandler(count);
    INT_SetInterruptHandler(processS1);
    
    i2c1_driver_open();
    
    loadParams();
    
    while (1)
    {
        if(!running)
        {
            running = true;
            resetUI();
            TMR1_StartTimer();
        }
        else
        {
            if(second_changed)
            {
                if(alarm)
                {
                    if(alarm_seconds == 0)
                    {
                        TMR2_StartTimer();
                        PWM_Output_D4_Enable();
                    }
                    
                    alarm_seconds++;
                }
                
                if( pmon != 0 && seconds%pmon == 0)
                {
                    convertedValue = ADCC_GetSingleConversion(channel_ANA0);
                    luminosity = (convertedValue >> 14);

                    NOP();
                    t = tsttc();       		
                    NOP();

                    if(luminosity > alal && alaf)
                        alarm = true;
                    
                    if(t > alat && alaf)
                        alarm = true;
                    
                    if(luminosity != llast && t != tlast)
                    {
                        tlast = t;
                        llast = luminosity;
                        RingBuffer_WriteRegister(hours, minutes, seconds, t, luminosity);
                    }
                    else if(luminosity != llast)
                    {
                        llast = luminosity;
                        RingBuffer_WriteRegister(hours, minutes, seconds, t, luminosity);
                    }                    
                    else if(t != tlast)
                    {
                        tlast = t;
                        RingBuffer_WriteRegister(hours, minutes, seconds, t, luminosity);
                    } 
                    
                    if(luminosity > lmax)
                    {
                        lmax = luminosity;
                        saveLumMax();
                    }

                    if(luminosity < lmin)
                    {
                        lmin = luminosity;
                        saveLumMin();
                    }  

                    if(t > tmax)
                    {
                        tmax = t;
                        saveTempMax();
                    }

                    if(t < tmin)
                    {
                        tmin = t;
                        saveTempMin();
                    }
                }
                
                second_changed = false;
            }
            
            if(minute_changed)
            {
                DATAEE_WriteByte(CLOCK_STAMP_ADDR + 0x0001, minutes);
                updateChecksum();
                minute_changed = false;
            }
            
            if(hour_changed)
            {
                DATAEE_WriteByte(CLOCK_STAMP_ADDR, hours);
                updateChecksum();
                hour_changed = false;
            }
            
            if(alarm_seconds > tala)
            {
                TMR2_StopTimer();
                PWM_Output_D4_Disable();
            }     
            
            while(EUSART_is_rx_ready())
            {
                uint8_t rec = 0;
                rec = EUSART_Read();
                
                if(message_started)
                {
                    message_started = false;
                    arg_pointer = 0;
                    command = rec;
                }
                else if(rec != SOM && rec != EOM)
                {
                    com_args[arg_pointer++] = rec;
                }
                
                if (rec == SOM)
                    message_started = true;
                
                if(rec == EOM)
                    processCommand(command);
            }
            
            if(RingBuffer_MHF())
            {
                if(!mhf)
                {
                    mhf = true;
                    //notify pc
                     EUSART_Write(SOM);
                     EUSART_Write(NMFL);
                     EUSART_Write(NREG);
                     EUSART_Write(Rb.nr);
                     EUSART_Write(Rb.read_pointer);
                     EUSART_Write(Rb.write_pointer);
                     EUSART_Write(EOM);
                }
            }
            else
            {
                if(mhf)
                    mhf = false;
            }
            /*
            if(M_STATE == NORMAL && !alarm)
            {
                NOP();
                SLEEP();
                NOP();
            }
            */
            resetUI();
            
            if(!IO_RC5_GetValue() && IO_RC5_GetValue() != RC5_LastValue)
                processS2();
            RC5_LastValue = IO_RC5_GetValue();
            
            displayUI();
        }
    }
}
/**
 End of File
*/