/***************************************************************************
| File: comando.c  -  Concretizacao de comandos (exemplo)
|
| Autor: Carlos Almeida (IST)
| Data:  Maio 2008
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "ringbuf.h"

Cyg_ErrNo err;

Register readRegister(cyg_handle_t rx_box2_h)
{
    Register result;
    
    result.h = cyg_mbox_get(rx_box2_h);
    result.m = cyg_mbox_get(rx_box2_h);
    result.s = cyg_mbox_get(rx_box2_h);
    result.temp = cyg_mbox_get(rx_box2_h);
    result.lum = cyg_mbox_get(rx_box2_h);
    
    return result;
}
/*-------------------------------------------------------------------------+
| Function: cmd_sair - termina a aplicacao
+--------------------------------------------------------------------------*/
void cmd_sair (int argc, char **argv)
{
  exit(0);
}
/*-------------------------------------------------------------------------+
| Function: cmd_rc - read clock
+--------------------------------------------------------------------------*/
void cmd_rc (int argc, char** argv)
{
    unsigned int n;    
    unsigned char buf[20];

    if(argc != 1)
    {
    	cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
   //send command to tx thread
   cyg_mutex_lock(&transmit_mux);
   cyg_mbox_put(tx_box1_h, SOM);
   cyg_mbox_put(tx_box1_h, RCLK);
   cyg_mbox_put(tx_box1_h, EOM);
   cyg_mutex_unlock(&transmit_mux);
   //get som cmd h m s eom
   buf[0] = cyg_mbox_get(rx_box2_h);
   buf[1] = cyg_mbox_get(rx_box2_h);
   buf[2] = cyg_mbox_get(rx_box2_h);
   buf[3] = cyg_mbox_get(rx_box2_h);
   buf[4] = cyg_mbox_get(rx_box2_h);
   buf[5] = cyg_mbox_get(rx_box2_h);
   //print result
   cyg_mutex_lock(&clipmux);
   printf("io_err = %x, h:%d m:%d s:%d\n", err, buf[2], buf[3], buf[4]);
   cyg_mutex_unlock(&clipmux);
}
/*-------------------------------------------------------------------------+
| Function: cmd_sc - set clock
+--------------------------------------------------------------------------*/
void cmd_sc (int argc, char** argv)
{
    uint8_t htemp, mtemp, stemp, m, temp;

    if(argc != 4)
    {
    	cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
	
	htemp = atoi(argv[1]);
	mtemp = atoi(argv[2]);
	stemp = atoi(argv[3]);
	
    //send command to tx
    cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);
    cyg_mbox_put(tx_box1_h, SCLK);
    cyg_mbox_put(tx_box1_h, htemp);
    cyg_mbox_put(tx_box1_h, mtemp);
    cyg_mbox_put(tx_box1_h, stemp);
    cyg_mbox_put(tx_box1_h, EOM);
    cyg_mutex_unlock(&transmit_mux);
    //wait for CMD_OK or CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get SOM
    temp = cyg_mbox_get(rx_box2_h);//Get CMD
    m = cyg_mbox_get(rx_box2_h);//Get CMD_OK/CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get EOM
    
    if(m == CMD_OK)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Clock set!\n");
    	cyg_mutex_unlock(&clipmux);
    }
    if(m == CMD_ERROR)
    {
	    cyg_mutex_lock(&clipmux);
    	printf("Error setting clock!\n");
    	cyg_mutex_unlock(&clipmux);
    }
}
/*-------------------------------------------------------------------------+
| Function: cmd_rtl - read temperature and luminosity
+--------------------------------------------------------------------------*/
void cmd_rtl (int argc, char** argv)
{
    unsigned int n;    
    unsigned char buf[20];

    if(argc != 1)
    {
    	cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
    //Send SOM RTL EOM
    cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);    
	cyg_mbox_put(tx_box1_h, RTL);
	cyg_mbox_put(tx_box1_h, EOM);
	cyg_mutex_unlock(&transmit_mux);
	//get SOM CMD T L EOM
	buf[0] = cyg_mbox_get(rx_box2_h);
	buf[1] = cyg_mbox_get(rx_box2_h);
	buf[2] = cyg_mbox_get(rx_box2_h);
	buf[3] = cyg_mbox_get(rx_box2_h);
	buf[4] = cyg_mbox_get(rx_box2_h);
	//print result
	cyg_mutex_lock(&clipmux);
	printf("io_err = %x, t:%d l:%d \n", err, buf[2], buf[3]);
	cyg_mutex_unlock(&clipmux);
}
/*-------------------------------------------------------------------------+
| Function: cmd_rp - read parameters (pmon and tala)
+--------------------------------------------------------------------------*/
void cmd_rp (int argc, char** argv)
{
	unsigned int n;    
    unsigned char buf[20];

    if(argc != 1)
    {
    	cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
        
	//Send SOM CMD EOM
	cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);    
	cyg_mbox_put(tx_box1_h, RPAR);
	cyg_mbox_put(tx_box1_h, EOM);
	cyg_mutex_unlock(&transmit_mux);
	//get SOM CMD pmon tala EOM
	buf[0] = cyg_mbox_get(rx_box2_h);
	buf[1] = cyg_mbox_get(rx_box2_h);
	buf[2] = cyg_mbox_get(rx_box2_h);
	buf[3] = cyg_mbox_get(rx_box2_h);
	buf[4] = cyg_mbox_get(rx_box2_h);
	//print result
	cyg_mutex_lock(&clipmux);
	printf("io_err = %x, pmon:%d tala:%d \n", err, buf[2], buf[3]);
	cyg_mutex_unlock(&clipmux);
}
/*-------------------------------------------------------------------------+
| Function: cmd_mmp - change monitoring period
+--------------------------------------------------------------------------*/
void cmd_mmp (int argc, char** argv)
{
	uint8_t pmontemp, m, temp;

    if(argc != 2)
    {
    	cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
	
	pmontemp = atoi(argv[1]);
	
    //Send SOM CMD EOM
    cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);    
	cyg_mbox_put(tx_box1_h, MMP);
	cyg_mbox_put(tx_box1_h, pmontemp);
	cyg_mbox_put(tx_box1_h, EOM);
	cyg_mutex_unlock(&transmit_mux);
    //wait for CMD_OK or CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get SOM
    temp = cyg_mbox_get(rx_box2_h);//Get CMD
    m = cyg_mbox_get(rx_box2_h);//Get CMD_OK/CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get EOM
    
    if(m == CMD_OK)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Monitoring period set!\n");
    	cyg_mutex_unlock(&clipmux);
    }
    if(m == CMD_ERROR)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Error setting monitoring period!\n");
    	cyg_mutex_unlock(&clipmux);
    }
}
/*-------------------------------------------------------------------------+
| Function: cmd_mta - change alarm time
+--------------------------------------------------------------------------*/
void cmd_mta (int argc, char** argv)
{
	uint8_t talatemp, m, temp;

    if(argc != 2)
    {
    	cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
	
	talatemp = atoi(argv[1]);
	
    //Send SOM CMD ... EOM
    cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);    
	cyg_mbox_put(tx_box1_h, MTA);
	cyg_mbox_put(tx_box1_h, talatemp);
	cyg_mbox_put(tx_box1_h, EOM);
	cyg_mutex_unlock(&transmit_mux);
    //wait for CMD_OK or CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get SOM
    temp = cyg_mbox_get(rx_box2_h);//Get CMD
    m = cyg_mbox_get(rx_box2_h);//Get CMD_OK/CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get EOM
    
    if(m == CMD_OK)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Alarm time set!\n");
    	cyg_mutex_unlock(&clipmux);
    }
    if(m == CMD_ERROR)
    {
	    cyg_mutex_lock(&clipmux);
    	printf("Error setting alarm time!\n");
    	cyg_mutex_unlock(&clipmux);
    }
}
/*-------------------------------------------------------------------------+
| Function: cmd_ra - read alat alal and alaf
+--------------------------------------------------------------------------*/
void cmd_ra (int argc, char** argv)
{
	unsigned int n;    
    unsigned char buf[20];

    if(argc != 1)
    {
    	cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
        
	//Send SOM CMD EOM
	cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);    
	cyg_mbox_put(tx_box1_h, RALA);
	cyg_mbox_put(tx_box1_h, EOM);
	cyg_mutex_unlock(&transmit_mux);
	//get alat alal alaf
	buf[0] = cyg_mbox_get(rx_box2_h);
	buf[1] = cyg_mbox_get(rx_box2_h);
	buf[2] = cyg_mbox_get(rx_box2_h);
	buf[3] = cyg_mbox_get(rx_box2_h);
	buf[4] = cyg_mbox_get(rx_box2_h);
	buf[5] = cyg_mbox_get(rx_box2_h);
	//print result
	cyg_mutex_lock(&clipmux);
	printf("io_err = %x, t:%d l:%d a:%d \n", err, buf[2], buf[3], buf[4]);
	cyg_mutex_unlock(&clipmux);
}
/*-------------------------------------------------------------------------+
| Function: cmd_dtl - set alarm temperature and luminosity
+--------------------------------------------------------------------------*/
void cmd_dtl (int argc, char** argv)
{
	uint8_t ttemp, ltemp, m, temp;

    if(argc != 3)
    {
    	cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
	
	ttemp = atoi(argv[1]);
	ltemp = atoi(argv[2]);
	
    //send command to tx
    cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);
    cyg_mbox_put(tx_box1_h, DATL);
    cyg_mbox_put(tx_box1_h, ttemp);
    cyg_mbox_put(tx_box1_h, ltemp);
    cyg_mbox_put(tx_box1_h, EOM);
    cyg_mutex_unlock(&transmit_mux);
    //wait for CMD_OK or CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get SOM
    temp = cyg_mbox_get(rx_box2_h);//Get CMD
    m = cyg_mbox_get(rx_box2_h);//Get CMD_OK/CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get EOM
    
    if(m == CMD_OK)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Alarm thresholds set!\n");
    	cyg_mutex_unlock(&clipmux);
    }
    if(m == CMD_ERROR)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Error setting alarm thresholds!\n");
    	cyg_mutex_unlock(&clipmux);
    }
}
/*-------------------------------------------------------------------------+
| Function: cmd_aa - activate/deactivate alarms
+--------------------------------------------------------------------------*/
void cmd_aa (int argc, char** argv)
{
	uint8_t atemp, m, temp;

    if(argc != 2)
    {
    	cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
	
	atemp = atoi(argv[1]);
	
    //send command to tx
    cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);
    cyg_mbox_put(tx_box1_h, AALA);
    cyg_mbox_put(tx_box1_h, atemp);
    cyg_mbox_put(tx_box1_h, EOM);
    cyg_mutex_unlock(&transmit_mux);
    //wait for CMD_OK or CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get SOM
    temp = cyg_mbox_get(rx_box2_h);//Get CMD
    m = cyg_mbox_get(rx_box2_h);//Get CMD_OK/CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get EOM
    
    if(m == CMD_OK)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Alarm flag changed!\n");
    	cyg_mutex_unlock(&clipmux);
    }
    if(m == CMD_ERROR)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Error changing alarm flag !\n");
    	cyg_mutex_unlock(&clipmux);
    }
}
/*-------------------------------------------------------------------------+
| Function: cmd_rmm - read maximum and minimum registers
+--------------------------------------------------------------------------*/
void cmd_rmm (int argc, char** argv)
{
	unsigned int n,i;    
    unsigned char buf[20], temp;
	Register r;
	
    if(argc != 1)
    {
    	cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
        
	//Send SOM CMD EOM
	cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);    
	cyg_mbox_put(tx_box1_h, RMM);
	cyg_mbox_put(tx_box1_h, EOM);
	cyg_mutex_unlock(&transmit_mux);
	//get registers
	temp = cyg_mbox_get(rx_box2_h);//Get SOM
	temp = cyg_mbox_get(rx_box2_h);//Get CMD
	for(i = 0; i < 4; i++)
	{
		r = readRegister(rx_box2_h);
		//print result
		cyg_mutex_lock(&clipmux);
		printf("h:%d m:%d s:%d t:%d l:%d \n", r.h, r.m, r.s, r.temp, r.lum);
		cyg_mutex_unlock(&clipmux);
	}
	temp = cyg_mbox_get(rx_box2_h);//Get EOM
}
/*-------------------------------------------------------------------------+
| Function: cmd_cmm -
+--------------------------------------------------------------------------*/
void cmd_cmm (int argc, char** argv)
{
	char m, temp;
	//Send SOM CMD EOM
	cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);    
	cyg_mbox_put(tx_box1_h, CMM);
	cyg_mbox_put(tx_box1_h, EOM);
	cyg_mutex_unlock(&transmit_mux);
	//wait for CMD_OK or CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get SOM
    temp = cyg_mbox_get(rx_box2_h);//Get CMD
    m = cyg_mbox_get(rx_box2_h);//Get CMD_OK/CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get EOM
    
    if(m == CMD_OK)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Maximum and minimum registers cleared!\n");
    	cyg_mutex_unlock(&clipmux);
    }
    if(m == CMD_ERROR)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Error clearing maximum and minimum registers !\n");
    	cyg_mutex_unlock(&clipmux);
    }
}
/*-------------------------------------------------------------------------+
| Function: cmd_ir -information about registers(N nr iread iwrite)
+--------------------------------------------------------------------------*/
void cmd_ir (int argc, char** argv)
{
	unsigned int n;    
    unsigned char buf[20], temp;
	Register r;
	
    if(argc != 1)
    {
    	cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
        
	//Send SOM CMD EOM
	cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);    
	cyg_mbox_put(tx_box1_h, IREG);
	cyg_mbox_put(tx_box1_h, EOM);
	cyg_mutex_unlock(&transmit_mux);
	//get registers
	temp = cyg_mbox_get(rx_box2_h);//Get SOM
	temp = cyg_mbox_get(rx_box2_h);//Get CMD
	buf[0] = cyg_mbox_get(rx_box2_h);//Get NREG
	buf[1] = cyg_mbox_get(rx_box2_h);//Get nr
	buf[2] = cyg_mbox_get(rx_box2_h);//Get iread
	buf[3] = cyg_mbox_get(rx_box2_h);//Get iwrite
	temp = cyg_mbox_get(rx_box2_h);//Get EOM
	//print result
	cyg_mutex_lock(&clipmux);
	printf("NREG:%d nr:%d iread:%d iwrite:%d  \n", buf[0], buf[1], buf[2], buf[3]);
	cyg_mutex_unlock(&clipmux);

}
/*-------------------------------------------------------------------------+
| Function: cmd_trc - transfer n registers from current iread position
+--------------------------------------------------------------------------*/
void cmd_trc (int argc, char** argv)
{
	unsigned int n, nr;    
    unsigned char buf[20], temp, m;
	Register r;
	
	if(argc != 2)
	{
		cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
    
    nr = atoi(argv[1]);
    
	//Send SOM CMD ... EOM
	cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);    
	cyg_mbox_put(tx_box1_h, TRGC);
	cyg_mbox_put(tx_box1_h, nr);
	cyg_mbox_put(tx_box1_h, EOM);
	cyg_mutex_unlock(&transmit_mux);
	//wait for CMD_OK or CMD_ERROR
	temp = cyg_mbox_get(rx_box2_h);//Get SOM
	temp = cyg_mbox_get(rx_box2_h);//Get CMD
	m = cyg_mbox_get(rx_box2_h);//Get CMD_OK/CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get EOM
    
    if(m == CMD_OK)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Registers transfered!\n");
    	cyg_mutex_unlock(&clipmux);
    }
    if(m == CMD_ERROR)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Error tranfering registers (no registers to transfer?) !\n");
    	cyg_mutex_unlock(&clipmux);
    }
}
/*-------------------------------------------------------------------------+
| Function: cmd_tri -
+--------------------------------------------------------------------------*/
void cmd_tri (int argc, char** argv)
{
	unsigned int n, nr, index;    
    unsigned char buf[20], temp, m;
	Register r;
	
	if(argc != 3)
	{
		cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
    
    nr = atoi(argv[1]);
    index = atoi(argv[2]);
	//Send SOM CMD ... EOM
	cyg_mutex_lock(&transmit_mux);
    cyg_mbox_put(tx_box1_h, SOM);    
	cyg_mbox_put(tx_box1_h, TRGI);
	cyg_mbox_put(tx_box1_h, nr);
	cyg_mbox_put(tx_box1_h, index);
	cyg_mbox_put(tx_box1_h, EOM);
	cyg_mutex_unlock(&transmit_mux);
	//wait for CMD_OK or CMD_ERROR
	temp = cyg_mbox_get(rx_box2_h);//Get SOM
	temp = cyg_mbox_get(rx_box2_h);//Get CMD
	m = cyg_mbox_get(rx_box2_h);//Get CMD_OK/CMD_ERROR
    temp = cyg_mbox_get(rx_box2_h);//Get EOM
    
    if(m == CMD_OK)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Registers transfered!\n");
    	cyg_mutex_unlock(&clipmux);
    }
    if(m == CMD_ERROR)
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Error tranfering registers (no registers to transfer?) !\n");
    	cyg_mutex_unlock(&clipmux);
    }
	
	
}
/*-------------------------------------------------------------------------+
| Function: cmd_irl - information about local registers
+--------------------------------------------------------------------------*/
void cmd_irl (int argc, char** argv)
{
	uint8_t nr, ri, wi;
	if(argc != 1)
	{
		cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
    
    cyg_mutex_lock(&buf_mux);
    nr = Rb.nr;
    ri = Rb.ri;
    wi = Rb.wi; 
    cyg_mutex_unlock(&buf_mux);
    
    cyg_mutex_lock(&clipmux);
    printf("N:%d nr:%d ri:%d wi:%d\n", NRBUF, nr, ri, wi);
    cyg_mutex_unlock(&clipmux);
}
/*-------------------------------------------------------------------------+
| Function: cmd_lr - list n registers from index i
+--------------------------------------------------------------------------*/
void cmd_lr (int argc, char** argv)
{
	uint8_t n, i, nr, ri, wi;
	Register r;
	
	if(argc != 2 && argc != 3)
	{
		cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
    
    cyg_mutex_lock(&buf_mux);
    nr = Rb.nr;
    cyg_mutex_unlock(&buf_mux);
    
    if(argc == 2)
    {
    	n = atoi(argv[1]);
    	
    	while(n > 0)
    	{
    		cyg_mutex_lock(&buf_mux);
    		ri = Rb.ri;
    		wi = Rb.wi;
    		cyg_mutex_unlock(&buf_mux);
    		if(ri != wi)
    		{
    			cyg_mutex_lock(&buf_mux);
    			r = RingBuffer_ReadRegister();
    			cyg_mutex_unlock(&buf_mux);
			}
    		else
    			break;
    		cyg_mutex_lock(&clipmux);
    		printf("Register: %d %d %d %d %d\n", r.h, r.m, r.s, r.temp, r.lum);
    		cyg_mutex_unlock(&clipmux);
    		n--;
    	}
    }
    else if(argc == 3)
    {
    	n = atoi(argv[1]);
    	i = atoi(argv[2]);
    	if(i < nr)
    	{
    		if(nr < NRBUF)
    		{
    			n = nr - i;
			}
			while(n > 0)
			{
				cyg_mutex_lock(&buf_mux);
				r = RingBuffer_ReadRegisterFromIndex(i);
				cyg_mutex_unlock(&buf_mux);
				cyg_mutex_lock(&clipmux);
				printf("Register:%d %d %d %d %d\n", r.h, r.m, r.s, r.temp, r.lum);
				cyg_mutex_unlock(&clipmux);
				i++;
				if(i > NRBUF-1)
					i = 0;
				n--;
			}
		}
    }
}
/*-------------------------------------------------------------------------+
| Function: cmd_dr - delete local registers
+--------------------------------------------------------------------------*/
void cmd_dr (int argc, char** argv)
{
	if(argc != 1)
	{
		cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
	cyg_mutex_lock(&buf_mux);
	RingBuffer_DeleteRegisters();
	cyg_mutex_unlock(&buf_mux);
}
/*-------------------------------------------------------------------------+
| Function: cmd_cpt - check period of transference
+--------------------------------------------------------------------------*/
void cmd_cpt (int argc, char** argv)
{
	uint8_t temp;
	
	if(argc != 1)
	{
		cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
	
	cyg_mutex_lock(&transf_mux);
	temp = transf_p;
	cyg_mutex_unlock(&transf_mux);
	
	cyg_mutex_lock(&clipmux);
	printf("Period of transference is: %d\n", temp);
	cyg_mutex_unlock(&clipmux);
}
/*-------------------------------------------------------------------------+
| Function: cmd_mpt - modify period of transference
+--------------------------------------------------------------------------*/
void cmd_mpt (int argc, char** argv)
{
	uint8_t temp, p;
	cyg_handle_t alarm_temp;
	
	if(argc != 2)
	{
		cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
        
    p = atoi(argv[1]);
    p = p*60;
    if(p == 0)
    {
    	cyg_mutex_lock(&transf_mux);
    	transf_p = p;
    	transf_on = false;
    	alarm_temp = transf_alarm_h;
    	cyg_mutex_unlock(&transf_mux);
    	cyg_alarm_disable(alarm_temp);
    }
    else
    {
    	cyg_mutex_lock(&transf_mux);
    	transf_p = p;
    	transf_on = true;
    	alarm_temp = transf_alarm_h;
    	cyg_mutex_unlock(&transf_mux);
    	cyg_alarm_enable(alarm_temp);
    	cyg_alarm_initialize(alarm_temp, cyg_current_time()+SECONDSTOTICKS*p, SECONDSTOTICKS*p);	
    }
}
/*-------------------------------------------------------------------------+
| Function: cmd_dttl - define threshold temperature and luminosity(local)
+--------------------------------------------------------------------------*/
void cmd_dttl (int argc, char** argv)
{
	uint8_t temp, lum;
	if(argc != 3)
	{
		cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
    
    temp = atoi(argv[1]);
    lum = atoi(argv[2]);
    
    if( (temp < 0 || temp > 50) || (lum < 0 || lum > 3) )
    {
    	cyg_mutex_lock(&clipmux);
    	printf("Invalid temperature or luminosity threshold\n");
    	cyg_mutex_unlock(&clipmux);
    	return;
    }
    
    cyg_mutex_lock(&transf_mux);
    transf_tt = temp;
    transf_lt = lum;
    cyg_mutex_unlock(&transf_mux);
    
}
/*-------------------------------------------------------------------------+
| Function: cmd_pr - process registers between instants t1 and t2
+--------------------------------------------------------------------------*/
void cmd_pr (int argc, char** argv)
{
	uint8_t h1, h2 ,m1, m2, s1, s2, temp;
	uint8_t maxt, mint, meant, maxl, minl, meanl;
	if(argc != 7 && argc != 4 && argc != 1)
	{
		cyg_mutex_lock(&clipmux);
        printf("Invalid number of arguments, please type sos for help.\n");
        cyg_mutex_unlock(&clipmux);
    }
    //send request to processing thread
    
    if(argc == 7)
    {
    	h1 = atoi(argv[1]);
    	m1 = atoi(argv[2]);
    	s1 = atoi(argv[3]);
    	
    	h2 = atoi(argv[4]);
    	m2 = atoi(argv[5]);
    	s2 = atoi(argv[6]);
    	
    	cyg_mbox_put(pr_box2_h, SOM);
    	cyg_mbox_put(pr_box2_h, PRR);
    	cyg_mbox_put(pr_box2_h, h1);
    	cyg_mbox_put(pr_box2_h, m1);
    	cyg_mbox_put(pr_box2_h, s1);
    	cyg_mbox_put(pr_box2_h, h2);
    	cyg_mbox_put(pr_box2_h, m2);
    	cyg_mbox_put(pr_box2_h, s2);
    	cyg_mbox_put(pr_box2_h, EOM);
    }
    else if(argc == 4)
    {
    	h1 = atoi(argv[1]);
    	m1 = atoi(argv[2]);
    	s1 = atoi(argv[3]);
    
    	cyg_mbox_put(pr_box2_h, SOM);
    	cyg_mbox_put(pr_box2_h, PRR);
    	cyg_mbox_put(pr_box2_h, h1);
    	cyg_mbox_put(pr_box2_h, m1);
    	cyg_mbox_put(pr_box2_h, s1);
    	cyg_mbox_put(pr_box2_h, EOM);
    }
    else if(argc == 1)
    {
    	cyg_mbox_put(pr_box2_h, SOM);
    	cyg_mbox_put(pr_box2_h, PRR);
    	cyg_mbox_put(pr_box2_h, EOM);
    }
    //get results
    temp = cyg_mbox_get(pr_box1_h);//SOM
    temp = cyg_mbox_get(pr_box1_h);//CMD
    maxt = cyg_mbox_get(pr_box1_h);
    mint = cyg_mbox_get(pr_box1_h);
    meant = cyg_mbox_get(pr_box1_h);
    maxl = cyg_mbox_get(pr_box1_h);
    minl = cyg_mbox_get(pr_box1_h);
    meanl = cyg_mbox_get(pr_box1_h);
    temp = cyg_mbox_get(pr_box1_h);//EOM
    
    cyg_mutex_lock(&clipmux);
	printf("t: max %d min %d mean %d\nl: max %d min %d mean %d\n", maxt, mint, meant, maxl, minl, meanl);
	cyg_mutex_unlock(&clipmux);
}
/*-------------------------------------------------------------------------+
| Function: cmd_ini - inicializar dispositivo
+--------------------------------------------------------------------------*/ 
void cmd_ini(int argc, char **argv)
{
  printf("io_lookup\n");
  if ((argc > 1) && (argv[1][0] = '1'))
    err = cyg_io_lookup("/dev/ser1", &serH);
  else err = cyg_io_lookup("/dev/ser0", &serH);
  printf("lookup err=%x\n", err);
}
