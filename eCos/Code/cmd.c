#include <stdio.h>
#include "ringbuf.h"

extern void cmd_ini (int, char** );
extern void monitor(void);

bool h1bigh2(uint8_t h1, uint8_t m1, uint8_t s1, uint8_t h2, uint8_t m2, uint8_t s2)
{
	if(h1 > h2)
		return true;
	else if(h1 < h2)
		return false;
	else
	{
		if(m1 > m2)
			return true;
		else if(m1 < m2)
			return false;
		else
		{
			if(s1 > s2)
				return true;
			else if(s1 < s2)
				return false;
			else
			{
				return true;
			}
		}
	}
}

void alarm_proc(cyg_handle_t alarmH, cyg_addrword_t data)
{
	if(transf_on)
	{
        do_transf = true;
        cyg_mbox_put(pr_box2_h, TRA);
	}
}

void pr_procedure(cyg_addrword_t data)
{
	uint8_t m, temp, cmd, tt, lt, n, i, nr;
	uint8_t maxt = 0x00, mint = 0xFF, meant = 0;
	uint8_t maxl = 0x00, minl = 0xFF, meanl = 0;
	float mean_ft = 0.0f, mean_fl = 0.0f;
	unsigned char buf[10];
	Register r;
	
	//init transference variables
  	transf_p = 0;
  	transf_on = false;
  	do_transf = false;
  	transf_tt = 30;
  	transf_lt = 2;
	//create alarm
	system_clock_h = cyg_real_time_clock();
	cyg_clock_to_counter(system_clock_h, &counter_h);
	cyg_alarm_create(counter_h, alarm_proc, 0, &transf_alarm_h, &transf_alarm);
	cyg_alarm_initialize(transf_alarm_h, cyg_current_time()+SECONDSTOTICKS*transf_p, SECONDSTOTICKS*transf_p);
	
	while(1)
	{
		m = cyg_mbox_get(pr_box2_h);
		
		if(m == TRA)
		{
			//ask for register transference
			cyg_mutex_lock(&transmit_mux);
			cyg_mbox_put(tx_box1_h, SOM);
			cyg_mbox_put(tx_box1_h, TRGC);
			cyg_mbox_put(tx_box1_h, 9);
			cyg_mbox_put(tx_box1_h, EOM);
			cyg_mutex_unlock(&transmit_mux);
		}
		else if(m == CMD_OK)
		{
			cyg_mutex_lock(&transf_mux);
			tt = transf_tt;
			lt = transf_lt;
			cyg_mutex_unlock(&transf_mux);
			
			cyg_mutex_lock(&clipmux);
			printf("Registers over thresholds.\n\n");
			cyg_mutex_unlock(&clipmux);
			
			cyg_mutex_lock(&buf_mux);
			
			while(Rb.ri != Rb.wi)
			{
				r = RingBuffer_ReadRegister();
				if(r.temp > tt || r.lum > lt)
				{
					cyg_mutex_lock(&clipmux);
					printf("Register: %d %d %d %d %d\n", r.h, r.m, r.s, r.temp, r.lum);
					cyg_mutex_unlock(&clipmux);
				}
			}
			
			cyg_mutex_unlock(&buf_mux);
			
		}
		else if(m == CMD_ERROR)
		{
			cyg_mutex_lock(&clipmux);
			printf("\nNo registers to transfer!\n");
			cyg_mutex_unlock(&clipmux);
		}
		else if(m == PRR)
		{
			n = 0;
			while(1)
			{
				m = cyg_mbox_get(pr_box2_h);
				
				if(m == EOM)
				{
					break;
				}
				else if(m == TRA)
				{
					//ask for register transference
					cyg_mutex_lock(&transmit_mux);
					cyg_mbox_put(tx_box1_h, SOM);
					cyg_mbox_put(tx_box1_h, TRGC);
					cyg_mbox_put(tx_box1_h, 9);
					cyg_mbox_put(tx_box1_h, EOM);
					cyg_mutex_unlock(&transmit_mux);
					continue;
				}
				else
				{
					buf[n] = m;
					n++;
				}
			}
			
			temp = 0;
			maxt = 0x00;
			mint = 0xFF;
			meant = 0;
			mean_ft = 0.0f;
			maxl = 0x00;
			minl = 0xFF;
			meanl = 0;
			mean_fl = 0.0f;
			
			cyg_mutex_lock(&buf_mux);
			nr = Rb.nr;
			cyg_mutex_unlock(&buf_mux);
			
			if(n == 6)
			{
				for(i = 0; i < nr; i++)
				{
					cyg_mutex_lock(&buf_mux);
					r = Rb.buffer[i];
					cyg_mutex_unlock(&buf_mux);
					
					if(h1bigh2(r.h, r.m, r.s, buf[0], buf[1], buf[2]))
					{
						if(h1bigh2(buf[3], buf[4], buf[5], r.h, r.m, r.s))
						{
							temp++;
						}
					}
					
				}
				for(i = 0; i < nr; i++)
				{
					cyg_mutex_lock(&buf_mux);
					r = Rb.buffer[i];
					cyg_mutex_unlock(&buf_mux);
					
					if(h1bigh2(r.h, r.m, r.s, buf[0], buf[1], buf[2]))
					{
						if(h1bigh2(buf[3], buf[4], buf[5], r.h, r.m, r.s))
						{
							if(r.temp > maxt)
								maxt = r.temp;
							if(r.temp < mint)
								mint = r.temp;
						
							if(r.lum > maxl)
								maxl = r.lum;
							if(r.lum < minl)
								minl = r.lum;
						
							mean_ft += r.temp/(float)(nr);
							mean_fl += r.lum/(float)(nr);
						}
					}
					
				}
				meant = (int)(mean_ft);
				meanl = (int)(mean_fl);
				//send result to ui thread
				cyg_mbox_put(pr_box1_h, SOM);
				cyg_mbox_put(pr_box1_h, PRR);
				cyg_mbox_put(pr_box1_h, maxt);
				cyg_mbox_put(pr_box1_h, mint);
				cyg_mbox_put(pr_box1_h, meant);
				cyg_mbox_put(pr_box1_h, maxl);
				cyg_mbox_put(pr_box1_h, minl);
				cyg_mbox_put(pr_box1_h, meanl);
				cyg_mbox_put(pr_box1_h, EOM);
			}
			else if(n == 3)
			{
				for(i = 0; i < nr; i++)
				{
					cyg_mutex_lock(&buf_mux);
					r = Rb.buffer[i];
					cyg_mutex_unlock(&buf_mux);
					
					if(h1bigh2(r.h, r.m, r.s, buf[0], buf[1], buf[2]))
					{
						temp++;
					}
					
				}
				for(i = 0; i < nr; i++)
				{
					cyg_mutex_lock(&buf_mux);
					r = Rb.buffer[i];
					cyg_mutex_unlock(&buf_mux);
					
					if(h1bigh2(r.h, r.m, r.s, buf[0], buf[1], buf[2]))
					{
						if(r.temp > maxt)
							maxt = r.temp;
						if(r.temp < mint)
							mint = r.temp;
					
						if(r.lum > maxl)
							maxl = r.lum;
						if(r.lum < minl)
							minl = r.lum;
					
						mean_ft += r.temp/(float)(nr);
						mean_fl += r.lum/(float)(nr);
					}
					
				}
				meant = (int)(mean_ft);
				meanl = (int)(mean_fl);
				//send result to ui thread
				cyg_mbox_put(pr_box1_h, SOM);
				cyg_mbox_put(pr_box1_h, PRR);
				cyg_mbox_put(pr_box1_h, maxt);
				cyg_mbox_put(pr_box1_h, mint);
				cyg_mbox_put(pr_box1_h, meant);
				cyg_mbox_put(pr_box1_h, maxl);
				cyg_mbox_put(pr_box1_h, minl);
				cyg_mbox_put(pr_box1_h, meanl);
				cyg_mbox_put(pr_box1_h, EOM);
			}
			else
			{
				for(i = 0; i < nr; i++)
				{
					cyg_mutex_lock(&buf_mux);
					r = Rb.buffer[i];
					cyg_mutex_unlock(&buf_mux);
					
					if(r.temp > maxt)
						maxt = r.temp;
					if(r.temp < mint)
						mint = r.temp;
						
					if(r.lum > maxl)
						maxl = r.lum;
					if(r.lum < minl)
						minl = r.lum;
						
					mean_ft += r.temp/(float)(nr);
					mean_fl += r.lum/(float)(nr);
				}
				meant = (int)(mean_ft);
				meanl = (int)(mean_fl);
				//send result to ui thread
				cyg_mbox_put(pr_box1_h, SOM);
				cyg_mbox_put(pr_box1_h, PRR);
				cyg_mbox_put(pr_box1_h, maxt);
				cyg_mbox_put(pr_box1_h, mint);
				cyg_mbox_put(pr_box1_h, meant);
				cyg_mbox_put(pr_box1_h, maxl);
				cyg_mbox_put(pr_box1_h, minl);
				cyg_mbox_put(pr_box1_h, meanl);
				cyg_mbox_put(pr_box1_h, EOM);
			}
		}	
	}
}

void tx_procedure(cyg_addrword_t data)
{
    uint8_t m;
	unsigned int n;    
    unsigned char buf[20];
    Cyg_ErrNo err;
    
    while(1)
    { 	
		m = cyg_mbox_get(tx_box1_h);
		n = 1;
		buf[0] = m;
		err = cyg_io_write(serH, buf, &n); 	
    }
    return;
}

void rx_procedure(cyg_addrword_t data)
{
    uint8_t transf_temp, cmd;
    unsigned int n, cmd_count, i;    
    unsigned char buf[100];
    Register r;
    Cyg_ErrNo err;
    cyg_handle_t alarm_temp;
    
    while(1)
    {	
		n = 1;
		err = cyg_io_read(serH, buf, &n);
		cmd_count = 1;
		while(buf[cmd_count-1] != EOM)
		{
			n = 1;
			err = cyg_io_read(serH, buf+cmd_count, &n);
			cmd_count++;
		}
		
		cmd = buf[1];
		
		if(cmd == NMFL)
		{
			cyg_mutex_lock(&clipmux);
			printf("\nNMHF: %d %d %d %d\n", buf[2], buf[3], buf[4], buf[5]);
			cyg_mutex_unlock(&clipmux);
			//block this
			cyg_mutex_lock(&transf_mux);
			transf_temp = transf_on;
			cyg_mutex_unlock(&transf_mux);
			if(!transf_temp)
			{
				cyg_mutex_lock(&transf_mux);
				transf_p = 60;
				transf_on = true;
				alarm_temp = transf_alarm_h;
				cyg_mutex_unlock(&transf_mux);
				cyg_alarm_enable(alarm_temp);
				cyg_alarm_initialize(alarm_temp, cyg_current_time()+SECONDSTOTICKS*60, SECONDSTOTICKS*60);
			}
			//
		}
		else if(cmd == TRGC || cmd == TRGI)
		{
			n = (cmd_count-2)/5;
			
			for(i=0; i < n; i++)
			{
				r.h = buf[2 + 5*i];
				r.m = buf[2 + 5*i + 1];
				r.s = buf[2 + 5*i + 2];
				r.temp = buf[2 + 5*i + 3];
				r.lum = buf[2 + 5*i + 4];
				
				RingBuffer_WriteRegister(r);
			}
			//inform process thread
			cyg_mbox_put(pr_box2_h, SOM);
			cyg_mbox_put(pr_box2_h, cmd);
			if(n > 0)
				cyg_mbox_put(pr_box2_h, CMD_OK);
			else
				cyg_mbox_put(pr_box2_h, CMD_ERROR);
			cyg_mbox_put(pr_box2_h, EOM);
			
			cyg_mutex_lock(&transf_mux);
			transf_temp = do_transf;
			cyg_mutex_unlock(&transf_mux);
			
			if(!transf_temp)
			{
				//inform ui thread
				cyg_mbox_put(rx_box2_h, SOM);
				cyg_mbox_put(rx_box2_h, cmd);
				if(n > 0)
					cyg_mbox_put(rx_box2_h, CMD_OK);
				else
					cyg_mbox_put(rx_box2_h, CMD_ERROR);
				cyg_mbox_put(rx_box2_h, EOM);
			}
			else
			{
				cyg_mutex_lock(&transf_mux);
				do_transf = 0;
				cyg_mutex_unlock(&transf_mux);
			}	
		}
		else
		{
			for(i=0; i < cmd_count; i++)
			{
				cyg_mbox_put(rx_box2_h, buf[i]);
			}
		}
		
    }
    return;
}

int main(void)
{
  cmd_ini(0, NULL);

  //init Ring Buffer
  Rb.nr = 0;
  Rb.ri = 0;
  Rb.wi = 0;
  Rb.empty = true; 
  //tx create
  cyg_thread_create(TX_PRIORITY, tx_procedure, 0, "tx", tx_stack
                    ,THREAD_STACK_SIZE, &tx_handle, &tx_thread);  
  //rx create
  cyg_thread_create(RX_PRIORITY, rx_procedure, 0, "rx", rx_stack
                    ,THREAD_STACK_SIZE, &rx_handle, &rx_thread);
  //pr create
  cyg_thread_create(PR_PRIORITY, pr_procedure, 0, "pr", pr_stack
                    ,THREAD_STACK_SIZE, &pr_handle, &pr_thread);
  //mboxes
  cyg_mbox_create(&tx_box1_h, &tx_box1);
  cyg_mbox_create(&rx_box1_h, &rx_box1);
  cyg_mbox_create(&rx_box2_h, &rx_box2);
  cyg_mbox_create(&pr_box1_h, &pr_box1);
  cyg_mbox_create(&pr_box2_h, &pr_box2);
  //synch variables
  cyg_mutex_init(&clipmux);
  cyg_mutex_init(&transmit_mux);
  cyg_mutex_init(&transf_mux);
  cyg_mutex_init(&buf_mux);
  //wake up threads
  cyg_thread_resume(tx_handle);
  cyg_thread_resume(rx_handle);
  cyg_thread_resume(pr_handle);
  
  monitor();
  
  //free synch variables
  cyg_mutex_destroy(&buf_mux);
  cyg_mutex_destroy(&transf_mux);
  cyg_mutex_destroy(&transmit_mux);
  cyg_mutex_destroy(&clipmux);
  //free mboxes
  cyg_mbox_delete(tx_box1_h);
  cyg_mbox_delete(rx_box1_h);
  cyg_mbox_delete(rx_box2_h);
  cyg_mbox_delete(pr_box1_h);
  cyg_mbox_delete(pr_box2_h);
  
  //free pr thread
  cyg_thread_kill(pr_handle);
  cyg_thread_delete(pr_handle);
  //free rx thread
  cyg_thread_kill(rx_handle);
  cyg_thread_delete(rx_handle);
  //free tx thread
  cyg_thread_kill(tx_handle);
  cyg_thread_delete(tx_handle);
  
  return 0;
}
