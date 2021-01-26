#ifndef DEFS_H 
#define DEFS_H

#include <cyg/io/io.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/kernel/kapi.h>
#include "ringbuf.h"

typedef unsigned char uint8_t;

#define SOM   0xFD  /* start of message */
#define EOM   0xFE  /* end of message */
#define RCLK  0xC0  /* read clock */
#define SCLK  0XC1  /* set clock */
#define RTL   0XC2  /* read temperature and luminosity */
#define RPAR  0XC3  /* read parameters */
#define MMP   0XC4  /* modify monitoring period */
#define MTA   0XC5  /* modify time alarm */
#define RALA  0XC6  /* read alarms (temperature, luminosity, active/inactive) */
#define DATL  0XC7  /* define alarm temperature and luminosity */
#define AALA  0XC8  /* activate/deactivate alarms */
#define RMM   0XC9  /* read maximums and minimus registers */
#define CMM   0XCA  /* clear maximums and minimus registers */
#define IREG  0XCB  /* information about registers (NREG, nr, iread, iwrite)*/
#define TRGC  0XCC  /* transfer registers (curr. position)*/
#define TRGI  0XCD  /* transfer registers (index) */
#define NMFL  0XCE  /* notification memory (half) full */
#define PRR	  0XCF  /*process registers (h1 m1 s1, h2 m2 s2)*/
#define TRA	  0XD0  /*transference alarm*/
#define CMD_OK    0     /* command successful */
#define CMD_ERROR 0xFF  /* error in command */

#define THREAD_STACK_SIZE CYGNUM_HAL_STACK_SIZE_TYPICAL
#define RX_PRIORITY 8
#define TX_PRIORITY 9
#define PR_PRIORITY 7
#define NRBUF 100
#define SECONDSTOTICKS 100

cyg_io_handle_t serH;
//threads
static unsigned char tx_stack[THREAD_STACK_SIZE];
static unsigned char rx_stack[THREAD_STACK_SIZE];
static unsigned char pr_stack[THREAD_STACK_SIZE];
static cyg_handle_t tx_handle, rx_handle, pr_handle;
static cyg_thread tx_thread, rx_thread, pr_thread;
//mboxes
cyg_handle_t tx_box1_h, rx_box1_h, rx_box2_h, pr_box1_h, pr_box2_h;
cyg_mbox tx_box1, rx_box1, rx_box2, pr_box1, pr_box2;
//time variables
cyg_handle_t system_clock_h, counter_h, transf_alarm_h;
cyg_alarm transf_alarm;
//synch variables
cyg_mutex_t clipmux;
cyg_mutex_t transmit_mux;
cyg_mutex_t transf_mux;
cyg_mutex_t buf_mux;
//transference global variables
uint8_t transf_p, transf_tt, transf_lt;
bool transf_on;
bool do_transf;
#endif
