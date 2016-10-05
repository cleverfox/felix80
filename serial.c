#include <atom.h>
#include <atomtimer.h>
#include <libopencm3/stm32/usart.h>
#include "serial.h"
#include "felix.h"
#include "tools.h"

#define UART_QLEN 64
#define STACK_SIZE      1024
uint8_t uart3_rx_storage[UART_QLEN];
uint8_t uart3_tx_storage[UART_QLEN];
static uint8_t serial_stack[STACK_SIZE];
static ATOM_TCB serial_thread_tcb;
static void serial_thread(uint32_t args __maybe_unused);
ATOM_QUEUE uart3_rx;
ATOM_QUEUE uart3_tx;
int usends(char *ptr, int len);
void usend(char p);


int init_serial_thread(void){
	if (atomQueueCreate (&uart3_rx, uart3_rx_storage, sizeof(uint8_t), UART_QLEN) != ATOM_OK)
		return -1;
	if (atomQueueCreate (&uart3_tx, uart3_tx_storage, sizeof(uint8_t), UART_QLEN) != ATOM_OK)
		return -1;

	if(atomThreadCreate(&serial_thread_tcb, 5, serial_thread, 0,
			serial_stack, STACK_SIZE, TRUE) != ATOM_OK)
		return -1;
	return 0;
}

void handle_data(uint8_t *buf, int len){
	int i=0;
	_write(0,"\r\n",2);
	for(;i<len;i++){
		xcout(0,buf[i]);
	}
	_write(0,"\r\n",2);
}

#define RBLEN 128
static void serial_thread(uint32_t args __maybe_unused) {
	uint8_t ch;
	uint8_t recvbuf[RBLEN];
	uint8_t escape=0;
	uint8_t last=0;
	uint8_t crc=0;
	uint8_t status;
	int rbptr;
	enum {idle, recv, fin} state=idle;
	while(1){
	    switch(state){
	    case idle:
	    	_write(0,"idle\r\n",6);
	    	status = atomQueueGet(&uart3_rx, 0, (void*)&ch);
	    	if(status == ATOM_OK){
	    		if(ch==S_ENQ){
	    			usend(S_ACK);
	    			state=recv;
	    			rbptr=0;
	    			escape=0;
	    			last=0;
	    			crc=0;
	    			//_write(0,"recv\r\n",6);
	    		}
	    	}else
	    		fault(7);
	    	break;
	    case recv:
	    	status = atomQueueGet(&uart3_rx, SYSTEM_TICKS_PER_SEC>>1, (void*)&ch);
	    	if(status == ATOM_OK){
				if(rbptr>0)
					crc^=ch;
	    		if(!escape && ch==S_DLE)
	    			escape=1;
	    		else{
	    			if(last){
	    				//check crc
	    				if(crc==0){
	    					usend(S_ACK);
	    					//_write(0,"crc ok\r\n",8);
	    					state=idle;
	    					handle_data(recvbuf,rbptr);
	    				}else{
	    					usend(S_NAK);
	    					_write(0,"crc bad\r\n",9);
	    					state=idle;
	    				}
	    			}else{
	    				if(ch==S_ETX){
	    					last=1;
	    				}
	    				recvbuf[rbptr]=ch;
	    				rbptr++;

	    				if(rbptr>=RBLEN){ //buffer overflow
	    					usend(S_NAK);
	    					state=idle;
	    				}
	    			}
	    		}
	    	}else if(status == ATOM_TIMEOUT){
	    		state=idle;
	    	}else
	    		fault(7);
	    	break;
	    case fin:
	    	status = atomQueueGet(&uart3_rx, SYSTEM_TICKS_PER_SEC>>1, (void*)&ch);
	    	state=idle;
	    }
//			_write(0,"s",1);
	}



}

int usends(char *ptr, int len) {
	int i;
	for (i = 0; i < len; i++){
		atomQueuePut(&uart3_tx,0, (uint8_t*) &ptr[i]);
	}
	USART_CR1(USART3) |= USART_CR1_TXEIE;
	return i;
}
void usend(char p) {
	atomQueuePut(&uart3_tx,0, (uint8_t*) &p);
	USART_CR1(USART3) |= USART_CR1_TXEIE;
}


