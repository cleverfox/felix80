#ifndef SERIAL_H
#define SERIAL_H
#include <atomqueue.h>


int init_serial_thread(void);

extern ATOM_QUEUE uart3_rx;
extern ATOM_QUEUE uart3_tx;

#define S_ENQ 0x05
#define S_ACK 0x06
#define S_STX 0x02
#define S_ETX 0x03
#define S_EOT 0x04
#define S_NAK 0x15
#define S_DLE 0x10

#endif
