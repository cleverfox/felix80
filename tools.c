#include "tools.h"

void out1(char c);
void out1(char c){
    char o[2]={c,0};
    _write(0,o,1);
}
void incout(uint32_t cn){
    if(cn>999999)
        out1((cn/1000000)%10+'0');
    if(cn>99999)
        out1((cn/100000)%10+'0');
    if(cn>9999)
        out1((cn/10000)%10+'0');
    if(cn>999)
        out1((cn/1000)%10+'0');
    if(cn>99)
        out1((cn/100)%10+'0');
    out1((cn/10)%10+'0');
    out1(cn%10+'0');
}

void ilncout(unsigned int cn){
    out1('\r');
    out1('\n');
    incout(cn);
}

void icout(uint32_t cn){
    incout(cn);
    out1('\r');
    out1('\n');
}


static char set[]="0123456789ABCDEF";
void cout(char c){
   out1(c);
}
void acout(unsigned char c){
    if(c>=32 && c<=127){
        out1(c);
    }else{
        _write(1,"\\x",2),
        out1(set[(c>>4)&0x0f]);
        out1(set[c&0x0f]);
    }
}

void xcout(unsigned char c){
   out1(set[(c>>4)&0x0f]);
   out1(set[c&0x0f]);
}
void x4cout(uint32_t c){
   out1(set[(c>>28)&0x0f]);
   out1(set[(c>>24)&0x0f]);
   out1(set[(c>>20)&0x0f]);
   out1(set[(c>>16)&0x0f]);
   out1(set[(c>>12)&0x0f]);
   out1(set[(c>>8)&0x0f]);
   out1(set[(c>>4)&0x0f]);
   out1(set[c&0x0f]);
}

uint32_t b2i(void* a){
    uint8_t *ba=a;
    return (ba[0]<<24) |
        (ba[1] << 16) |
        (ba[2] << 8) |
        ba[3];
}
void i2b(uint32_t i, void* a){
    uint8_t *ba=a;
    ba[0]=0xff&(i>>24);
    ba[1]=0xff&(i>>16);
    ba[2]=0xff&(i>>8);
    ba[3]=0xff&(i);
}
uint16_t b2s(void* a){
    uint8_t *ba=a;
    return  (ba[0] << 8) | ba[1];
}

void* memcpy(uint8_t *dst0, uint8_t * src, uint32_t len){
    uint8_t *dst=dst0;
    while(len--)
        *(dst++)=*(src++);
    return dst0;
}

/*
void memcpy32(uint32_t *dst, uint32_t* src, uint32_t len){
    while(len--)
        *(dst++)=*(src++);
}
*/
