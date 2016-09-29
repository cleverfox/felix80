#include "tools.h"

void out1(int fd, char c);
void out1(int fd, char c){
    char o[2]={c,0};
    _write(fd,o,1);
}
void incout(int fd, uint32_t cn){
    uint8_t out[10];
    int r=0;
    while(cn>0){
        out[r]=(cn%10)+'0';
        cn=cn/10;
        r++;
    }
    while(r){
        r--;
        out1(fd, out[r]);
    }
}


static char set[]="0123456789ABCDEF";
void acout(int fd, unsigned char c){
    if(c>=32 && c<=127){
        out1(fd, c);
    }else{
        _write(fd,"\\x",2),
        out1(fd, set[(c>>4)&0x0f]);
        out1(fd, set[c&0x0f]);
    }
}

void xcout(int fd,unsigned char c){
   out1(fd, set[(c>>4)&0x0f]);
   out1(fd, set[c&0x0f]);
}
void x4cout(int fd, uint32_t c){
   out1(fd, set[(c>>28)&0x0f]);
   out1(fd, set[(c>>24)&0x0f]);
   out1(fd, set[(c>>20)&0x0f]);
   out1(fd, set[(c>>16)&0x0f]);
   out1(fd, set[(c>>12)&0x0f]);
   out1(fd, set[(c>>8)&0x0f]);
   out1(fd, set[(c>>4)&0x0f]);
   out1(fd, set[c&0x0f]);
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
