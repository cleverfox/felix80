#include <stdint.h>

int _write(int file, char *ptr, int len);
void incout(int fd, uint32_t cn);
void acout(int fd, uint8_t cn);
void xcout(int fd, unsigned char c);
void cout(int fd, char c);
void x4cout(int fd, uint32_t c);
uint32_t b2i(void* a);
void i2b(uint32_t i, void* a);
uint16_t b2s(void* a);
void* memcpyx(uint8_t *dst, uint8_t * src, uint32_t len);
//void* memcpy32(uint32_t *dst, uint32_t* src, uint32_t len);

