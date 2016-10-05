#ifndef FELIX_H
#define FELIX_H

#define ACTION_PRINT 1
#define ACTION_FEED 2


struct print_str {
    uint8_t repeat_lines;
    uint8_t action;
    char str[96];
};


void _fault(int, int, const char*);
#define fault(code) _fault(code,__LINE__,__FUNCTION__)

#endif
