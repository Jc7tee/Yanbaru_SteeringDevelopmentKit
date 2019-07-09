#ifndef SERIALTW_h
#define SERIALTW_h

#include "mbed.h"
#include "mode.h"

class SerialTW
{
    public:
        SerialTW(RawSerial *pc) : _pc(pc){
        }
        bool read(char ch, int data_rx[]);
        void write(int data_tx[]);
        int mode = -1;
    private:
        RawSerial *_pc;
};

#endif /* SERIALTW_h */
