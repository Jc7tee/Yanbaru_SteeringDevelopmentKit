#ifndef SERIALTW_h
#define SERIALTW_h

#include "mbed.h"

#define _data_tx_size 4
#define _data_rx_size 2

class SerialTW
{
    public:
        SerialTW(RawSerial *pc) : _pc(pc){
        }
        bool read(char ch, int data_rx[]);
        void write(int data_tx[]);

        enum Mode_t{
            MODE_INIT = 0,
            MODE_STOP = 1,
            MODE_CTRL = 2
        };

        int mode = -1;
    private:
        RawSerial *_pc;
};

#endif /* SERIALTW_h */
