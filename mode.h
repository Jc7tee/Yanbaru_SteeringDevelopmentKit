#ifndef MODE_h
#define MODE_h

#define _data_tx_size 10
#define _data_rx_size 2

enum Seletctor_t{
    SELECTOR_CW = 0,
    SELECTOR_MIDLE = 1,
    SELECTOR_CCW = 2
};

enum Action_t{
    ACTION_CALIBRATION = 0,
    ACTION_IDLE = 1,
    ACTION_CLOSEDLOOP = 2,
    ACTION_POSITION_CTRL = 3,
    ACTION_TRAJECTRY_CTRL = 4,
    ACTION_VELOCITY_CTRL = 5,
    ACTION_GET_POSITION = 6
};

#endif /* MODE_h */
