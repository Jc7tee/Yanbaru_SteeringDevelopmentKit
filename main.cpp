#include "mbed.h" 

#include "lpf.h"
#include "mode.h"
#include "serial.h"
#include "ODriveMbed.h"


/* instance setting*/
DigitalOut led(LED3);
AnalogIn potentio_a(A0);
AnalogIn potentio_t(A1);
DigitalIn switch_cw(PA_5, PullUp);
DigitalIn switch_ccw(PA_6, PullUp);
RawSerial pc(USBTX, USBRX, 115200);
SerialTW stw(&pc);
Serial odrive_serial(PA_9, PA_10, 115200);
ODriveMbed odrive(odrive_serial);
LPF lpf_potentio_a(0.05);
LPF lpf_potentio_t(0.2);

Timer tm_main;
Ticker tk_update;
Ticker tk_serial;

/* macro setting */
#define ANGLE_OFFSET +0.0f
#define ANGLE_DIR -1.0f
#define CURRENT_LIMIT 70.0f
#define VELOCITY_LIMIT 40000.0f
#define VELOCITY_GAIN_P 0.4f

/* debug macro setting */
// #define USE_SELECTOR_SWITCH
#define USE_SERIAL_INTERFACE

/* grobal variable */
int _data_tx[_data_tx_size] = {0};
int _data_rx[_data_rx_size] = {0};
volatile bool _flagControl = false;
volatile bool _flagSerialTx = false;
volatile bool _flagSerialRx_action = false;


void IRQ_Control()
{
    _flagControl = true;
}

void IRQ_SerialTx()
{
    _flagSerialTx = true;
}

void IRQ_SerialRx()
{
    _flagSerialRx_action = true;
    led = !led;
    char ch = pc.getc();
    stw.read(ch, _data_rx);
}

float odrive_action(int action, int motornum, float target_value=0.0)
{
    int requested_state = 0;
    switch (action) {
        case ACTION_CALIBRATION:
            /* limit */
            odrive_serial.printf("w axis%d.controller.config.vel_limit %f\n",motornum, VELOCITY_LIMIT);
            odrive_serial.printf("w axis%d.motor.config.current_lim %f\n",motornum, CURRENT_LIMIT);

            /* calibration */
            requested_state = ODriveMbed::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
            pc.printf("Axis%d: Requesting State %d\n", motornum, requested_state);
            odrive.run_state(motornum, requested_state, false);
            // wait_ms(15000);
            pc.printf("Axis%d: Calibrated\n", motornum);
            break;
        case ACTION_IDLE:
            requested_state = ODriveMbed::AXIS_STATE_IDLE;
            pc.printf("Axis%d: Requesting State %d\n", motornum, requested_state);
            odrive.run_state(motornum, requested_state, false);
            break;
        case ACTION_CLOSEDLOOP:
            requested_state = ODriveMbed::AXIS_STATE_CLOSED_LOOP_CONTROL;
            pc.printf("Axis%d: Requesting State %d\n", motornum, requested_state);
            odrive.run_state(motornum, requested_state, false);
            break;
        case ACTION_POSITION_CTRL:
            odrive.setPosition(motornum, target_value);
            break;
        case ACTION_TRAJECTRY_CTRL:
            odrive.setTrajectory(motornum, target_value);
            break;
        case ACTION_VELOCITY_CTRL:
            odrive.setVelocity(motornum, target_value);
            break;
        case ACTION_GET_POSITION:
            odrive_serial.printf("r axis%d.encoder.pos_estimate\n", motornum);
            return odrive.readFloat();
            break;
        default:
            break;
    }
    return 0.0;
}


float minmax_lim(float value, float min=-1080.0, float max=1080.0)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

int main()
{
    tm_main.start();

    const float control_cycle_sec = 0.001;
    const float serialtx_cycle_sec = 0.02;
    tk_update.attach(&IRQ_Control, control_cycle_sec);
    tk_serial.attach(&IRQ_SerialTx, serialtx_cycle_sec);

    pc.attach(&IRQ_SerialRx, Serial::RxIrq);

    /* store variable */
    int time_us = 0;
    int time_us_diff = 0;
    int time_us_z1 = 0;
    int selector_switch_z1 = -1;
    int selector_switch = -1;
    float actual_encoder_pos = 0.0;
    float initial_steering_offset = 0.0;

    while(1) {
        if (_flagControl)
        {
            _flagControl = false;

            /* measure time */
            time_us = tm_main.read_us();
            time_us_diff = (time_us - time_us_z1)*0.2 + time_us_diff*0.8;

            /* sensors */
            float actual_angle_raw = minmax_lim(-1.0 * float(potentio_a.read_u16() - 32768) / 65536.0
                 * (360.0 * 10.0) * (52.0/20.0 * 70.0/18.0 * 27.0/270.0) + ANGLE_OFFSET);
            float actual_angle_lpf = lpf_potentio_a.pass(actual_angle_raw);
            float target_angle_lpf = 0.0;

#if defined(USE_SELECTOR_SWITCH)
            float target_angle_raw = minmax_lim(-1.0 * float(potentio_t.read_u16() - 32768) / 65536.0 * 360.0 * 6.0);
            target_angle_lpf = lpf_potentio_t.pass(target_angle_raw);

            /* calc mode based on switch */
            if (switch_cw and switch_ccw)
            {
                selector_switch = SELECTOR_MIDLE;
            }
            else if (!switch_ccw)
            {
                selector_switch = SELECTOR_CCW;
            }
            else if (!switch_cw)
            {
                selector_switch = SELECTOR_CW;
            }

            /* state */
            int motornum = 0;
            switch (selector_switch) {
                case SELECTOR_CCW:
                    if (selector_switch_z1 != SELECTOR_CCW)
                    {
                        odrive_action(ACTION_CALIBRATION, motornum);
                    }
                    break;
                case SELECTOR_MIDLE:
                    if (selector_switch_z1 == SELECTOR_CCW)
                    {
                        /* store initial steering offset */
                        initial_steering_offset = actual_angle_lpf;

                        odrive_action(ACTION_CLOSEDLOOP, motornum);
                    }
                    else if (selector_switch_z1 == SELECTOR_CW)
                    {
                        actual_encoder_pos = odrive_action(ACTION_GET_POSITION, motornum);
                        pc.printf("%f\n", actual_encoder_pos);

                        odrive_action(ACTION_IDLE, motornum);
                    }
                    break;
                case SELECTOR_CW:
                    if (selector_switch_z1 != SELECTOR_CW)
                    {
                        odrive_action(ACTION_CLOSEDLOOP, motornum);
                    }
                    odrive_action(ACTION_VELOCITY_CTRL, motornum, 
                        ANGLE_DIR * (target_angle_lpf - actual_angle_lpf) * 8192.0/360.0 * 270.0/27.0 * VELOCITY_GAIN_P);
                    break;
                default:
                    break;
            }
#elif defined(USE_SERIAL_INTERFACE)
            int motornum = 0;
            if (stw.mode == ACTION_VELOCITY_CTRL)
            {
                target_angle_lpf = minmax_lim(float(_data_rx[0]) / 1000.0);
                odrive_action(ACTION_VELOCITY_CTRL, motornum, 
                    ANGLE_DIR * (target_angle_lpf - actual_angle_lpf) * 8192.0/360.0 * 270.0/27.0 * VELOCITY_GAIN_P);
            }
            else if (_flagSerialRx_action)
            {
                odrive_action(stw.mode, motornum);
                _flagSerialRx_action = false;
            }
#endif
            /* serial sending */
            if (_flagSerialTx)
            {
                _flagSerialTx = false;
                _data_tx[0] = stw.mode;
                _data_tx[1] = actual_angle_lpf;
                _data_tx[2] = target_angle_lpf;
                _data_tx[3] = selector_switch;
                _data_tx[4] = int(actual_encoder_pos);
                _data_tx[5] = potentio_a.read_u16() - 32768;
                stw.write(_data_tx);
            }

            /* z1 */
            time_us_z1 = time_us;
            selector_switch_z1 = selector_switch;
        }
    }
}
