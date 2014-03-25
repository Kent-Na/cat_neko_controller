/////////////////////
//Utility

//Assert output on serial io is just bad idea. It messed up all protocol
//state.

/*
#define assert(cond) if (!(cond)){\
    Serial.print("assert fail(");\
    Serial.print(__LINE__);\
    Serial.print("):"#cond);\
    }
    */

#define assert(cond)

/////////////////////
//constants

//You can customize those.

//Unit of time.
const int dt_unit = 10;
const int serial_data_rate = 9600;

//Pattern count must be 2^n where n is natural number.
//Bellow 4 bit are used.
byte pattern[4] = {
    B00000001,
    B00000100,
    B00000010,
    B00001000,
};

int pin_map[3][4] = {
    {4,   5,  6,  7}, //x-axis
    {8,   9, 10, 11}, //y-axis
    {A0, A1, A2, A3}, //z-axis
};


/////////////////////
//Master state

uint8_t state;
const uint8_t state_wait_next_message = 0x00;
const uint8_t state_wait_compleate_request = 0x01;

/////////////////////
//Message io prototypes

void receive_message();
void receive_message_type();
void receive_request_move_message();

void send_request_result(int8_t result_code);
void send_request_complete();
void send_error_message(uint8_t error_code);

//////////////////////////
//Motor control
//X, y and z axis steppnig motor state
typedef struct {
    uint8_t phase;

    //Bresenham algo parameters
    int delta_phase_2;
    int delta_t_2;
    int error;

    //-1 or +1.
    int move_direction;

} Axis_state;

//Remaining time step.
int remain_t;

Axis_state axis_state[3];

void init_all();
void init_axis(uint8_t axis);

void update_all();
void set_axis_output(uint8_t axis);
void update_axis(uint8_t axis);

void init_all(){
    remain_t = 0;
    init_axis(0);
    init_axis(1);
    init_axis(2);
}
void init_axis(uint8_t axis){
    axis_state[axis].phase = 0;
}

void set_axis_output(uint8_t axis){
    uint8_t phase = axis_state[axis].phase & B11;
    for (int i = 0; i<4; i++){
        digitalWrite(
            pin_map[axis][i], 
            (pattern[phase] & (1<<i)) ? HIGH : LOW);
    }
}

void update_all(){
    update_axis(0);
    update_axis(1);
    update_axis(2);
    remain_t --;
    if (remain_t == 0){
        send_request_complete();
        for (int i= 0; i<3; i++){
            for (int j= 0; j<3; j++){
                digitalWrite(pin_map[j][i], LOW);
            }
        }
        state = state_wait_next_message;
    }
}

void update_axis(uint8_t axis){
    const int dir = axis_state[axis].move_direction;

    axis_state[axis].error += axis_state[axis].delta_phase_2 * dir;
    if (axis_state[axis].error >= 0){
        axis_state[axis].phase += dir;
        set_axis_output(axis);
        axis_state[axis].error -= axis_state[axis].delta_t_2;
    }
}

//must be called from request_move
void request_axis_move(uint8_t axis, int16_t d, int16_t dt){
    assert(dt > 0);
    assert(d >= dt);

    axis_state[axis].delta_t_2 = dt*2;
    axis_state[axis].delta_phase_2 = d*2;
    axis_state[axis].error = -dt;
    if (d > 0)
        axis_state[axis].move_direction = +1;
    else
        axis_state[axis].move_direction = -1;
}

void request_move(int16_t dx, int16_t dy, int16_t dz, int16_t dt){
    remain_t = dt;

    request_axis_move(0, dx, dt);
    request_axis_move(1, dy, dt);
    request_axis_move(2, dz, dt);
}

////////////////////////
//Message io

//Arduino have 128 byte internal serial buffer. This means about 13 request
//can be stored in there.

//Currently receiving message type.
uint8_t message_type;

const uint8_t null_message = 0x00;
const uint8_t request_move_message = 0x10;
const uint8_t request_result_message = 0x20;
const uint8_t request_complete_message = 0x21;
const uint8_t error_message = 0x30;

const int8_t result_fail = -1;
const int8_t result_success = 0;


void receive_message(){
    if (message_type == null_message){
        receive_message_type();
    }
    else if (message_type == request_move_message){
        receive_request_move_message();
    }
    else{
        send_error_message(message_type);
    }
}
void receive_message_type(){
    if (Serial.available()>0){
        message_type = Serial.read();
    }
}

void receive_request_move_message(){
    if (Serial.available()<8){
        return;
    }

    int16_t dx  = Serial.read()<<8;
            dx |= Serial.read();
    int16_t dy  = Serial.read()<<8;
            dy |= Serial.read();
    int16_t dz  = Serial.read()<<8;
            dz |= Serial.read();
    int16_t dt  = Serial.read()<<8;
            dt |= Serial.read();

    if (dt < 1 || abs(dx) > dt || abs(dy) > dt || abs(dz) > dt){
        send_request_result(result_fail);
        message_type = null_message;
        return;
    }

    send_request_result(result_success);
    message_type = null_message;
    request_move(dx, dy, dz, dt);
    state = state_wait_compleate_request;
}

void send_request_result(int8_t result_code){
    Serial.write(request_result_message);
    Serial.write(result_code);
}

void send_request_complete(){
    Serial.write(request_complete_message);
}

void send_error_message(uint8_t error_code){
    Serial.write(error_message);
    Serial.write(error_code);
}

void setup(){
    Serial.begin(serial_data_rate);
    state = state_wait_next_message;
    message_type = null_message;

    for (int i= 0; i<4; i++){
        for (int j= 0; j<3; j++){
            pinMode(pin_map[j][i], OUTPUT);
        }
    }

    for (int i= 0; i<4; i++){
        for (int j= 0; j<3; j++){
            digitalWrite(pin_map[j][i], LOW);
        }
    }
}

void loop(){
    if (state == state_wait_compleate_request){
        update_all();
    }
    if (state == state_wait_next_message){
        receive_message();
    }
    delay(dt_unit);
}
