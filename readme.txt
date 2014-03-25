
////////////////////////////////////////////////
Stepping motor controller for 3D CNC by Kent-Na
////////////////////////////////////////////////


////////////
////Build & upload

This program require "ino" to build and upload.

--------
ino build
sudo ino upload
--------

////////////
////Protocol

[byte order]
This protocol uses network byte order(big-endian) thus you may need
to convert it.

[message]
Massage consists from message_type(1 byte) + message_contents(n byte)
where n is depend on type of massage.

////////////
////Massage flow

[move request] -> controller
host <- [request result]
host <- [request complete] //if request succeeded

////////////
////messages

[move request message]

message_type = 0x10
message_length = 8 byte

message_contents =
struct{
    int16_t dx;
    int16_t dy;
    int16_t dz;
    int16_t dt;
};

dx, dy and dz are step count of stepping motor on each axis.
Movement speeds is determined by dt parameter witch specifies 
total duration.
dt must be greater or equal to any absolute value of dx, dy or dz, 
or fail to execute request. In this case request result message with 
result value -1 will be sent from the controller.

The controller will responds with "request result" and "request complete" 
massage.

[request result message]

message_type = 0x20
message_length = 1 byte
message_contents = 
struct{
    int8_t result;
};

The "result" value is 0 when request are successfully accepted or -1(0xff) if
request parameter was illegal.

[request complete message]

message_type = 0x21
message_length = 0 byte
message_contents = 
struct{
};

This message will send from the controller when last request has been 
completed and ready to accept next request.

[bad message error message]

message_type = 0x30
message_length = 1 byte
message_contents = 
struct{
    uint8_t received_byte;
};

This message will send from the controller when received unknown message 
type.
