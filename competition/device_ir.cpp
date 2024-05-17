#include "device_ir.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////      IR      //////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void IR::init()
{
    pinMode(PIN_IR_L, INPUT);
    pinMode(PIN_IR_M, INPUT);
    pinMode(PIN_IR_R, INPUT);
}

void IR::get_simple_motor_speed(uint16_t *speed_left_motor, uint16_t *speed_right_motor)
{
    int ir_value_left = analogRead(PIN_IR_L);
    int ir_value_middle = analogRead(PIN_IR_M);
    int ir_value_right = analogRead(PIN_IR_R);

    if (ir_value_left > 100)
    {
        speed_left_motor = 0;
        speed_right_motor = 100;
    }
    else if (ir_value_right > 100)
    {
        speed_left_motor = 100;
        speed_right_motor = 0;
    }
    else if (ir_value_middle > 100)
    {
        speed_left_motor = 100;
        speed_right_motor = 100;
    }
    else
    {
        speed_left_motor = 0;
        speed_right_motor = 0;
    }
}

void IR::test()
{
    int ir_value_left = analogRead(PIN_IR_L);
    int ir_value_middle = analogRead(PIN_IR_M);
    int ir_value_right = analogRead(PIN_IR_R);
    Serial.print("IR_L=");
    Serial.print(ir_value_left);
    Serial.print("| IR_M=");
    Serial.print(ir_value_middle);
    Serial.print("| IR_R=");
    Serial.println(ir_value_right);
}