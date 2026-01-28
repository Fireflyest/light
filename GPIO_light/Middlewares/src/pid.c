# include "pid.h"


float PID_Update(PID_t* pid, float target, float measured, float dt) {
    float error = target - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}