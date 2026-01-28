#ifndef __PID_H
#define __PID_H

typedef struct {
    float kp, ki, kd;
    float integral;
    float last_error;
} PID_t;

float PID_Update(PID_t* pid, float target, float measured, float dt);

#endif /* __PID_H */