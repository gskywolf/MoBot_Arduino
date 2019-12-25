#ifndef PIBOT_PID_H_
#define PIBOT_PID_H_

class PID{
    public:
        PID(float* input, float* feedback, float kp, float ki, float kd, unsigned short max_output);
        short compute(float interval);
        void clear();
        
        void update(float kp, float ki, float kd, unsigned short max_output);
    private:
        float kp;
        float ki;
        float kd;
        float max_output;
        float* input;
        float* feedback;

        float error;
        float integra;
        float derivative;

        float previous_error;
};

#endif