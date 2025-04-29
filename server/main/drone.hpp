#pragma once

#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensors.hpp"
#include "motors.hpp"

constexpr int MOTOR_COUNT = 4;

static inline double bound(double val, double bottom, double top){
    return (val < top)?
           ((val > bottom)?(val):(bottom)) : (top);
}

class drone {
    motor* motors;
    mpu6050_handler mpu6050;
    bmp280_handler bmp280;

    double YPR_tar[3] = {};

    double YPR_diff_n [3] = {}; //YPR diff on this iteration
    double YPR_diff_n1[3] = {}; //YPR diff on previous iteration
    double YPR_diff_n2[3] = {}; //YPR diff on prev-previous iteration
    
    //Y  P  R
    double PID_Kprop[3] = {1, 1, 1};
    double PID_Kdiff[3] = {2, 2, 2};
    double PID_Kintegr[3] = {0.1, 0.1, 0.1};

    double Control_val_YPR[3] = {};

    double max_throttle = 30;
    double throttle = 0;

public:

    ~drone() {
        delete[] motors;
    }

    void initialize_motors_and_timer(
        ledc_timer_bit_t timer_resolution, ledc_mode_t timer_mode, uint32_t frequency, int min_period_ms, int max_period_ms,
        int GPIO_UP_L, int GPIO_UP_R, int GPIO_DOWN_R, int GPIO_DOWN_L) {

        //initializing timer_0
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = timer_mode,
            .duty_resolution  = timer_resolution,
            .timer_num        = LEDC_TIMER_0,
            .freq_hz          = frequency,
            .clk_cfg          = LEDC_AUTO_CLK,
            .deconfigure      = false
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        //motors
        motors = new motor[MOTOR_COUNT] (
        {LEDC_CHANNEL_0, GPIO_UP_L,   timer_resolution, timer_mode, frequency, min_period_ms, max_period_ms},
        {LEDC_CHANNEL_1, GPIO_UP_R,   timer_resolution, timer_mode, frequency, min_period_ms, max_period_ms},
        {LEDC_CHANNEL_2, GPIO_DOWN_R, timer_resolution, timer_mode, frequency, min_period_ms, max_period_ms},
        {LEDC_CHANNEL_3, GPIO_DOWN_L, timer_resolution, timer_mode, frequency, min_period_ms, max_period_ms});
        
        for(int i = 0; i < MOTOR_COUNT; ++i){
            motors[i].init_motor();
        }
    };

    void initialize_sensors(){
        init_sensors(&mpu6050, &bmp280);
    };

    void set_duty(short index, float percents){
        motors[index].throttle() = percents;
        motors[index].update_duty();
    };

    void set_duty(float percents){
        for(int i = 0; i < MOTOR_COUNT; ++i){
            if(motors[i].throttle() != percents){
                motors[i].throttle() = percents;
                motors[i].update_duty();
            }
        }
    }

    void read_sensors_values(){
        read_sensors(&mpu6050, &bmp280);
    }

    void print_state(){
        printf("\033[0H\033[0J");
        printf("T_YAW: %3.1f, T_PITCH: %3.1f, T_ROLL: %3.1f \nYAW:   %3.1f, PITCH:   %3.1f, ROLL:   %3.1f \ng: [%.2f, %.2f, %.2f] \npressure: %.2f, temperature: %.2f\nPID: %.3f, %.3f, %.3f\n %.2f %.2f %.2f %.2f\n", 
                YPR_tar[0], YPR_tar[1], YPR_tar[2],
                mpu6050.ypr[0] * 180/M_PI, mpu6050.ypr[1] * 180/M_PI, mpu6050.ypr[2] * 180/M_PI, mpu6050.gravity.x, mpu6050.gravity.y, mpu6050.gravity.z, 
                bmp280.pressure, bmp280.temperature, 
                Control_val_YPR[0], Control_val_YPR[1], Control_val_YPR[2],
                motors[0].throttle(),  motors[1].throttle(),  motors[2].throttle(),  motors[3].throttle()
            );
    }

    void set_targets(float Y, float P, float R){
        YPR_tar[0] = Y;
        YPR_tar[1] = P;
        YPR_tar[2] = R;
    }

    void set_throttle(float tar_throttle){
        throttle =  bound(tar_throttle, 0, max_throttle);
    }

    void update_motors(){
        /*

    -Y<-  +P  ->+Y
        0   1
         \ /
    +R    ^    -R
         / \
        3   2
          -P
        
          */

        set_duty(0, bound(throttle + Control_val_YPR[0] - Control_val_YPR[1] - Control_val_YPR[2], 0, max_throttle));
        set_duty(1, bound(throttle - Control_val_YPR[0] - Control_val_YPR[1] + Control_val_YPR[2], 0, max_throttle));
        set_duty(2, bound(throttle + Control_val_YPR[0] + Control_val_YPR[1] + Control_val_YPR[2], 0, max_throttle));
        set_duty(3, bound(throttle - Control_val_YPR[0] + Control_val_YPR[1] - Control_val_YPR[2], 0, max_throttle));
    }

    void processPID(){
        //replacing iteration steps with new ones
        //getting new controlling values on Yaw (0), Pitch (1) and Roll (2)

        bool YPR_tune[3] = {true, true, true};
        int PID_tune[3] = {1, 1, 1};

        double val = 0;
        for(int i = 0; i < 3; i++) {
            if(YPR_tune[i]){
                YPR_diff_n2[i] = YPR_diff_n1[i];
                YPR_diff_n1[i] = YPR_diff_n[i];
                YPR_diff_n[i] = YPR_tar[i] - mpu6050.ypr[i];
    
                /* U(n) =   U(n-1) + 
                            Kp * (E(n) - E(n-1)) +
                            Ki * E(n) + 
                            Kd * (E(n) - 2E(n-1) + E(n-2))
                */

                val =   Control_val_YPR[i] + 
                        PID_tune[0] * PID_Kprop[i]    * (YPR_diff_n[i] - YPR_diff_n1[i]) + 
                        PID_tune[1] * PID_Kintegr[i]  *  YPR_diff_n[i] + 
                        PID_tune[2] * PID_Kdiff[i]    * (YPR_diff_n[i] - 2*YPR_diff_n1[i] + YPR_diff_n2[i]);
                Control_val_YPR[i] = val;
    
                // ESP_LOGI("PID", "%d: %f", i, val);
            }
        }
    }


};