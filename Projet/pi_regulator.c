#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

#define POSITION_NOT_REACHED	0
#define POSITION_REACHED       	1

#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

static bool turn = 0;


//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0, compteur=0;
    int16_t speed_correction = 0;
    static uint32_t position=0;

    bool black_line=1;

    while(1){
        time = chVTGetSystemTime();

        //computes the speed to give to the motors
        //black_line is modified by the image processing thread
        speed = pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2);
        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));


        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }


        if (speed<0 ){
        	speed=0;
        }

        if (speed>100){
        	speed=100;
        }

        black_line=get_black_line();
        compteur++;

//        if (black_line==1){
//        	compteur=0;
//        }
//
//        if (black_line==0 && compteur>5){
//        	right_motor_set_speed(0);
//        	left_motor_set_speed(0);
//        }
//
//        else{
//        	//applies the speed from the PI regulator and the correction for the rotation
//        	right_motor_set_speed(speed+100 - ROTATION_COEFF * speed_correction);
//        	left_motor_set_speed(speed+100 + ROTATION_COEFF * speed_correction);
//        }

        left_motor_set_pos(123);
        position = left_motor_get_pos();




        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));




    }
}
//
//static THD_WORKING_AREA(waCheckStep, 124);
//static THD_FUNCTION(CheckStep, arg) {
//
//	uint32_t position=0;
//
//	while(1){
////		chThdSleepMilliseconds(10);
////		position= left_motor_get_pos();
//		if(turn==1 && position>200){
//			turn=0;
//		}
//
//	}
//}
//
////bool get_turn(){
////	return turn;
////}

//

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), HIGHPRIO, PiRegulator, NULL);
//	chThdCreateStatic(CheckStep, sizeof(waCheckStep), NORMALPRIO, CheckStep, NULL);
}
