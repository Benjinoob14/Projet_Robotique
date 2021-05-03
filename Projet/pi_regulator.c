#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <leds.h>


static int8_t mode=0;


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

static THD_WORKING_AREA(waPiRegulator, 1024);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    bool inclined=0;

    int16_t speed = 0, compteur=0;
    int16_t speed_correction = 0;

    bool black_line=1;

    get_inclined();

    mode=0;

    while(1){
        time = chVTGetSystemTime();


//        if(inclined==1){
//        	mode=SUIVIT_LIGNE_BACK;
//        }

        if (mode==SUIVIT_LIGNE ){

        	set_body_led(0);

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

			if (speed>VITESSE_STABLE){
				speed=VITESSE_STABLE;
			}

			black_line=get_black_line();
	        compteur++;

	        if (black_line==1){
	        	compteur=0;
	        	right_motor_set_speed(speed+VITESSE_STABLE/2 - ROTATION_COEFF * speed_correction);
				left_motor_set_speed(speed+VITESSE_STABLE/2 + ROTATION_COEFF * speed_correction);
	        }
	        if (black_line==0 && compteur>5){
	        	right_motor_set_speed(0);
	        	left_motor_set_speed(0);
	        	set_body_led(1);
	        	chThdSleepMilliseconds(TEMPS_ATTENTE/2);
	        	set_body_led(0);
	        	chThdSleepMilliseconds(TEMPS_ATTENTE/2);
	        }

        }
/*
        if (mode==SUIVIT_LIGNE_BACK ){

//        			chprintf((BaseSequentialStream *)&SD1, "mode = %dus\n",mode);

                	set_body_led(0);

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

        			if (speed>VITESSE_STABLE){
        				speed=VITESSE_STABLE;
        			}

//        			speed=-speed;

        			black_line=get_black_line();
        	        compteur++;

        	        if (black_line==1){
        	        	compteur=0;
        	        	right_motor_set_speed(speed+VITESSE_STABLE/2 - ROTATION_COEFF * speed_correction);
        				left_motor_set_speed(speed+VITESSE_STABLE + ROTATION_COEFF * speed_correction);
        	        }
        	        if (black_line==0 && compteur>5){
        	        	right_motor_set_speed(0);
        	        	left_motor_set_speed(0);
        	        	set_body_led(1);
        	        	chThdSleepMilliseconds(TEMPS_ATTENTE/2);
        	        	set_body_led(0);
        	        	chThdSleepMilliseconds(TEMPS_ATTENTE/2);
        	        }

        }
*/
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}
static THD_WORKING_AREA(waContournement, 1024);
static THD_FUNCTION(Contournement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    uint8_t proxii=0;
    uint8_t ligne=0;
    bool tour=0;

    while(1){
    	time = chVTGetSystemTime();

    	proxii=get_proxii();

		if (proxii>SENSIBLE_PROX && mode!=2){
			mode=CONTOURNEMENT;
		}
		if (mode==CONTOURNEMENT){

			set_led(LED3,1);
			set_led(LED7,1);
			set_body_led(0);

			if (tour==0){
				right_motor_set_speed(-VITESSE_ROTATION);
				left_motor_set_speed(VITESSE_ROTATION);
				chThdSleepMilliseconds(TEMPS_ATTENTE);
				right_motor_set_speed(VITESSE_ROTATION);
				left_motor_set_speed(0.57*VITESSE_ROTATION);
				chThdSleepMilliseconds(2*TEMPS_ATTENTE);
				tour=1;
			}
			else{
				right_motor_set_speed(VITESSE_ROTATION);
				left_motor_set_speed(0.57*VITESSE_ROTATION);
			}
			ligne=get_liigne();

			if(ligne>2){
				chThdSleepMilliseconds(1.2*TEMPS_ATTENTE);
				right_motor_set_speed(-VITESSE_ROTATION);
				left_motor_set_speed(VITESSE_ROTATION);
				chThdSleepMilliseconds(TEMPS_ATTENTE);
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				mode=0;
				tour=0;
				ligne=0;
				set_led(LED3,0);
				set_led(LED7,0);
			}
		}
    	//100Hz
    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}



void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
//	chThdCreateStatic(waContournement, sizeof(waContournement), NORMALPRIO, Contournement, NULL);
}
