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

    uint8_t compteur=0;
    int16_t speed = 0;
    int16_t speed_correction = 0;

    bool black_line=1;


    while(1){
        time = chVTGetSystemTime();

        inclined = get_inclined();

//        if(inclined==1){
//        	mode=SUIVIT_LIGNE_PENTE;
//        }
//        else{
//        	mode=SUIVIT_LIGNE;
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

			if (black_line==TRUE){
				compteur=0;
				right_motor_set_speed(speed+VITESSE_STABLE/2 - ROTATION_COEFF * speed_correction);
				left_motor_set_speed(speed+VITESSE_STABLE/2 + ROTATION_COEFF * speed_correction);
			}
			if (black_line==FALSE && compteur>FAUX_POSITIF_LIGNE){
				right_motor_set_speed(0);
				left_motor_set_speed(0);
//				set_body_led(TRUE);
//				chThdSleepMilliseconds(TEMPS_ATTENTE/2);
				set_body_led(FALSE);
//				chThdSleepMilliseconds(TEMPS_ATTENTE/2);
			}

		}

        if (mode==SUIVIT_LIGNE_PENTE ){

                	set_body_led(1);
//                	right_motor_set_speed(0);
//					left_motor_set_speed(0);
        }

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
    uint8_t compteur_ligne=0;
    bool tour=0;

    while(1){
    	time = chVTGetSystemTime();

    	proxii=get_proxii();

		if (proxii>SENSIBLE_PROX && mode!=SUIVIT_LIGNE_PENTE){
			mode=CONTOURNEMENT;
		}
		if (mode==CONTOURNEMENT){

			set_led(LED3,TRUE);
			set_led(LED7,TRUE);
			set_body_led(FALSE);

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

			compteur_ligne=get_compteur_liigne();


			if(compteur_ligne>FAUX_POSITIF_LIGNE/2){
				chThdSleepMilliseconds(1.2*TEMPS_ATTENTE);
				right_motor_set_speed(-VITESSE_ROTATION);
				left_motor_set_speed(VITESSE_ROTATION);
				chThdSleepMilliseconds(TEMPS_ATTENTE);
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				mode=SUIVIT_LIGNE;
				tour=0;
				compteur_ligne=0;
				set_led(LED3,FALSE);
				set_led(LED7,FALSE);
			}
		}
    	//100Hz
    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}



void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
	chThdCreateStatic(waContournement, sizeof(waContournement), NORMALPRIO, Contournement, NULL);
}
