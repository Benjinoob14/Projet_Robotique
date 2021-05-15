#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <process_info.h>
#include <leds.h>
#include <move.h>


//permet de savoir le mode du robot
//mode0->suivit de ligne
//mode1->contournement
//mode2->suivit de ligne en pente
static int8_t mode=1;

//simple PI regulator implementation
int16_t pi_regulator(float position, float goal){

	float error_position = 0;
	float speed_correction = 0;

	static float sum_error = 0;
	static float last_error=0;

	error_position = (position - goal);

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error_position) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error_position;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed_correction = KP * error_position + KI * sum_error + KD * (error_position-last_error);

	last_error=error_position;

	//if the line is nearly in front of the camera, don't rotate
	if(abs(speed_correction) < ROTATION_THRESHOLD){
		speed_correction = 0;
	}

	if (speed_correction>VITESSE_STABLE){
		speed_correction=VITESSE_STABLE;
	}
	if (speed_correction<-VITESSE_STABLE){
		speed_correction=-VITESSE_STABLE;
	}

    return (int16_t)speed_correction;
}

static THD_WORKING_AREA(waMove, 1024);
static THD_FUNCTION(Move, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    uint8_t compteur=0;
    int16_t speed_correction = 0;

    uint16_t line=0;

    uint8_t compteur_ligne=0;


    while(1){


        time = chVTGetSystemTime();


//        chprintf((BaseSequentialStream *)&SDU1, "%d ",mode);

        if (mode==SUIVIT_LIGNE_PENTE){

			set_body_led(TRUE);

			//computes the speed to give to the motors
			//line_width is modified by the image processing thread
			speed_correction = pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2);

			//on récupère la taille de la ligne
			line=get_line_width();
			compteur++;

//			chprintf((BaseSequentialStream *)&SDU1, "line= %d ",line);

			if (line>SENSIBILITY_LIGNE){
				compteur=0;
				right_motor_set_speed(1.2*VITESSE_STABLE - speed_correction);
				left_motor_set_speed(1.2*VITESSE_STABLE +  speed_correction);
			}
			if (line<SENSIBILITY_LIGNE && compteur>FAUX_POSITIF_LIGNE){
				right_motor_set_speed(0);
				left_motor_set_speed(0);
//				set_led(LED3,FALSE);
//				set_body_led(TRUE);
//				chThdSleepMilliseconds(TEMPS_ATTENTE/2);
//				chThdSleepMilliseconds(TEMPS_ATTENTE/2);
			}
		}

        if (mode==SUIVIT_LIGNE ){

        	set_body_led(FALSE);

        	//computes the speed to give to the motors
			//line_width is modified by the image processing thread
			speed_correction = pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2);


			//on récupère la taille de la ligne
			line=get_line_width();
			compteur++;

//			chprintf((BaseSequentialStream *)&SDU1, "line= %d ",line);

			if (line>SENSIBILITY_LIGNE){
				compteur=0;
				right_motor_set_speed(VITESSE_STABLE - speed_correction/2);
				left_motor_set_speed(VITESSE_STABLE +  speed_correction/2);
			}
			if (line<SENSIBILITY_LIGNE && compteur>FAUX_POSITIF_LIGNE){
				right_motor_set_speed(0);
				left_motor_set_speed(0);
//				set_body_led(TRUE);
//				chThdSleepMilliseconds(TEMPS_ATTENTE/2);
//				chThdSleepMilliseconds(TEMPS_ATTENTE/2);
			}

        }


        if (mode>=DEBUT_CONTOURNEMENT){

			set_led(LED3,TRUE);
			set_led(LED7,TRUE);
			set_body_led(FALSE);

			if (mode==DEBUT_CONTOURNEMENT){
				right_motor_set_speed(-VITESSE_ROTATION);
				left_motor_set_speed(VITESSE_ROTATION);
				chThdSleepMilliseconds(1.2*TEMPS_ATTENTE);
				mode=MILIEU_CONTOURNEMENT;
			}
			if(mode==MILIEU_CONTOURNEMENT){
				right_motor_set_speed(VITESSE_ROTATION);
				left_motor_set_speed(0.5*VITESSE_ROTATION);
			}

			compteur_ligne=get_compteur_liigne();


			if(mode==MILIEU_CONTOURNEMENT && compteur_ligne>FAUX_POSITIF_LIGNE/4){
				chThdSleepMilliseconds(1.2*TEMPS_ATTENTE);
				mode=FIN_CONTOURNEMENT;
			}
			if(mode==FIN_CONTOURNEMENT){
				right_motor_set_speed(-VITESSE_ROTATION);
				left_motor_set_speed(VITESSE_ROTATION);
				chThdSleepMilliseconds(TEMPS_ATTENTE);
				mode=SUIVIT_LIGNE;
				compteur_ligne=0;
				set_led(LED3,FALSE);
				set_led(LED7,FALSE);
			}
		}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}


//la thread qui check à chaque fois dans quel mode le robot se trouve et modifie en fonction de s'il voit un obstacle ou s'il est en pente
static THD_WORKING_AREA(waCheckMODE, 256);
static THD_FUNCTION(CheckMODE, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int8_t incliined=0;

    volatile uint16_t proxii_tab[2]={0};

    volatile uint16_t proxii=0;


    while(1){
    	time = chVTGetSystemTime();

    	incliined = get_inclined();

//		if(incliined==DESCEND && mode<DEBUT_CONTOURNEMENT){
//			mode=SUIVIT_LIGNE_PENTE;
//			set_led(LED3,0);
//			set_led(LED7,1);
//		}
//		if(incliined==MONTE && mode<DEBUT_CONTOURNEMENT){
//			mode=SUIVIT_LIGNE_PENTE;
//			set_led(LED3,1);
//			set_led(LED7,0);
//		}
//		if(incliined==PLAT && mode<DEBUT_CONTOURNEMENT){
//    		mode=SUIVIT_LIGNE;
//			set_led(LED3,0);
//			set_led(LED7,0);
//		}




//		chprintf((BaseSequentialStream *)&SDU1, "proxii= %d ",proxii);

		if (proxii_tab[0]>SENSIBLE_PROX && mode==SUIVIT_LIGNE){
			mode=DEBUT_CONTOURNEMENT;
		}
		if(mode==MILIEU_CONTOURNEMENT && proxii_tab[1]>200 ){
			mode=DEBUT_CONTOURNEMENT;
		}

    	//100Hz
    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void move_start(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
	chThdCreateStatic(waCheckMODE, sizeof(waCheckMODE), NORMALPRIO, CheckMODE, NULL);
}
