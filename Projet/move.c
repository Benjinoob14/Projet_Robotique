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
static int8_t mode=SUIVI_LIGNE;

//simple PID regulator implementation
int16_t pid_regulator(float position, float goal){

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

	//on divise par deux pour r�partir l'erreur entre les deux roues
	speed_correction= speed_correction/2;

	last_error=error_position;

//	chprintf((BaseSequentialStream *)&SDU1, "daamam= %f  ",speed_correction);

	//if the line is nearly in front of the camera, don't rotate
	if(abs(speed_correction) < ROTATION_THRESHOLD){
		speed_correction = 0;
	}

	if (speed_correction>VITESSE_STABLE_PLAT){
		speed_correction=VITESSE_STABLE_PLAT;
	}
	if (speed_correction<-VITESSE_STABLE_PLAT){
		speed_correction=-VITESSE_STABLE_PLAT;
	}


    return (int16_t)speed_correction;
}

static THD_WORKING_AREA(waMove, 1024);
static THD_FUNCTION(Move, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    uint8_t compteur_sans_ligne=0;
    int16_t speed_correction = 0;

    uint16_t line=0;

    uint8_t compteur_ligne=0;


    while(1){


    	mode=1;
        time = chVTGetSystemTime();


//        chprintf((BaseSequentialStream *)&SDU1, "%d ",mode);

        if (mode==SUIVI_LIGNE_PENTE || mode==SUIVI_LIGNE){

			//computes the speed to give to the motors
			//line_width is modified by the image processing thread
			speed_correction = pid_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2);

			//on r�cup�re la taille de la ligne
			line=get_line_width();

			if (line<SENSIBILITY_LIGNE){
				compteur_sans_ligne++;
			}
			if (line<SENSIBILITY_LIGNE && compteur_sans_ligne>FAUX_POSITIF_LIGNE){
				right_motor_set_speed(0);
				left_motor_set_speed(0);
	//				set_led(LED3,FALSE);
	//				set_body_led(TRUE);
	//				chThdSleepMilliseconds(TEMPS_ATTENTE/2);
	//				chThdSleepMilliseconds(TEMPS_ATTENTE/2);
			}


        	if (mode==SUIVI_LIGNE_PENTE){

				set_body_led(TRUE);

				//fait avancer le robot s'il voit une ligne
				if (line>SENSIBILITY_LIGNE){
					compteur_sans_ligne=0;
//					right_motor_set_speed(VITESSE_STABLE_PENTE - speed_correction);
//					left_motor_set_speed(VITESSE_STABLE_PENTE +  speed_correction);
				}
			}

			if (mode==SUIVI_LIGNE ){

				set_body_led(FALSE);


				//fait avancer le robot s'il voit une ligne
				if (line>SENSIBILITY_LIGNE){
					compteur_sans_ligne=0;
//					right_motor_set_speed(VITESSE_STABLE_PLAT - speed_correction);
//					left_motor_set_speed(VITESSE_STABLE_PLAT +  speed_correction);
				}

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
				left_motor_set_speed(0.6*VITESSE_ROTATION);
				compteur_ligne=get_compteur_liigne();
			}

			if(mode==MILIEU_CONTOURNEMENT && compteur_ligne>FAUX_POSITIF_LIGNE){
				chThdSleepMilliseconds(3100);
				mode=FIN_CONTOURNEMENT;
			}
			if(mode==FIN_CONTOURNEMENT){
				right_motor_set_speed(-VITESSE_ROTATION);
				left_motor_set_speed(VITESSE_ROTATION);
				chThdSleepMilliseconds(TEMPS_ATTENTE);
				if(line>SENSIBILITY_LIGNE){
					compteur_ligne=0;
					set_led(LED3,FALSE);
					set_led(LED7,FALSE);
					mode=SUIVI_LIGNE;
				}

			}
		}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}


//la thread qui check � chaque fois dans quel mode le robot se trouve et modifie en fonction de s'il voit un obstacle ou s'il est en pente
static THD_WORKING_AREA(waCheckMODE, 256);
static THD_FUNCTION(CheckMODE, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int8_t incliined=0;

    volatile uint16_t *proxi_front = get_proxi();

    volatile uint16_t *proxi_front_left=proxi_front+1;


    while(1){
    	time = chVTGetSystemTime();

       	incliined = get_inclined();

//		if(incliined==DESCEND && mode<DEBUT_CONTOURNEMENT){
//			mode=SUIVI_LIGNE_PENTE;
//			set_led(LED3,0);
//			set_led(LED7,1);
//		}
//		if(incliined==MONTE && mode<DEBUT_CONTOURNEMENT){
//			mode=SUIVI_LIGNE_PENTE;
//			set_led(LED3,1);
//			set_led(LED7,0);
//		}
//		if(incliined==PLAT && mode<DEBUT_CONTOURNEMENT){
//    		mode=SUIVI_LIGNE;
//			set_led(LED3,0);
//			set_led(LED7,0);
//		}

//
//		if (*proxi_front>SENSIBLE_PROX && mode==SUIVI_LIGNE){
//			mode=DEBUT_CONTOURNEMENT;
//		}
//		if(mode==MILIEU_CONTOURNEMENT && *proxi_front_left>200 ){
//			mode=DEBUT_CONTOURNEMENT;
//		}

    	//100Hz
    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void move_start(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
	chThdCreateStatic(waCheckMODE, sizeof(waCheckMODE), NORMALPRIO, CheckMODE, NULL);
}
