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

//plouf

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

	//on divise par deux pour rÃ©partir l'erreur entre les deux roues
	speed_correction= speed_correction/2;

	last_error=error_position;

//	chprintf((BaseSequentialStream *)&SDU1, "daamam= %f  ",speed_correction);

	//if the line is nearly in front of the camera, don't rotate
	if(abs(speed_correction) < ROTATION_THRESHOLD){
		speed_correction = 0;
	}

	if(mode==SUIVI_LIGNE){
		if (speed_correction>VITESSE_STABLE_PLAT){
			speed_correction=VITESSE_STABLE_PLAT;
		}
		if (speed_correction<-VITESSE_STABLE_PLAT){
			speed_correction=-VITESSE_STABLE_PLAT;
		}
	}
	else{
		if (speed_correction>VITESSE_STABLE_PENTE){
			speed_correction=VITESSE_STABLE_PENTE;
		}
		if (speed_correction<-VITESSE_STABLE_PENTE){
			speed_correction=-VITESSE_STABLE_PENTE;
		}
	}

    return (int16_t)speed_correction;
}
/*
 * le thread le plus importante car il regroupe tout le reste en indiquant au robot quel
 * mouvement il doit procéder en fonction du mode dans lequel il se trouve
 */
static THD_WORKING_AREA(waMove, 256);
static THD_FUNCTION(Move, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    uint8_t compteur_sans_ligne=0;
    int16_t speed_correction = 0;

    uint16_t line=0;
    bool rebond=FALSE;

    uint8_t compteur_ligne=0;


    while(1){

        time = chVTGetSystemTime();



//        chprintf((BaseSequentialStream *)&SDU1, "%d ",mode);

        if (mode==SUIVI_LIGNE_PENTE_MONTEE || mode==SUIVI_LIGNE || mode==SUIVI_LIGNE_PENTE_DESCENTE){

			//computes the speed to give to the motors
			//line_width is modified by the image processing thread
			speed_correction = pid_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2);

			//on récupère la taille de la ligne
			line=get_line_width();

			//si la ligne est trop petite on incrimente pour arreter le robot s'il n'y plus de ligne devant plusieurs fois de suite
			if (line<SENSIBILITY_LIGNE){
				compteur_sans_ligne++;
			}
			//arrete le robot s'il n'y a plus de ligne devant lui
			if (line<SENSIBILITY_LIGNE && compteur_sans_ligne>FAUX_POSITIF_LIGNE){
				right_motor_set_speed(0);
				left_motor_set_speed(0);
			}


        	if (mode==SUIVI_LIGNE_PENTE_MONTEE || mode==SUIVI_LIGNE_PENTE_DESCENTE){

				set_body_led(TRUE);

				if( mode==SUIVI_LIGNE_PENTE_MONTEE){
					toggle_rgb_led(LED2,GREEN_LED,INTENSITY);
					toggle_rgb_led(LED4,GREEN_LED,INTENSITY);
					toggle_rgb_led(LED6,GREEN_LED,INTENSITY);
					toggle_rgb_led(LED8,GREEN_LED,INTENSITY);
				}
				else{
					toggle_rgb_led(LED2,BLUE_LED,INTENSITY);
					toggle_rgb_led(LED4,BLUE_LED,INTENSITY);
					toggle_rgb_led(LED6,BLUE_LED,INTENSITY);
					toggle_rgb_led(LED8,BLUE_LED,INTENSITY);

				}

				//fait avancer le robot s'il voit une ligne
				if (line>SENSIBILITY_LIGNE){
					compteur_sans_ligne=0;
					right_motor_set_speed(VITESSE_STABLE_PENTE - speed_correction);
					left_motor_set_speed(VITESSE_STABLE_PENTE +  speed_correction);
				}
			}

			if (mode==SUIVI_LIGNE ){

				set_body_led(FALSE);
				//fait avancer le robot s'il voit une ligne
				if (line>SENSIBILITY_LIGNE){
					compteur_sans_ligne=0;
					right_motor_set_speed(VITESSE_STABLE_PLAT - speed_correction);
					left_motor_set_speed(VITESSE_STABLE_PLAT +  speed_correction);
				}

			}

        }

        if (mode>=DEBUT_CONTOURNEMENT){

			set_led(LED3,TRUE);
			set_led(LED7,TRUE);

			compteur_ligne=get_counter_line();


			if (mode==DEBUT_CONTOURNEMENT){
				//on se positionne une fois pour un grand virage
				if (rebond==FALSE){
					right_motor_set_speed(-VITESSE_ROTATION);
					left_motor_set_speed(VITESSE_ROTATION);
					chThdSleepMilliseconds(TEMPS_ATTENTE_ROT);
					rebond=TRUE;
				}
				//ensuite on ne fait que des rebond et donc des virages plus petits
				else{
					right_motor_set_speed(-VITESSE_ROTATION);
					left_motor_set_speed(VITESSE_ROTATION);
					chThdSleepMilliseconds(TEMPS_ATTENTE_REBOND);
				}
				right_motor_set_speed(VITESSE_VIRAGE_ROUE_EXT);
				left_motor_set_speed(VITESSE_VIRAGE_ROUE_INT);
				chThdSleepMilliseconds(TEMPS_ATTENTE);
				mode=MILIEU_CONTOURNEMENT;
			}

			compteur_ligne=get_counter_line();

			//si le robot est entrain de contourner et revoie des pixels noirs il se prépare à s'arreter
			//il l'indique par le fait de clignoter les LEDs
			if(mode==MILIEU_CONTOURNEMENT && compteur_ligne>FAUX_POSITIF_REPLACEMENT){
				rebond=FALSE;
				set_led(LED3,FALSE);
				set_led(LED7,FALSE);
				chThdSleepMilliseconds(MINI_ATTENTE);
				set_led(LED3,TRUE);
				set_led(LED7,TRUE);
				chThdSleepMilliseconds(MINI_ATTENTE);
				set_led(LED3,FALSE);
				set_led(LED7,FALSE);
				chThdSleepMilliseconds(MINI_ATTENTE);
				set_led(LED3,TRUE);
				set_led(LED7,TRUE);
				chThdSleepMilliseconds(MINI_ATTENTE);

				//dès qu'il est sur la ligne il tourne sur lui
				//on divise pas deux la vitesse de rotation car on veut lui laisser le temps de bien voir la ligne quand il se replace
				right_motor_set_speed(-VITESSE_ROTATION_REPLACEMENT);
				left_motor_set_speed(VITESSE_ROTATION_REPLACEMENT);
				chThdSleepMilliseconds(MINI_ATTENTE);
				mode=FIN_CONTOURNEMENT;
			}

			compteur_ligne=get_counter_line();

			if(mode==FIN_CONTOURNEMENT){
				if(compteur_ligne<FAUX_POSITIF_REPLACEMENT){
					//il toune sur lui meme pour retrouver la ligne
					//on divise pas deux la rotation car on veut lui laisser le temps de bien voir la ligne quand il se replace
					right_motor_set_speed(-VITESSE_ROTATION_REPLACEMENT);
					left_motor_set_speed(VITESSE_ROTATION_REPLACEMENT);
				}
				//s'il retrouve une ligne il se remet en mode de suivi de ligne
				else{
					right_motor_set_speed(0);
					left_motor_set_speed(0);
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


//la thread qui check Ã  chaque fois dans quel mode le robot se trouve et modifie en fonction de s'il voit un obstacle ou s'il est en pente
static THD_WORKING_AREA(waCheckMODE, 256);
static THD_FUNCTION(CheckMODE, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    /*
    * on récupère les valeurs des detecteurs
    * 1. proximity fontal
    * 2. proximity lateral gauche 45deg
    * 3. l'inclinaison
    */
    valeurs Capteurs = {0,0,0};

    while(1){
    	time = chVTGetSystemTime();

    	Capteurs = get_reception();

    	//si l'inclinaison est vers l'avant
		if(Capteurs.inclinaison==DESCENTE && mode<DEBUT_CONTOURNEMENT){
			clear_leds();
			mode=SUIVI_LIGNE_PENTE_DESCENTE;
		}

		//si l'inclinaison est vers l'arrière
		if(Capteurs.inclinaison==MONTEE && mode<DEBUT_CONTOURNEMENT){
			clear_leds();
			mode=SUIVI_LIGNE_PENTE_MONTEE;

		}
		//si le robot est à plat
		if(Capteurs.inclinaison==PLAT && mode<DEBUT_CONTOURNEMENT){
			clear_leds();
			mode=SUIVI_LIGNE;

		}

		//se lance dans un mode de contournement s'il le capteur de devant detecte un objet
		if (mode<FIN_CONTOURNEMENT && Capteurs.frontal>SENSIBLE_PROX_FRONT){
			mode=DEBUT_CONTOURNEMENT;
		}
		//se lance dans un rebond s'il revoit un objet pendant le contournement
		if(mode==MILIEU_CONTOURNEMENT && Capteurs.lateral>SENSIBLE_PROX_LEFT){
			mode=DEBUT_CONTOURNEMENT;
		}


    	chThdSleepUntilWindowed(time, time + MS2ST(30));
    }
}

void move_start(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
	chThdCreateStatic(waCheckMODE, sizeof(waCheckMODE), NORMALPRIO, CheckMODE, NULL);
}
