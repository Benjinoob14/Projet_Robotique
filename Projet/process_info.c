#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <math.h>
#include <camera/po8030.h>
#include <sensors/proximity.h>
#include <sensors/imu.h>
#include <leds.h>
#include <process_info.h>


static uint16_t line_width = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static valeurs reception = {0,0,0}; //structure recevant les valeurs des deux capteurs et l'inclinaison
static uint8_t counter_line=0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//calcul pour savoir si le robot est inclin? en avant, en arri?re ou ? plat
int8_t show_inclined(int16_t accel_value, int16_t calibrate){

	int16_t value = 0;


    value=accel_value-calibrate;




    if (value>MAX_PROXI){
    	value=MAX_PROXI;
    }
    if (value<-MAX_PROXI){
       	value=-MAX_PROXI;
    }


    if(abs(value) > SENSI_PENTE){


        if(value>0){
        	 return MONTEE;
        }
        if(value<0){
        	return DESCENTE;
        }
        else{
        	return PLAT;
        }
    }
    else{
    	return PLAT;
    }

    return PLAT;

}
/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 *  Il scan l?image de gauche ? droite pour trouver des pixels noirs, s?il y en a, le compteur de ligne est incr?ment?.
 *  Ensuite, il regarde s?il trouve plusieurs pixels ? la suite qui se finissent ? un moment, cela veut dire qu?il y a
 *  une ligne devant lui et renvoie la taille de la ligne et change la valeur de la position de la ligne
 *
 */
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = 0;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	//s'il voit une suite de pixels noirs il incr?mente le compteur qui permet de d?terminer si le robot est revenu ou non sur une ligne noire
	if(buffer[IMAGE_BUFFER_SIZE/2]< VALEUR_SENSIBLE_DETECTION_BLACK){
		counter_line++;
	}
	//evite l'overflow
	if(counter_line>MAX_COMPTEUR){
		counter_line=MAX_COMPTEUR;
	}
	//remet ? z?ro le compteur s'il ne voit pas de ligne ou si le compteur arrive au max
	if(buffer[IMAGE_BUFFER_SIZE/2]> VALEUR_SENSIBLE_DETECTION_BLACK){
		counter_line=0;
	}


	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}

	}while(wrong_line);


	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	return width;

}
//capture une image ? l?aide de la cam?ra
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }

}

//lorsque l?image est prise, elle est mise dans le buffer puis envoy? dans extract_line_width pour ?tre analys?
static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	bool send_to_computer = true;



    while(1){


    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image and gets its width in pixels
		line_width=extract_line_width(image);


		if (line_width<MIN_LINE_WIDTH || line_width>10*MIN_LINE_WIDTH){
			line_width=0;
		}



		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}
/*
 *  r?cup?re la valeur des d?tecteurs de proximit? et ceux de l'acc?l?rom?tre et v?rifie ? l?aide d?un compteur
 *  s?il ne s?agit pas d?un faux changement (si plusieurs fois la m?me valeur alors c?est bon)
 *
 */
static THD_WORKING_AREA(waInfoMode, 512);
static THD_FUNCTION(InfoMode, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t accel_value=0;
    systime_t time;

    //calibre l'accelerom?tre
    calibrate_acc();

    int8_t value=0;
    uint8_t compteur_inclinaison=0;
    uint8_t compteur_prox1 = 0;
    uint8_t compteur_prox2 = 0;
    uint16_t prox_tab[2]={0};
    int16_t calibrate= get_acc_offset(Y_AXIS);

    while(1){
    	time = chVTGetSystemTime();

		prox_tab[PROX1]=get_prox(SENSOR_FRONT);
		prox_tab[PROX2]=get_prox(SENSOR_FRONT_LEFT);


		//*****v?rification de valeurs pour ne pas avoir de faux positifs*****


		//s'il voit que la proximit? est assez proche le compteur est incr?ment?
		if(prox_tab[PROX1] > SENSIBLE_PROX_FRONT){
			compteur_prox1++;
		}
		//?vite l'overflow
		if(compteur_prox1>MAX_COMPTEUR){
			compteur_prox1=MAX_COMPTEUR;
		}
		//remet ? z?ro le compteur si la valeur est trop faible
		if(prox_tab[PROX1] < SENSIBLE_PROX_FRONT){
			compteur_prox1=0;
			reception.frontal=0;
		}
		//si il y a plusieurs fois de suite une valeur acceptable on envoie la valeur
		if(compteur_prox1>FAUX_POSITIF_PROX){
			reception.frontal=prox_tab[PROX1];
		}


		//s'il voit que la proximit? est assez proche le compteur est incr?ment?
		if(prox_tab[PROX2] > SENSIBLE_PROX_LEFT){
			compteur_prox2++;
		}
		//?vite l'overflow
		if(compteur_prox2>MAX_COMPTEUR){
			compteur_prox2=MAX_COMPTEUR;
		}
		//remet ? z?ro le compteur si la valeur est trop faible
		if(prox_tab[PROX2] < SENSIBLE_PROX_LEFT){
			compteur_prox2=0;
			reception.lateral=0;
		}
		//si il y a plusieurs fois de suite une valeur acceptable on envoie la valeur
		if(compteur_prox2>FAUX_POSITIF_PROX){
			reception.lateral=prox_tab[PROX2];
		}



		accel_value = get_acc_filtered(Y_AXIS,FAUX_POSITIF_PENTE);

		value=show_inclined(accel_value,calibrate);



		//verification de faux positif
		if(value != reception.inclinaison){
			compteur_inclinaison++;
		}
		else{
			compteur_inclinaison=0;
		}
		//evite l'overflow
		if(compteur_inclinaison>MAX_COMPTEUR){
			compteur_inclinaison=MAX_COMPTEUR;
		}

		if(value != reception.inclinaison && compteur_inclinaison>FAUX_POSITIF_PENTE){
			reception.inclinaison = value;
		}


		chThdSleepUntilWindowed(time, time + MS2ST(10));

    }
}

//renvoie la taille de la ligne dans le fichier move.c
uint16_t get_line_width(void){
	return line_width;
}
//renvoie la position de la ligne dans le fichier move.c
uint16_t get_line_position(void){
	return line_position;
}
//renvoie un compteur dans le fichier move.c qui permet de revenir sur la ligne ? la fin du contournement
uint8_t get_counter_line(void){
	return counter_line;
}
//renvoie dans le fichier move.c la struct qui a les valeurs de proximit? et d'inclinaison
valeurs get_reception(void){
	return reception;
}


void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	chThdCreateStatic(waInfoMode, sizeof(waInfoMode), NORMALPRIO, InfoMode, NULL);
}
