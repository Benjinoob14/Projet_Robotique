#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <math.h>
#include <camera/po8030.h>
#include <sensors/proximity.h>
#include <process_image.h>
#include <sensors/imu.h>
#include <leds.h>


static uint16_t black_line = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint16_t proxii=0;
static uint8_t compteur_liigne=0;
static bool inclined = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//fonction qui calcule pour savoir si le robot est en pente
bool show_inclined(int16_t *accel_values){

    //threshold value to not use the leds when the robot is too horizontal
    float threshold = SENSI_GYRO;
    //create a pointer to the array for shorter name
    float *accel = accel_values;


    if(fabs(accel[X_AXIS]) > threshold || fabs(accel[Y_AXIS]) > threshold){

    	//clock wise angle in rad with 0 being the back of the e-puck2 (Y axis of the IMU)
        float angle = atan2(accel[X_AXIS], accel[Y_AXIS]);

        //rotates the angle by 22.5 degrees (simpler to compare with PI and PI/2 than with 5*PI/4)
        angle += M_PI/4;

        //if the angle is greater than PI, then it has shifted on the -PI side of the quadrant
        //so we correct it
        if(angle > M_PI){
            angle = -2 * M_PI + angle;
        }

//        chprintf((BaseSequentialStream *)&SDU1, "swag=  %d  ",angle);

        if(angle>0){
        	 return 1;
        }
        else{
        	return 0;
        }
    }

    return 0;

}
/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
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

	//s'il voit une suite de pixels noir il incrémente le compteur qui permet de savoir que le robot est revenu sur une ligne noir
	if(buffer[IMAGE_BUFFER_SIZE/2]< VALEUR_SENSIBLE_DETECTION_BLACK){
		compteur_liigne++;
	}
	if(buffer[IMAGE_BUFFER_SIZE/2]> VALEUR_SENSIBLE_DETECTION_BLACK || compteur_liigne>MAX_COMPTEUR){
		compteur_liigne=0;
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
		black_line=extract_line_width(image);


		if (black_line<MIN_LINE_WIDTH || black_line>10*MIN_LINE_WIDTH){
			black_line=0;
		}



		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}

static THD_WORKING_AREA(waMode, 512);
static THD_FUNCTION(Mode, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t accel_values[3] = {0};
    systime_t time;
    bool value=0;
    uint16_t compteur=0;

    while(1){
    	time = chVTGetSystemTime();

		proxii=get_prox(SENSOR_FRONT_FRONT_LEFT);

		get_acc_all(accel_values);

//		get_gyro_all(gyro_values);

		value=show_inclined(accel_values);

//		chprintf((BaseSequentialStream *)&SDU1, "angle= %d ",value);

		if(value != inclined){
			compteur++;
		}
		else{
			compteur=0;
		}
		if(value != inclined && compteur>FAUX_POSITIF_GYRO){
			inclined = !inclined;
		}

//		chprintf((BaseSequentialStream *)&SDU1, "%d",inclined);


		chThdSleepUntilWindowed(time, time + MS2ST(10));

    }
}

uint16_t get_black_line(void){
	return black_line;
}

uint16_t get_line_position(void){
	return line_position;
}

uint8_t get_compteur_liigne(void){
	return compteur_liigne;
}

uint8_t get_proxii(void){
	return proxii;
}

bool get_inclined(void){
	return inclined;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	chThdCreateStatic(waMode, sizeof(waMode), NORMALPRIO, Mode, NULL);
}
