#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			5.0f	//[cm] because of the noise of the camera
#define KP						3.0
#define KI 						0.1f	//must not be zero
#define MAX_SUM_ERROR 			(VITESSE_STABLE/2)

#define TAILLE_LIGNE_MIN 170
#define TAILLE_LIGNE_MAX 400
#define MAX_COMPTEUR 250

#define VALEUR_SENSIBLE_DETECTION_BLACK  55
#define SENSIBILITY_LIGNE 120
#define VITESSE_STABLE 200
#define VITESSE_ROTATION 300
#define SENSIBLE_PROX 240
#define SENSI_GYRO 500
#define TEMPS_ATTENTE 1000
#define FAUX_POSITIF_GYRO 20
#define FAUX_ZERO_GYRO 2
#define FAUX_POSITIF_LIGNE 10

#define SENSOR_FRONT_FRONT_LEFT 7

#define SUIVIT_LIGNE 0
#define CONTOURNEMENT 1
#define SUIVIT_LIGNE_PENTE 2

#define MONTE 1
#define DESCEND -1
#define PLAT 0

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
