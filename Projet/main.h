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
#define ERROR_THRESHOLD			5.0f
#define KP						3.0f
#define KI 						0.1f	//must not be zero
#define KD						1.5f
#define MAX_SUM_ERROR 			VITESSE_STABLE_PLAT

#define TAILLE_LIGNE_MIN 	170
#define TAILLE_LIGNE_MAX 	400
#define MAX_COMPTEUR 		250

#define VALEUR_SENSIBLE_DETECTION_BLACK  55
#define SENSIBILITY_LIGNE 				 60
#define VITESSE_STABLE_PLAT 			 200
#define VITESSE_STABLE_PENTE 			 300
#define VITESSE_ROTATION 			     300

#define SENSIBLE_PROX 					 400
#define SENSI_GYRO 						 500

#define TEMPS_ATTENTE 			 		 1000
#define FAUX_POSITIF_GYRO 				 20
#define FAUX_POSITIF_PROX 				 10
#define FAUX_POSITIF_LIGNE 				 10

#define SENSOR_FRONT_FRONT_LEFT  7

#define SENSOR_FRONT_LEFT		 6

typedef enum {
	ARRET,
	SUIVI_LIGNE,
	SUIVI_LIGNE_PENTE,
	DEBUT_CONTOURNEMENT,
	MILIEU_CONTOURNEMENT,
	FIN_CONTOURNEMENT,
} choose_mode_t;

#define MONTE 					 1
#define DESCEND 				-1
#define PLAT 					 0

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
