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
#define PXTOCM					1570.0f //experimental value
#define ERROR_THRESHOLD			5.0f

#define TAILLE_LIGNE_MIN 	170
#define TAILLE_LIGNE_MAX 	400
#define MAX_COMPTEUR 		250
#define MAX_PROXI			10000

//***les valeurs ci-dessous ont été trouvé de manière experimentale****

//valeurs pour le PID
#define KP						3.0f
#define KI 						0.1f	//must not be zero
#define KD						1.5f
#define MAX_SUM_ERROR 			VITESSE_STABLE_PLAT/(4*KI)

//liste des differentes vitesses
#define VITESSE_STABLE_PLAT 			 200
#define VITESSE_STABLE_PENTE 			 300
#define VITESSE_ROTATION 			     300
#define VITESSE_ROTATION_REPLACEMENT	 150
#define VITESSE_VIRAGE_ROUE_EXT			 300
#define VITESSE_VIRAGE_ROUE_INT			 150

//toutes les sensibilitées paramétré experimentalement
#define VALEUR_SENSIBLE_DETECTION_BLACK  55
#define SENSIBILITY_LIGNE 				 100
#define SENSIBLE_PROX_FRONT 			 500
#define SENSIBLE_PROX_LEFT				 300
#define SENSI_PENTE 					 1000

//liste des attentes en fonction du besoins
#define TEMPS_ATTENTE 			 		 1000
#define TEMPS_ATTENTE_ROT				 1.3*TEMPS_ATTENTE
#define TEMPS_ATTENTE_REBOND			 0.8*TEMPS_ATTENTE
#define MINI_ATTENTE					 400
#define MINUSCULE_ATTENTE				 100

//permet la verification en serie pour etre sûr d avoir une bonne valeurs
#define FAUX_POSITIF_PENTE 				 5
#define FAUX_POSITIF_PROX 				 4
#define FAUX_POSITIF_LIGNE 				 20
#define FAUX_POSITIF_REPLACEMENT		 3

#define SENSOR_FRONT		     7
#define SENSOR_FRONT_LEFT		 6
#define PROX1					 0
#define PROX2					 1

#define INTENSITY				 10

//la struct qui a les valeurs des detecteurs de proxi et l'inclinaison
typedef struct {
	uint16_t frontal;
	uint16_t lateral;
	int8_t inclinaison;
} valeurs;

//la liste de tout les modes
typedef enum {
	ARRET,
	SUIVI_LIGNE,
	SUIVI_LIGNE_PENTE_MONTEE,
	SUIVI_LIGNE_PENTE_DESCENTE,
	DEBUT_CONTOURNEMENT,
	MILIEU_CONTOURNEMENT,
	FIN_CONTOURNEMENT,
} choose_mode_t;

#define MONTEE 					 1
#define DESCENTE				-1
#define PLAT 					 0

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
