#ifndef ROBOT_H
#define ROBOT_H
#include <cmath>
#include <windows.h>
#include <cmath>

#include "cercle.h"
#include "obstacle.h"
#include "vector.h"



class Robot : public Cercle{
	
	/***************LES CONSTANTES****************/
		
	static const float Dt = 0.1f; 			// Intervale de temps entre chaque mise à jour de la position et l'orientation du robot Dt = 1 s
	static const float D = 0.05f; 			// Distance entre les roues D = 0.05 m
	static const float Rr = 0.05f; 		// Rayon du robot Rr = 0.05 m
	static const float R0 = 0.02f; 		// Rayon d'une roue R0 = 0.02 m
	static const float w0Max = 10.0f; 		// Vitesse angulaire maximum des roues w0Max = 10 rad/s
	static const float Dw0Max = 2.0f; 	// Acceleration angulaire maximum des roues Dw0Max = 2 rad/s²
	
	/***************LES VARIABLES****************/
	
	float Dr;		//Distance parcourue pendant Dt en m
	float Dalpha;	//Reorientation pendant Dt en rad
	float wg;		//Vitesse angulaire de la roue gauche en rad/s
	float wd;		//Vitesse angulaire de la roue droite en rad/s
	float Dg;		//Distance parcourue par la roue gauche pendant Dt en m
	float Dd;		//Distance parcourue par la roue droite pendant Dt en m
	float Rc; 		//Rayon de courbure de la trajectoire du robot en m
	
	
	//acceleration angulaire des deux roues gauche et droite
	
	bool enRotation;
	bool rotation_needed;
	
	float Dx, Dy;		//Distance instantanee du robot pendant Dt de x et y en pixel/s
	
	public:
		
		Robot(float = 900.0f, float = 300.0f, float = 50.0f);
		
		//-------------------------GETTERS-----------------------
		
		float getDalpha();
		
		float getTheta(Vector,Vector);
		
		//-------------------------- COS THETHA ---------------------------------
		
		float getCosTheta(Vector, Vector);
		
		//-------------------------- SIN THETHA ---------------------------------
		
		float getSinTheta(Vector, Vector);
		
		/***************LES METHODES****************/
		
		void updateDr();
		
		void updateDalpha();
			
		void updateDg();
			
		void updateDd();
			
		void updateRc();
			
		void updateDx();
			
		void updateDy();
			
		float distancePointLine(float, float, float, float);
		
		float distance(float,float,float,float);
		
		// ---------------------------UPDATES-------------------------------
			
		void update_auto(Obstacle Tobs[], int nObs, int xG, int yG);
		
		void update_manual(Obstacle Tobs[], int nObs);
		
		
	
		
};

#endif
