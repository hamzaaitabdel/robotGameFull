#ifndef ROBOT_H
#define ROBOT_H
#include <cmath>
#include <iostream>
#include <windows.h>
#include <cmath>
using namespace std;
struct Vector{
	float x;
	float y;
	Vector(float x0,float y0){
		x=x0;
		y=y0;
	}
};
/**
struct LinearEquation{
	float a;
	float b;
	LinearEquation(float a,float b){
		this->a = a;
		this->b = b;
	}
};
**/
class Cercle{
	float x , y, r;
	public:
		Cercle(float x = 400.0f, float y = 500.0f, float r = 50.0f){
			this->x = x;
			this->y = y;
			this->r = r;
		}
		float getX(){
			return x;
		}
		void setX(float x0){
			x = x0;
		}
		float getY(){
			return y;
		}
		void setY(float y0){
			y = y0;
		}
		float getR(){
			return r;
		}
		void setR(float r0){
			r = r0;
		}
		
};

class Obstacle: public Cercle{
	public:
		Obstacle(float x = 0, float y = 0, float r = 10):Cercle(x, y, r){}
};

class Robot : public Cercle{
	
	public:
			float getDalpha(){
			return Dalpha;
		}
		
		float getTheta(Vector u,Vector v){
			return atan2(u.x*v.y-u.y*v.x,u.x*v.x+u.y*v.y);;
		}
		
		//-------------------------- COS THETHA ---------------------------------
		
		float getCosTheta(Vector u, Vector v){
			return cos(getTheta(u,v));
		}
		
		//-------------------------- SIN THETHA ---------------------------------
		
		float getSinTheta(Vector u, Vector v){
			return sin(getTheta(u,v));
		}//à verifier
		
		Robot(float x = 900.0f, float y = 300.0f, float r = 50.0f):Cercle(x, y, r){}
		
		
		/***************LES CONSTANTES****************/
		
		const float Dt = 0.1f; 			// Intervale de temps entre chaque mise à jour de la position et l'orientation du robot Dt = 1 s
		const float D = 0.05f; 			// Distance entre les roues D = 0.05 m
		const float Rr = 0.05f; 		// Rayon du robot Rr = 0.05 m
		const float R0 = 0.02f; 		// Rayon d'une roue R0 = 0.02 m
		const float w0Max = 10.0f; 		// Vitesse angulaire maximum des roues w0Max = 10 rad/s
		const float Dw0Max = 2.0f; 	// Acceleration angulaire maximum des roues Dw0Max = 2 rad/s²
		
		/***************LES VARIABLES****************/
		
		float Dr;		//Distance parcourue pendant Dt en m
		float Dalpha;	//Reorientation pendant Dt en rad
		float wg = 0.0f;	//Vitesse angulaire de la roue gauche en rad/s
		float wd = 0.0f;	//Vitesse angulaire de la roue droite en rad/s
		float Dg;		//Distance parcourue par la roue gauche pendant Dt en m
		float Dd;		//Distance parcourue par la roue droite pendant Dt en m
		float Rc; 		//Rayon de courbure de la trajectoire du robot en m
		
		
		//acceleration angulaire des deux roues gauche et droite
		
		
		bool enRotation = false;
		
		float Dx, Dy;		//Distance instantanee du robot pendant Dt de x et y en pixel/s
		
		
		/***************LES METHODES****************/
		
		void updateDr(){
			Dr = (Dg + Dd)/2;
		}
		
		void updateDalpha(){
			Dalpha += (Dg - Dd)/D;
		}
		
		void updateDg(){
			Dg = wg * Dt * R0;
		}
		
		void updateDd(){
			Dd = wd * Dt * R0;
		}
		
		void updateRc(){
			if(Dd == Dg)enRotation = false;
			else{
				Rc = D * (Dd + Dg) / ( 2 * (Dd - Dg) );
				enRotation = true;
			}
		}
		
		void updateDx(){
			Dx = Dr*cos(Dalpha);
		}
		
		void updateDy(){
			Dy = Dr*sin(Dalpha);
		}
		
		float distancePointLine(float x, float y, float a, float b){
			return abs(-a*x + y - b)/sqrt(a*a + 1);
		}//NEED'S TO BE AS SEGMENT, NOT AS LINE
		float distance(float x1,float y1,float x2,float y2){
			return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
		}
		
		
		void update_auto(Obstacle Tobs[], int nObs,int xG,int yG){
			float nDx, nDy;
			float nwg = wg, nwd = wd;
			
			cout << "update called"   << endl;
			
			
			    
		    /**-------------VECTORS U AND V----------------**/
		    
			Vector v(xG-getX(), yG-getY());
			Vector minusV(-v.x, -v.y);
        	Vector u(getR()*cos(Dalpha), getR()*sin(Dalpha));
        	
        	Vector goalObs(0.0f,0.0f);
        	
        	
        	float a = v.y/v.x;
        	float b = getY() - a*getX();//f(x) = ax + b
        	//LinearEquation D1(a, b);
			//-------------------------------
        	
			//line(getX(),getY(),getX()+u.x,getY()+u.y);
			//line(getX(),getY(),getX()+v.x,getY()+v.y);
			float theta=getTheta(u,v);
			float cosTheta = getCosTheta(u, v);
			float sinTheta = getSinTheta(u, v);
			
			cout<<"                     newtheta= "<<theta<<endl;
			cout<<"                     newcosTheta= "<<cosTheta<<endl;
			cout<<"                     newsinTheta= "<<sinTheta<<endl;
			
				
        	nDx = Dr*cos(Dalpha + ((nwg * Dt * R0) - (nwd * Dt * R0))/D);
        	nDy = Dr*sin(Dalpha + ((nwg * Dt * R0) - (nwd * Dt * R0))/D);
        	
        	bool inCollision = false;
        	int i;
        	for(i = 0; i<nObs; i++){
        		Vector v1(Tobs[i].getX()-getX(),Tobs[i].getY()-getY());
        		if(sqrt((getX() + nDx*500-Tobs[i].getX())*(getX() + nDx*500-Tobs[i].getX())
					+ (getY() + nDy*500-Tobs[i].getY())*(getY() + nDy*500-Tobs[i].getY())) <= 3+getR() + Tobs[i].getR()){
				
        			inCollision = true;
        			break;
				}
			}	
			bool freeway = true;
			for(i = 0; i<nObs; i++){
        		if(distancePointLine(Tobs[i].getX(),Tobs[i].getY(),a,b) <= Tobs[i].getR() + getR()){
        			if(distance(getX(),getY(),xG+.0f,yG+.0f)>distance(Tobs[i].getX(),Tobs[i].getY(),xG+.0f,yG+.0f)){
						freeway = false;
        				break;
					}
        			
				}
			}
			
			goalObs.x = -xG+Tobs[i].getX();
			goalObs.y = -yG+Tobs[i].getY();
			
			
					
			if(!inCollision){
				if(!freeway){
					cout << "!!!!!!!!!!!!!!not FREE!!!FREE-------------FREE"<<endl;
					cout<<"--------------->"<<getTheta(v, goalObs)<<endl;
				if(getTheta(v, goalObs)<0){
					if(theta + M_PI/2 > 0.5){
						nwg += 0.05;
						nwd -= 0.05;
						cout<<"wa7d*"<<endl;
					}
					else if(theta + M_PI/2 < -0.5){
						nwg -= 0.05;
						nwd += 0.05;
						cout<<"joj*"<<endl;
					}
					else{
						nwg=1.5;
						nwd=1.5;
						nwg += 0.02;
						nwd += 0.02;
						cout<<"tlata*"<<endl;
					}
				}else{
					
					if(theta - M_PI/2 <-0.5){
						nwg -= 0.05;
						nwd += 0.05;
						cout<<"wa7d"<<endl;
					}
					else if(theta - M_PI <-0.5){
						nwg += 0.05;
						nwd -= 0.05;
						cout<<"joj"<<endl;
					}
					else{
						nwg=1.5;
						nwd=1.5;
						nwg += 0.02;
						nwd += 0.02;
						cout<<"tlata"<<endl;
					}
					
					
					
				}
			}else{
			cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!newtheta= "<<theta<<endl;
				if(theta <0.1 && theta > -0.1){
						nwg=1.5;
						nwd=1.5;
						nwg += 0.02;
						nwd += 0.02;
						
						cout<<"one"<<endl;
					}
					else if(sinTheta > 0 && cosTheta > 0){//tourner a droite
					nwg += 0.05;
					nwd -= 0.05;
					cout<<"two"<<endl;	
					}
					else if(sinTheta > 0 && cosTheta <= 0){//tourner a gauche
					nwg += 0.05;
					nwd -= 0.05;
					cout<<"three"<<endl;	
					}
					else if(sinTheta <= 0 && cosTheta <= 0){//tourner a gauche
					nwg -= 0.05;
					nwd += 0.05;
					cout<<"four"<<endl;	
					}
					else if(sinTheta <= 0 && cosTheta > 0){//tourner a droite
					nwg -= 0.05;
					nwd += 0.05;
					cout<<"five"<<endl;	
					}
				}
				Dx = nDx;
				Dy = nDy;
				wg = nwg;
				wd = nwd;
        		updateDd();
        		cout << "Dd = " << Dd << "\n";
        		updateDg();
        		cout << "Dg = " << Dg << "\n";
				updateDr();
				cout << "Dr = " << Dr << "\n";
        		updateDalpha();
        		cout << "Dalpha = " << Dalpha << "\n";
        		updateRc();
        		cout << "Rc = " << Rc << "\n";//Not a Number --> NaN
        		updateDx();
        		cout << "Dx = " << Dx << "\n";
        		updateDy();
        		cout << "Dy = " << Dy << "\n";
        		setX(getX() + Dx*500);
        		cout << "x = " << getX() << "\n";
        		setY(getY() + Dy*500);
        		cout << "y = " << getY() << "\n";
        	}
        	else{
        		cout<<"                           collicion"<<endl;
        		if(nwg + 0.05 <= w0Max) nwg +=0.05;
        	    Dx = nDx;
				Dy = nDy;
				wg = nwg;
				wd = nwd;
        		updateDd();
        		cout << "Dd = " << Dd << "\n";
        		updateDg();
        		cout << "Dg = " << Dg << "\n";
				updateDr();
				cout << "Dr = " << Dr << "\n";
        		updateDalpha();
        		cout << "Dalpha = " << Dalpha << "\n";
        		updateRc();
        		cout << "Rc = " << Rc << "\n";//Not a Number --> NaN
        		updateDx();
        		cout << "Dx = " << Dx << "\n";
        		updateDy();
        		cout << "Dy = " << Dy << "\n";
        		//setX(getX() + Dx*500);
        		cout << "x = " << getX() << "\n";
        		//setY(getY() + Dy*500);
        		cout << "y = " << getY() << "\n";
        		//wg -= 0.02;
			   // wd -= 0.02;
			   inCollision=false;
			}
			
			cout << "wd = " << wd << "\n";
			cout << "wg = " << wg << "\n";
		}
		
		void update_manual(Obstacle Tobs[], int nObs){
			
			float nDx, nDy;
			float nwg = wg, nwd = wd;
			
			
			cout << "update called"   << endl;
		
			if(GetKeyState(VK_UP) & 0x8000) {
				cout << "Up key. \n " << endl;
				if(nwg + 0.05 <= w0Max) nwg +=0.05;
				if(nwd + 0.05 <= w0Max) nwd +=0.05;
			}
            else if(GetKeyState(VK_LEFT) & 0x8000) {
				cout << "Left key. \n "   << endl;
				if(nwg - 0.05 >= -w0Max && nwd + 0.05 <= w0Max){	
					nwg -= 0.05;
					nwd += 0.05;
				}
			}
            else if(GetKeyState(VK_RIGHT) & 0x8000) {
				cout << "Right key. \n "  << endl;
				if(nwg - 0.05 >= -w0Max && nwd + 0.05 <= w0Max){	
					nwg += 0.05;
					nwd -= 0.05;
				}
			}
            else if(GetKeyState(VK_DOWN) & 0x8000) {
			    cout << "Down key. \n "   << endl;
				if(nwg - 0.05 >= -w0Max) nwg -=0.05;
				if(nwd - 0.05 >= -w0Max) nwd -=0.05;
			}
            else{
            	
            	if(nwg>nwd){
            		nwg-=0.025;
            		nwd+=0.025;
				}
				if(nwg<nwd){
            		nwd-=0.025;	
            		nwg+=0.025;
				}
            	//nwg -=0.05;
            	//nwd -=0.05;
				cout << "Unknown extended key. \n" << endl;
        	}
        	
        	nDx = Dr*cos(Dalpha + ((nwg * Dt * R0) - (nwd * Dt * R0))/D);
        	nDy = Dr*sin(Dalpha + ((nwg * Dt * R0) - (nwd * Dt * R0))/D);
        	bool inCollision = false;
        	for(int i = 0; i<nObs; i++){
        		if(sqrt((getX() + nDx*500-Tobs[i].getX())*(getX() + nDx*500-Tobs[i].getX())
					+ (getY() + nDy*500-Tobs[i].getY())*(getY() + nDy*500-Tobs[i].getY())) <= 4+getR() + Tobs[i].getR()){
        			inCollision = true;
        			break;
				}
			}
        	
			if(!inCollision){
				Dx = nDx;
				Dy = nDy;
				wg = nwg;
				wd = nwd;
        		updateDd();
        		cout << "Dd = " << Dd << "\n";
        		updateDg();
        		cout << "Dg = " << Dg << "\n";
				updateDr();
				cout << "Dr = " << Dr << "\n";
        		updateDalpha();
        		cout << "Dalpha = " << Dalpha << "\n";
        		updateRc();
        		cout << "Rc = " << Rc << "\n";//Not a Number --> NaN
        		updateDx();
        		cout << "Dx = " << Dx << "\n";
        		updateDy();
        		cout << "Dy = " << Dy << "\n";
        		setX(getX() + Dx*500);
        		cout << "x = " << getX() << "\n";
        		setY(getY() + Dy*500);
        		cout << "y = " << getY() << "\n";
        	}
        	else{
        	    Dx = nDx;
				Dy = nDy;
				wg = nwg;
				wd = nwd;
        		updateDd();
        		cout << "Dd = " << Dd << "\n";
        		updateDg();
        		cout << "Dg = " << Dg << "\n";
				updateDr();
				cout << "Dr = " << Dr << "\n";
        		updateDalpha();
        		cout << "Dalpha = " << Dalpha << "\n";
        		updateRc();
        		cout << "Rc = " << Rc << "\n";//Not a Number --> NaN
        		updateDx();
        		cout << "Dx = " << Dx << "\n";
        		updateDy();
        		cout << "Dy = " << Dy << "\n";
        		//setX(getX() + Dx*500);
        		cout << "x = " << getX() << "\n";
        		//setY(getY() + Dy*500);
        		cout << "y = " << getY() << "\n";
        		wg -= 0.02;
			    wd -= 0.02;
			}
			/*
			if(wg>=0.25&&wd>=0.25){
				wg -= 0.02;
			    wd -= 0.02;
			}
			*/
			cout << "wd = " << wd << "\n";
			cout << "wg = " << wg << "\n";
		}
		
		
		
		//-------------------------GETTERS ET SETTERS-----------------------
	
		
};

#endif
