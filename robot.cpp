#include "robot.h"

#ifndef IOSTREAM
#define IOSTREAM
#include <iostream>
using namespace std;;
#endif

Robot::Robot(float x, float y, float r):Cercle(x, y, r){
	wg = 0.0f;
	wd = 0.0f;
	enRotation = false;
	rotation_needed = false;
}

float Robot::getDalpha(){
	return Dalpha;
}
		
float Robot::getTheta(Vector u,Vector v){
	return atan2(u.x*v.y-u.y*v.x,u.x*v.x+u.y*v.y);;
}
		
//-------------------------- COS THETHA ---------------------------------
		
float Robot::getCosTheta(Vector u, Vector v){
	return cos(getTheta(u,v));
}
		
//-------------------------- SIN THETHA ---------------------------------
		
float Robot::getSinTheta(Vector u, Vector v){
	return sin(getTheta(u,v));
}//à verifier
		

void Robot::updateDr(){
	Dr = (Dg + Dd)/2;
}
		
void Robot::updateDalpha(){
	Dalpha += (Dg - Dd)/D;
}
		
void Robot::updateDg(){
	Dg = wg * Dt * R0;
}
		
void Robot::updateDd(){
	Dd = wd * Dt * R0;
}
		
void Robot::updateRc(){
	if(Dd == Dg)enRotation = false;
	else{
		Rc = D * (Dd + Dg) / ( 2 * (Dd - Dg) );
		enRotation = true;
	}
}
		
void Robot::updateDx(){
	Dx = Dr*cos(Dalpha);
}
		
void Robot::updateDy(){
	Dy = Dr*sin(Dalpha);
}
		
float Robot::distancePointLine(float x, float y, float a, float b){
	return abs(-a*x + y - b)/sqrt(a*a + 1);
}//NEED'S TO BE AS SEGMENT, NOT AS LINE

float Robot::distance(float x1,float y1,float x2,float y2){
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}
		
		
void Robot::update_auto(Obstacle Tobs[], int nObs,int xG,int yG){
	float nDx, nDy;
	float nwg = wg, nwd = wd;
			
	cout << "update called"   << endl;
			
			
			    
	/**-------------VECTORS U AND V----------------**/
		    
	Vector v(xG-getX(), yG-getY());//vecteur du centre du robot au centre du goal
	Vector minusV(-v.x, -v.y);
	Vector u(getR()*cos(Dalpha), getR()*sin(Dalpha));//vecteur directionnele du robot
        	
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
        	
	Obstacle actual_obs;
        	
	bool inCollision = false;
	int i;
	for(i = 0; i<nObs; i++){
		Vector v1(Tobs[i].getX()-getX(),Tobs[i].getY()-getY());
		if(sqrt((getX() + nDx*500-Tobs[i].getX())*(getX() + nDx*500-Tobs[i].getX())
			+ (getY() + nDy*500-Tobs[i].getY())*(getY() + nDy*500-Tobs[i].getY())) <= 5+getR() + Tobs[i].getR()){
			actual_obs.setX(Tobs[i].getX());
			actual_obs.setY(Tobs[i].getY());
			actual_obs.setR(Tobs[i].getR());
        	inCollision = true;
        	rotation_needed = true;
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
					
			int i;
   			for(i = 0; i<nObs; i++){
	       		Vector v1(Tobs[i].getX()-getX(),Tobs[i].getY()-getY());
	       		if(sqrt((getX() + nDx*500-Tobs[i].getX())*(getX() + nDx*500-Tobs[i].getX())
					+ (getY() + nDy*500-Tobs[i].getY())*(getY() + nDy*500-Tobs[i].getY())) <= 50+getR() + Tobs[i].getR()){
					actual_obs.setX(Tobs[i].getX());
					actual_obs.setY(Tobs[i].getY());
					actual_obs.setR(Tobs[i].getR());
	       			rotation_needed = true;
	       			break;
				}
			}
					
			if(rotation_needed){
				cout << "rotation needed" << endl;
    			Vector k(-getX()+actual_obs.getX(), -getY()+actual_obs.getY());// vecteur du centre du robot a l'obstacle actuel
    			
    			if( (-getTheta(u,k) > 0 && -getTheta(u,k) - M_PI/2 > 0)
				 || (-getTheta(u,k) < 0 && -getTheta(u,k) + M_PI/2 < 0.2)){
				 	cout << "do nothing" << endl;
				 	cout << "			getTheta(u,k) = " << -getTheta(u,k) << endl;
				 	nwg = 1.5;
				 	nwd = 1.5;
				}
    			else if(-getTheta(u,k) > 0 && -getTheta(u,k) < M_PI/2 - 0.05 ) {
    				cout << "turn right 11111111111111111111111111111" << endl;
					cout << "			getTheta(u,k) = " << -getTheta(u,k) << endl;
					if(nwg + 0.05 <= w0Max) nwg +=0.05;
		        	if(nwd - 0.05 > -w0Max) nwd -=0.05;
		        }else if(-getTheta(u,k) > 0 && -getTheta(u,k) > M_PI/2 + 0.05){
		        	cout << "turn left 222222222222222222222222222222" << endl;
		        	cout << "			getTheta(u,k) = " << -getTheta(u,k) << endl;
		        	if(nwg - 0.05 > -w0Max) nwg -=0.01;
		        	if(nwd + 0.05 <= w0Max) nwd +=0.01;
				}
				else if(-getTheta(u,k) < 0 && -getTheta(u,k) < M_PI/2 - 0.05 ) {
					cout << "turn left 33333333333333333333333333" << endl;
					cout << "			getTheta(u,k) = " << -getTheta(u,k) << endl;
					if(nwg - 0.05 > -w0Max) nwg -=0.05;
		        	if(nwd + 0.05 <= w0Max) nwd +=0.05;
		        }else if(-getTheta(u,k) < 0 && -getTheta(u,k) > M_PI/2 + 0.05){
		        	cout << "turn right 44444444444444444444444444444" << endl;
		        	cout << "			getTheta(u,k) = " << -getTheta(u,k) << endl;
		        	if(nwg + 0.05 <= w0Max) nwg +=0.05;
		        	if(nwd - 0.05 > -w0Max) nwd -=0.05;
				}
				if(freeway){
					rotation_needed = false;
				}
			}
			else{
						
				cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!newtheta= "<<theta<<endl;
				if(theta <0.1 && theta > -0.1){
					nwg=1.5;
					nwd=1.5;
					nwg += 0.02;
					nwd += 0.02;
					cout<<"one 1111111111111111111111"<<endl;
				}
				else if(sinTheta > 0 && cosTheta > 0){//tourner a droite
					nwg += 0.05;
					nwd -= 0.05;
					cout<<"two 22222222222222222222 "<<endl;	
				}
				else if(sinTheta > 0 && cosTheta <= 0){//tourner a gauche
					nwg += 0.05;
					nwd -= 0.05;
					cout<<"three 333333333333333333333"<<endl;	
				}
				else if(sinTheta <= 0 && cosTheta <= 0){//tourner a gauche
					nwg -= 0.05;
					nwd += 0.05;
					cout<<"four 4444444444444444444444"<<endl;	
				}
				else if(sinTheta <= 0 && cosTheta > 0){//tourner a droite
					nwg -= 0.05;
					nwd += 0.05;
					cout<<"five 55555555555555555555555"<<endl;	
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
		
    }
    else{
    	cout<<"collision"<<endl;
        rotation_needed = true;
		inCollision=false;
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
	cout << "wd = " << wd << "\n";
	cout << "wg = " << wg << "\n";
}
		
void Robot::update_manual(Obstacle Tobs[], int nObs){
			
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
	
	cout << "wd = " << wd << "\n";
	cout << "wg = " << wg << "\n";
}
