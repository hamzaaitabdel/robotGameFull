#include "cercle.h"

Cercle::Cercle(float x = 400.0f, float y = 500.0f, float r = 50.0f){
	this->x = x;
	this->y = y;
	this->r = r;
}

float Cercle::getX(){
	return x;
}
void Cercle::setX(float x0){
	x = x0;
}
float Cercle::getY(){
	return y;
}
void Cercle::setY(float y0){
	y = y0;
}
float Cercle::getR(){
	return r;
}
void Cercle::setR(float r0){
	r = r0;
}
