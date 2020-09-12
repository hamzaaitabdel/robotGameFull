//TODO when count ==100 some fuck happend
#include <graphics.h>
#include <vector>
#ifndef IOSTREAM
#define IOSTREAM
#include <iostream>
using namespace std;;
#endif
#include<fstream>
#include<cstdlib>
#define M_PI 3.14159265358979323846
#include "robot.h"
#include "obstacle.h"



Obstacle Tobs[15];//Tableau des obstacles
//int lastPosX[1000];
//int lastPosY[1000];
int nObs = 0;
int posCount=0;
int xG=350;
int yG=500;
//The straight line passes by Robot and Goal
float a,b;
void scanner(char fileName[50]){//
    ifstream bucky;
    cout << "18, fileName = " << fileName << endl;
	//cin.getline(fileName,50);
	cout << "20" << endl;
   	bucky.open(fileName);
   	cout << "22" << endl;
    if(!bucky.is_open()){
    	cout << "24" << endl;
       	exit(EXIT_FAILURE);
    }
    int input;
	std::vector<int> elements;
	while(bucky >> input){
		elements.push_back(input);
	}
    for(int i = 0; i<elements.size()-2; i+=3){//here
		Tobs[i/3].setX(elements[i]);//0/3 = 0, 1/3 = 0, 2/3 = 0 c'est ce qu'on veut
		cout << "Element " << i << " = " << elements[i] << endl;
		Tobs[i/3].setY(elements[i+1]);
		cout << "Element " << i+1 << " = " << elements[i+1] << endl;
		Tobs[i/3].setR(elements[i+2]);
		cout << "Element " << i+2 << " = " << elements[i+2] << endl;
		nObs++;
	}
}

int main(void){
	char path[] = ".\\obstacles.obs";
	scanner(path);
	float scale=1.0f;
	initwindow(1230,1000,"DifferencialRobot"); 
	
	
	float alpha;
	Robot robot;
	

	 
	moveto(0, 0);
    settextstyle(8, 0, 5);
    
    char automatic[] = "AUTOMATIQUE";
    char manual[] = "MANUELLE";
    setactivepage(0);//AUTOMATIQUE
    settextjustify(CENTER_TEXT, TOP_TEXT);
	outtextxy(getmaxx()/2, getmaxy()/2 -100, automatic);
    outtextxy(getmaxx()/2, getmaxy()/2 +100, manual);
    rectangle(getmaxx()/2 - 200, getmaxy()/2 -100, getmaxx()/2 + 200, getmaxy()/2 -50);
    
    setactivepage(1);//MANUELLE
	outtextxy(getmaxx()/2, getmaxy()/2 -100, automatic);
    outtextxy(getmaxx()/2, getmaxy()/2 +100, manual);
    rectangle(getmaxx()/2 - 200, getmaxy()/2 +100, getmaxx()/2 + 200, getmaxy()/2 +150);
    
    int user_input = -1;
    bool user_select = false;
    setvisualpage(0);
    int bug_counter = 0;
    cout<<"not ended " << bug_counter++ <<endl;
    while(user_input == -1){
    	delay(10);
    	cout<<"not ended " << bug_counter++ <<endl;
    	if(GetKeyState(VK_UP) & 0x8000){
    		setvisualpage(0);
    		user_select = false;
		}
		else if(GetKeyState(VK_DOWN) & 0x8000){
			setvisualpage(1);
			user_select = true;
		}
		else if(GetKeyState(VK_SPACE) | GetKeyState(VK_RETURN) & 0x8000){
			if(user_select){
				user_input = 0;//MANUELLE
			}
			else{
				user_input = 1;//AUTOMATIQUE
			}
		}
	}
    
    while(user_input == 1 && robot.distance(robot.getX(), robot.getY(), xG, yG) > robot.getR()+100)
    {
		setactivepage(1); 
		/*
		if(!(lastPosX[posCount-1]==robot.getX()&&lastPosY[posCount-1]==robot.getY())&& posCount%5==0){
			lastPosX[posCount]=robot.getX();
        	lastPosY[posCount]=robot.getY();
		}
		*/
        posCount++;
        cout<<"count pos= "<<posCount<<endl;
        alpha = robot.getDalpha();
		//TODO save old trajectory
        cleardevice();
        
		setcolor(YELLOW);
        circle(xG,yG,100);//THE GOAL
        //line(robot.getX(),robot.getY(),xG,yG);
        
        setcolor(3);
        circle(robot.getX(),robot.getY(),robot.getR());
        /**
          *----------------------TRIANGLE DANS LE CERCLE----------------------------
		***/
		
        line(robot.getX() + (int)robot.getR()*cos(alpha - (M_PI/2)),
		robot.getY()+ (int)robot.getR()*sin(alpha - (M_PI/2)), 
		robot.getX() + (int)robot.getR()*cos(alpha),
		robot.getY()+ (int)robot.getR()*sin(alpha));
		
        line(robot.getX() + (int)robot.getR()*cos(alpha), 
		robot.getY()+ (int)robot.getR()*sin(alpha), 
		robot.getX() + (int)robot.getR()*cos(alpha + (M_PI/2)), 
		robot.getY()+ (int)robot.getR()*sin(alpha + (M_PI/2)));
		
        line(robot.getX() + (int)robot.getR()*cos(alpha - (M_PI/2)), 
		robot.getY()+ (int)robot.getR()*sin(alpha - (M_PI/2)), 
		robot.getX() + (int)robot.getR()*cos(alpha + (M_PI/2)), 
		robot.getY()+ (int)robot.getR()*sin(alpha + (M_PI/2)));
		
		
    	
    	/**
          *----------------------------LES OBSTACLES--------------------------------
		***/
		for(int u = 0; u<nObs; u++){
			cout << "OBS = (" << Tobs[u].getX() << ", " << Tobs[u].getY() << ", " << Tobs[u].getR() << ")\n";
			circle(Tobs[u].getX(),Tobs[u].getY(),Tobs[u].getR());
		}
        /*
        setcolor(GREEN);
        setfillstyle(1,GREEN);
        for(int j=0;j<posCount;j++){
        circle(lastPosX[j],lastPosY[j],2);	
        floodfill(lastPosX[j],lastPosY[j],GREEN);
		}
		*/
		setvisualpage(1);
        delay(100);
        robot.update_auto(Tobs, nObs,xG,yG);
        /////////////////////////////////////////////
     	setactivepage(0); 
		/*
		if(!(lastPosX[posCount-1]==robot.getX()&&lastPosY[posCount-1]==robot.getY())&& posCount%5==0){
			lastPosX[posCount]=robot.getX();
        	lastPosY[posCount]=robot.getY();
		}
        */
		posCount++;
        cout<<"count pos= "<<posCount<<endl;
        alpha = robot.getDalpha();
		//TODO save old trajectory
        cleardevice();
        
		setcolor(YELLOW);
        circle(xG,yG,100);//THE GOAL
        //line(robot.getX(),robot.getY(),xG,yG);
        
        setcolor(3);
        circle(robot.getX(),robot.getY(),robot.getR());
        /**
          *----------------------TRIANGLE DANS LE CERCLE----------------------------
		***/
		
        line(robot.getX() + (int)robot.getR()*cos(alpha - (M_PI/2)),
		robot.getY()+ (int)robot.getR()*sin(alpha - (M_PI/2)), 
		robot.getX() + (int)robot.getR()*cos(alpha),
		robot.getY()+ (int)robot.getR()*sin(alpha));
		
        line(robot.getX() + (int)robot.getR()*cos(alpha), 
		robot.getY()+ (int)robot.getR()*sin(alpha), 
		robot.getX() + (int)robot.getR()*cos(alpha + (M_PI/2)), 
		robot.getY()+ (int)robot.getR()*sin(alpha + (M_PI/2)));
		
        line(robot.getX() + (int)robot.getR()*cos(alpha - (M_PI/2)), 
		robot.getY()+ (int)robot.getR()*sin(alpha - (M_PI/2)), 
		robot.getX() + (int)robot.getR()*cos(alpha + (M_PI/2)), 
		robot.getY()+ (int)robot.getR()*sin(alpha + (M_PI/2)));
		
		
    	
    	/**
          *----------------------------LES OBSTACLES--------------------------------
		***/
		for(int u = 0; u<nObs; u++){
			cout << "OBS = (" << Tobs[u].getX() << ", " << Tobs[u].getY() << ", " << Tobs[u].getR() << ")\n";
			circle(Tobs[u].getX(),Tobs[u].getY(),Tobs[u].getR());
		}
        //circle(xo,yo,ro);
        /*
		setcolor(GREEN);
        setfillstyle(1,GREEN);
        for(int j=0;j<posCount;j++){
        circle(lastPosX[j],lastPosY[j],2);	
        floodfill(lastPosX[j],lastPosY[j],GREEN);
		}
		*/
		setvisualpage(0);
        delay(100);
        robot.update_auto(Tobs, nObs,xG,yG);
        
        
       
    }
    
    char you_win[] = "OBJECTIF ATTEIND!!!";
    
	if(user_input == 1){
		outtextxy(getmaxx()/2, getmaxy()/2 -100, you_win);
		delay(3000);
    
	}
    
    
    while(user_input == 0 && robot.distance(robot.getX(), robot.getY(), xG, yG) > robot.getR()+100)
    {
    	cout << "dddddddd"<< endl;
        robot.update_manual(Tobs, nObs);
        /*
		if(!(lastPosX[posCount-1]==robot.getX()&&lastPosY[posCount-1]==robot.getY())&& posCount%5==0){
			lastPosX[posCount]=robot.getX();
        lastPosY[posCount]=robot.getY();
		}
		*/
        posCount++;
        cout<<"count pos= "<<posCount<<endl;
        alpha = robot.getDalpha();
		//TODO save old trajectory
        cleardevice();
        //outtextxy(100, 0,t);
        //lineto(600,600);
        setcolor(3);
        circle(robot.getX(),robot.getY(),robot.getR());
        /**
          *----------------------TRIANGLE DANS LE CERCLE----------------------------
		***/
        line(robot.getX() + (int)robot.getR()*cos(alpha - (M_PI/2)), robot.getY()+ (int)robot.getR()*sin(alpha - (M_PI/2)), robot.getX() + (int)robot.getR()*cos(alpha), robot.getY()+ (int)robot.getR()*sin(alpha));
        line(robot.getX() + (int)robot.getR()*cos(alpha), robot.getY()+ (int)robot.getR()*sin(alpha), robot.getX() + (int)robot.getR()*cos(alpha + (M_PI/2)), robot.getY()+ (int)robot.getR()*sin(alpha + (M_PI/2)));
        line(robot.getX() + (int)robot.getR()*cos(alpha - (M_PI/2)), robot.getY()+ (int)robot.getR()*sin(alpha - (M_PI/2)), robot.getX() + (int)robot.getR()*cos(alpha + (M_PI/2)), robot.getY()+ (int)robot.getR()*sin(alpha + (M_PI/2)));
    	
    	/**
          *----------------------------LES OBSTACLES--------------------------------
		***/
		setcolor(YELLOW);
        circle(xG,yG,100);//THE GOAL
		setcolor(RED);
		for(int u = 0; u<nObs; u++){
			cout << "OBS = (" << Tobs[u].getX() << ", " << Tobs[u].getY() << ", " << Tobs[u].getR() << ")\n";
			circle(Tobs[u].getX(),Tobs[u].getY(),Tobs[u].getR());
		}
        //circle(xo,yo,ro);
        
        delay(100);
        /*
        setcolor(GREEN);
        for(int j=0;j<posCount;j++){
        circle(lastPosX[j],lastPosY[j],2);	
		}
        */
    }
    
	if(user_input == 0){
		outtextxy(getmaxx()/2, getmaxy()/2 -100, you_win);
		delay(3000);
    
	}
    
    while(!kbhit());     //kerobot.getY()pressé
    closegraph();
    
	return 0;
}
