#ifndef OBSTACLE_H
#define OBSTACLE_H

class Obstacle: public Cercle{
	public:
		Obstacle(float x = 0, float y = 0, float r = 0):Cercle(x, y, r){}
};

#endif
