/*
 * Car.h
 *
 *  Created on: 23.07.2011
 *      Author: Alex
 */

#ifndef CAR_H_
#define CAR_H_

#include "racingGame.h"

class Car {
public:
	Car();
	~Car();
	void update(u32 timeSpan);

protected:
private:
	vector2df position;
	vector2df direction;
	float speed;
	float acceleration;
	float frictionForward;
	float frictionSide;
	float frictionBrakes;

	IMesh * carMesh;
};
#endif /* CAR_H_ */
