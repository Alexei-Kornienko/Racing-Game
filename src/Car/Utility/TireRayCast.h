/*
 * TireRayCast.h
 *
 *  Created on: Sep 10, 2011
 *      Author: alex
 */

#ifndef TIRERAYCAST_H_
#define TIRERAYCAST_H_

#include "racingGame.h"
#include "Car/BaseCar.h"

class BaseCar;

class TireRayCast {
public:
	TireRayCast(BaseCar *carBody, const dMatrix & globalSpace, Tire *tire, dFloat suspensionLenght);
	~TireRayCast();

	void castRay();

	static dFloat filterCallback(const NewtonBody *body, const dFloat *hitNormal, int collisionID, void *userData, dFloat intersectParam);
    static unsigned preFilterCallback(const NewtonBody *body, const NewtonCollision *collision, void *userData);
    bool hasContact();
    dFloat getHitDistance() const;
    dVector getHitNormal() const;
    Tire *getTire() const;
private:
    BaseCar *car;
    dMatrix globalSpace;
    Tire *tire;
    dFloat suspensionLenght;
    dVector hitNormal;
    dFloat hitDistance;
};

#endif /* TIRERAYCAST_H_ */
