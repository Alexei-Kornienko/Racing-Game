/*
 * TireRayCast.cpp
 *
 *  Created on: Sep 10, 2011
 *      Author: alex
 */

#include "TireRayCast.h"

TireRayCast::TireRayCast(BaseCar *car, const dMatrix & globalSpace, Tire *tire, dFloat suspensionLenght)
{
	this->car = car;
	this->globalSpace = globalSpace;
	this->tire = tire;
	this->suspensionLenght = suspensionLenght;
	this->hitDistance = 1;
}

TireRayCast::~TireRayCast()
{
}

void TireRayCast::castRay()
{
	dVector tirePosGlobal = this->globalSpace.TransformVector(this->tire->getHarpoint());
	dVector sVectorDown =  this->car->getLocalCoordinates().m_up.Scale(
		(this->suspensionLenght + this->tire->getRaduis())
	);
	dVector sVectorUp =  this->car->getLocalCoordinates().m_up.Scale(
		(this->suspensionLenght - this->tire->getRaduis())
	);
	sVectorDown = this->globalSpace.RotateVector(sVectorDown);
	dVector tireRayDirection = tirePosGlobal - sVectorDown;
	tirePosGlobal += sVectorUp;
	NewtonWorldRayCast(
		this->car->getWorld(),
		&tirePosGlobal[0],
		&tireRayDirection[0],
		TireRayCast::filterCallback,
		this,
		TireRayCast::preFilterCallback
	);
}

dFloat TireRayCast::filterCallback(const NewtonBody *body, const dFloat *hitNormal, int collisionID, void *userData, dFloat intersectParam)
{
	TireRayCast * me = (TireRayCast *)userData;
	if (intersectParam < me->hitDistance) {
		me->hitDistance = intersectParam;
		me->hitNormal = dVector(hitNormal);
	}
	return me->hitDistance;
}

unsigned TireRayCast::preFilterCallback(const NewtonBody *body, const NewtonCollision *collision, void *userData)
{
	TireRayCast * me = (TireRayCast *)userData;
	return me->car->getCarBody() != body;
}

bool TireRayCast::hasContact()
{
	return this->hitDistance < 1;
}

dFloat TireRayCast::getHitDistance() const
{
    return (hitDistance*2 - 1);
}

dVector TireRayCast::getHitNormal() const
{
    return hitNormal;
}

Tire *TireRayCast::getTire() const
{
    return tire;
}
