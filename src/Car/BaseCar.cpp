/*
 * BaseCar.cpp
 *
 *  Created on: Sep 9, 2011
 *      Author: alex
 */

#include "BaseCar.h"


BaseCar::BaseCar(int tiresCount, NewtonWorld * world)
{
	this->tiresCount = tiresCount;
	this->tiresC = 0;
	this->tires = new SuspensionTire[this->tiresCount];

	this->world = world;
}



BaseCar::~BaseCar()
{
	for(int i=0; i < this->getTiresCount(); i++) {
		delete this->getTire(i);
	}
	delete[] this->tires;
	this->tires = 0;
}



void BaseCar::update(const float timeSpan)
{
	dMatrix globalPos;
	NewtonBodyGetMatrix(this->carBody, &globalPos[0][0]);

	TireRayCast * tireCast = new TireRayCast(this, globalPos, this->getTire(0), this->tires[0].suspensionLenght);
	tireCast->castRay();
	if(tireCast->hasContact()) {
		this->getTire(0)->setSuspension(this->tires[0].suspensionLenght * tireCast->getHitDistance());
//		NewtonCalculateSpringDamperAcceleration();
	} else {
		this->getTire(0)->setSuspension(this->tires[0].suspensionLenght);
	}
	delete tireCast;

	NewtonBodyAddForce(this->carBody, &this->gravity[0]);
}



dVector BaseCar::getSpeed() const
{
	return this->speed;
}



int BaseCar::getTiresCount() const
{
	return this->tiresCount;
}



Tire *BaseCar::getTire(const int index) const
{
	return this->tires[index].t;
}



void BaseCar::addSuspensionTire(Tire *t, const float suspensionLenght, const float suspensionSpring, const float suspensionDamper)
{
	if(this->tiresC < this->tiresCount) {
		t->setLocalCoordinates(this->getLocalCoordinates());
		this->tires[this->tiresC].t = t;
		this->tires[this->tiresC].suspensionLenght = suspensionLenght;
		this->tires[this->tiresC].suspensionSpring = suspensionSpring;
		this->tires[this->tiresC].suspensionDamper = suspensionDamper;
		this->tiresC++;
	}
}

NewtonBody *BaseCar::getCarBody() const
{
    return carBody;
}

void BaseCar::setTirePosTest()
{

}

void BaseCar::setCarBodyAndGravity(NewtonBody *carBody, const dVector &gravity)
{
    this->carBody = carBody;
    dFloat Ixx;
    dFloat Iyy;
    dFloat Izz;
    NewtonBodyGetMassMatrix(this->carBody, &this->mass, &Ixx, &Iyy, &Izz);
    this->mass = 5;
    this->gravity = dVector(
    	gravity.m_x * this->mass,
    	gravity.m_y * this->mass,
    	gravity.m_z * this->mass,
    	1.0f
    );

}

dMatrix BaseCar::getLocalCoordinates() const
{
    return localCoordinates;
}

void BaseCar::setLocalCoordinates(dMatrix localCoordinates)
{
    this->localCoordinates = localCoordinates;
}

NewtonWorld *BaseCar::getWorld() const
{
    return world;
}






