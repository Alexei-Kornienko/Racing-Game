/*
 * BaseCar.cpp
 *
 *  Created on: Sep 9, 2011
 *      Author: alex
 */

#include "BaseCar.h"




BaseCar::BaseCar(int tiresCount)
{
	this->tiresCount = tiresCount;
	this->tires = new Tire*[this->tiresCount];
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
	// TODO implement
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
	return this->tires[index];
}



void BaseCar::addSuspensionTire(Tire *t, const float suspensionLenght, const float suspensionSpring, const float suspensionDamper)
{
}

NewtonBody *BaseCar::getCarBody() const
{
    return carBody;
}

void BaseCar::setCarBody(NewtonBody *carBody)
{
    this->carBody = carBody;
}



