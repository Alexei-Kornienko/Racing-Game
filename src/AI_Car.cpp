/*
 * AI_Car.cpp
 *
 *  Created on: Sep 5, 2011
 *      Author: alex
 */

#include "AI_Car.h"
#include "math.h"

AI_Car::AI_Car(GameStateController * controller, Car * targetCar) : Car(controller)
{
	this->targetCar = targetCar;
	vector3df position((rand()%20) - 10, 0, (rand()%20) - 10);
	this->setPosition(position);
}

AI_Car::~AI_Car()
{
}

void AI_Car::update(u32 timeSpan)
{
	vector3df targetVector = this->targetCar->getPosition() - this->getPosition();
	vector3df direction = this->getDirection().getHorizontalAngle();
	vector3df targetAngle = targetVector.getHorizontalAngle();
	float resultAngle = targetAngle.Y - direction.Y;

	float rotation = this->getWheelsTurn() * this->getSpeed() * timeSpan*2 / 1000.f;

	resultAngle -= rotation;
	if(resultAngle > 10) {
		this->doTurnRight();
	} else if(resultAngle < -10) {
		this->doTurnLeft();
	}

	if(targetVector.getLength() > 2 && fabs(this->getWheelsTurn() < 30)) {
		this->doAccelerate();
	} else {
		if(this->getSpeed() > -2) {
			this->doBrake();
		}
	}
	Car::update(timeSpan);
}
