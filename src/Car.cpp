/*
 * Car.cpp
 *
 *  Created on: Aug 11, 2011
 *      Author: alex
 */

#include "Car.h"
#include "math.h"

Car::Car(ISceneManager * smgr)
{
	this->initModels(smgr);


	this->position = vector3df(0,0,0);
	this->direction = vector3df(0,0,1);

	this->helthPoints = 100;
	this->speed = 0;

	this->resetMovement();

	// Experimental constants
	this->acceleration = 0.05f;
	this->frictionBrakes = 0.1f;
	this->frictionForward = 0.01f;
	this->frictionSide = 0.005f;

}

void Car::initModels(ISceneManager * smgr) {
	this->carMeshClean = smgr->getMesh("res/carC.obj");
	this->carMeshDamaged = smgr->getMesh("res/carD.obj");
	this->carNode = smgr->addAnimatedMeshSceneNode(this->carMeshClean);

	this->wheelMesh = smgr->getMesh("res/wheel.obj");
	this->wheelFR = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(0.3065f, -0.2235f, 0.619f)
	);

	this->wheelFL = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(-0.3065f, -0.2235f, 0.619f)
	);
	this->wheelBR = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(0.3065f, -0.2235f, -0.55f)
	);
	this->wheelBL = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(-0.3065f, -0.2235f, -0.55f)
	);
}

Car::~Car()
{
}

void Car::update(u32 timeSpan)
{
	if(this->accelerate) {
		this->speed += this->acceleration;
	}
	if(this->brake) {
		this->speed -= this->frictionBrakes;
	}
	if(!this->floatEqual(this->speed, 0)) {
		this->speed += (this->speed > 0) ? -this->frictionForward: this->frictionForward;
	}

	float seconds = timeSpan / 1000.f;
	vector3df positionChange = this->direction * this->speed * seconds;
	this->position += positionChange;
	this->carNode->setPosition(this->position);

	this->turnWheels(seconds);

	this->resetMovement();
}

bool Car::floatEqual(float f1, float f2) {
	return fabs(f1 - f2) < 0.01f;
}

void Car::turnWheels(float seconds) {
	vector3df rotation = this->wheelFL->getRotation();

	float turnSpeedDeg = 500 * seconds;
	if(this->turnLeft || this->turnRigth) {
		if(this->turnRigth) {
			if(rotation.Y < 45) {
				rotation.Y += turnSpeedDeg;
			}
		}
		if (this->turnLeft) {
			if(rotation.Y > -45) {
				rotation.Y -= turnSpeedDeg;
			}
		}
	} else {
		if(!this->floatEqual(rotation.Y,0) && !this->floatEqual(this->speed, 0)) {
			turnSpeedDeg /= 2;
			if(fabs(rotation.Y) < turnSpeedDeg) {
				rotation.Y = 0;
			} else {
				rotation.Y += (rotation.Y > 0) ? -turnSpeedDeg : turnSpeedDeg;
			}
		}
	}
	this->wheelFL->setRotation(rotation);
	this->wheelFR->setRotation(rotation);
}

void Car::resetMovement() {
	this->accelerate = false;
	this->brake = false;
	this->turnRigth = false;
	this->turnLeft = false;
}

void Car::doAccelerate()
{
	this->accelerate = true;
}

void Car::doBrake()
{
	this->brake = true;
}

void Car::doTurnLeft()
{
	this->turnLeft = true;
}

void Car::doTurnRight()
{
	this->turnRigth = true;
}
