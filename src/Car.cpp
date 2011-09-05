/*
 * Car.cpp
 *
 *  Created on: Aug 11, 2011
 *      Author: alex
 */

#include "Car.h"
#include "math.h"

Car::Car(GameStateController * controller)
{
	this->controller = controller;

	ISceneManager * smgr = controller->getSmgr();
	this->initModels(smgr);

	this->position = vector3df(0,0,0);
	this->direction = vector3df(0,0,1);

	this->helthPoints = 100;
	this->speed = 0;

	this->resetMovement();

	// Experimental constants //TODO fix hardcode
	this->acceleration = 0.05f;
	this->frictionBrakes = 0.08f;
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
		vector3df(0.3065f, -0.2235f, 0.619f) //TODO fix hardcode
	);

	this->wheelFL = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(-0.3065f, -0.2235f, 0.619f) //TODO fix hardcode
	);
	this->wheelBR = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(0.3065f, -0.2235f, -0.55f) //TODO fix hardcode
	);
	this->wheelBL = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(-0.3065f, -0.2235f, -0.55f) //TODO fix hardcode
	);
}

Car::~Car()
{
}

void Car::updateSpeed()
{
    if(this->accelerate){
        this->speed += this->acceleration;
    }
    if(this->brake){
        this->speed -= this->frictionBrakes;
    }
    if(!this->floatEqual(this->speed, 0)){
        this->speed += (this->speed > 0) ? -this->frictionForward : this->frictionForward;
    }
}

void Car::update(u32 timeSpan)
{
	float seconds = timeSpan / 1000.f;


    this->updateSpeed();

    float turnSpeed = 50.f; //TODO fix hardcode
	float rotation = 0;
	if (this->turnRigth) {
		rotation = -turnSpeed * seconds * this->speed;
	}
	if (this->turnLeft) {
		rotation = turnSpeed * seconds * this->speed;
	}
	this->direction.rotateXZBy(rotation, vector3df(0, 0, 0));

	this->carNode->setRotation(this->direction.getHorizontalAngle());

	float distance = this->speed * seconds;
	vector3df positionChange = this->direction * distance;

	this->position += positionChange;
	this->carNode->setPosition(this->position);

	this->rotateWheels(distance);
	this->turnWheels(seconds);

	this->resetMovement();
}

bool Car::floatEqual(float f1, float f2) {
	return fabs(f1 - f2) < 0.01f;
}

void Car::rotateWheels(float distance) {
	float angle = 360 * (distance / 0.68); // 0.68 -- Approximate wheel length //TODO fix hardcode

	vector3df rotation = this->wheelFL->getRotation();
	rotation.X += angle;

	this->wheelFL->setRotation(rotation);
	this->wheelFR->setRotation(rotation);

	rotation.Y = 0;
	this->wheelBL->setRotation(rotation);
	this->wheelBR->setRotation(rotation);
}

void Car::turnWheels(float seconds) {
	vector3df rotation = this->wheelFL->getRotation();

	float turnSpeedDeg = 500 * seconds; //TODO fix hardcode
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
			turnSpeedDeg /= 3;
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
