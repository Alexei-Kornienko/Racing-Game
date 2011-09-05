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
	this->maxSpeed = 10;

	this->resetMovement();

	// Experimental constants
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
	this->carNode->remove();
}

vector3df Car::getDirection() const
{
    return direction;
}

int Car::getHelthPoints() const
{
    return helthPoints;
}

vector3df Car::getPosition() const
{
    return position;
}

float Car::getSpeed() const
{
    return speed;
}

void Car::setDirection(const vector3df direction)
{
    this->direction = direction;
}

void Car::setPosition(const vector3df position)
{
    this->position = position;
}

float Car::getWheelsTurn() const
{
	return this->wheelFL->getRotation().Y;
}

void Car::updateSpeed()
{
    if(this->accelerate && this->speed < this->maxSpeed){
        this->speed += this->acceleration;
    }
    if(this->brake && this->speed > this->maxSpeed / -2.f){
        this->speed -= this->frictionBrakes;
    }
    if(!this->floatEqual(this->speed, 0)){
        this->speed += (this->speed > 0) ? -this->frictionForward : this->frictionForward;
    }
}

void Car::update(u32 timeSpan)
{
	float seconds = timeSpan / 1000.f;

	this->turnWheels(seconds);

    this->updateSpeed();

    float distance = this->speed * seconds;

    this->rotateWheels(distance);

	float rotation = -this->wheelFL->getRotation().Y * distance;

	this->direction.rotateXZBy(rotation, vector3df(0, 0, 0));

	this->carNode->setRotation(this->direction.getHorizontalAngle());

	vector3df positionChange = this->direction * distance;

	this->position += positionChange;
	this->carNode->setPosition(this->position);

	this->resetMovement();
}

bool Car::floatEqual(float f1, float f2) {
	return fabs(f1 - f2) < 0.01f;
}

void Car::rotateWheels(float distance) {
	float angle = 360 * (distance / 0.68); // 0.68 -- Approximate wheel length

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

	float turnSpeedDeg = 50 * seconds;
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
			turnSpeedDeg *= 2;
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
