/*
 * Car.cpp
 *
 *  Created on: Aug 11, 2011
 *      Author: alex
 */

#include "Car.h"
#include "math.h"

void applyCarMoveForce(const NewtonBody* body, dFloat timestep, int threadIndex) {
	Car* car = (Car*) NewtonBodyGetUserData(body);
	if (car)
	{
		car->update(timestep);
		car->turnWheels(timestep);

		dVector force(0.0f, 0.0f, 0.0f, 1.0f);

		force += car->getAccelerationForce(timestep);
		car->speed += force.m_y;
		NewtonBodySetForce(body, &force[0]);
		car->resetMovement();
	}
}

void applyCarTransform (const NewtonBody* body, const dFloat* matrix, int threadIndex) {
	Car* car = (Car*) NewtonBodyGetUserData(body);
	if (car)
	{
		matrix4 transform;
		transform.setM(matrix);
		//transform *= NEWTON_TO_IRR;
		vector3df oldPos = car->carNode->getPosition();
		car->carNode->setPosition(transform.getTranslation());
		vector3df distance = car->carNode->getPosition() - oldPos;
		car->rotateWheels(distance.getLength());
		car->carNode->setRotation(transform.getRotationDegrees());
	}
}

Car::Car(GameStateController * controller)
{
	this->controller = controller;

	this->initModels(controller->getSmgr());
	this->initPhysics(controller->getWorld());

	this->position = vector3df(0,0,0);
	this->direction = vector3df(0,0,1);

	this->helthPoints = 100;
	this->speed = 0;

	this->resetMovement();

	// Experimental constants
	this->acceleration = 100.0f;
	this->frictionBrakes = 50.0f;
	this->frictionForward = 0.0f;
	this->frictionSide = 0.0f;

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

void Car::initPhysics(NewtonWorld * nWorld) {
	NewtonCollision* collision;
	float mass = 1.0f; // mass is equal to 1 Car :)

	vector3df v1 = this->carNode->getBoundingBox().MinEdge;
	vector3df v2 = this->carNode->getBoundingBox().MaxEdge;

	dVector minBox(v1.X, v1.Y, v1.Z);
	dVector maxBox(v2.X, v2.Y, v2.Z);

	dVector size(maxBox - minBox);
	dVector origin((maxBox + minBox).Scale(0.5f));

	size.m_w = 1.0f;
	origin.m_w = 1.0f;

	dMatrix offset(GetIdentityMatrix());
	offset.m_posit = origin;

	collision = NewtonCreateBox(nWorld, size.m_x, size.m_y, size.m_z, 0, &offset[0][0]);

	dVector inertia;

	dQuaternion q(this->carNode->getRotation().X, this->carNode->getRotation().Y, this->carNode->getRotation().Z, 1.f);
	dVector v(this->carNode->getPosition().X, this->carNode->getPosition().Y, this->carNode->getPosition().Z);
	dMatrix matrix(q, v);

	this->body = NewtonCreateBody(nWorld, collision, &matrix[0][0]);

	NewtonBodySetUserData(this->body, this);

	NewtonConvexCollisionCalculateInertialMatrix(collision, &inertia[0], &origin[0]);

	NewtonBodySetMassMatrix(this->body, mass, mass * inertia.m_x, mass * inertia.m_y, mass * inertia.m_z);

	NewtonBodySetCentreOfMass(this->body, &origin[0]);

	NewtonBodySetForceAndTorqueCallback(this->body, applyCarMoveForce);

	NewtonBodySetTransformCallback(this->body, applyCarTransform);
}

Car::~Car()
{
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
    return this->carNode->getPosition();
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
	dQuaternion q(this->carNode->getRotation().X, this->carNode->getRotation().Y, this->carNode->getRotation().Z, 1.f);
	vector3df scalePos = position;// * IRR_TO_NEWTON;
	dVector v(scalePos.X, scalePos.Y, scalePos.Z);
	dMatrix matrix(q, v);
	NewtonBodySetMatrix(this->body, &matrix[0][0]);
}

float Car::getWheelsTurn() const
{
	return this->wheelFL->getRotation().Y;
}

bool Car::floatEqual(float f1, float f2) {
	return fabs(f1 - f2) < 0.01f;
}

void Car::rotateWheels(float distance) {
	float angle = 360 * (distance / 0.68) * 0.75f; // 0.68 -- Approximate wheel length

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

dVector Car::getAccelerationForce(dFloat time)
{
	dVector force(0.0f,0.0f, -this->frictionForward * time, 1.0f);
	if(this->accelerate) {
		force.m_z += this->acceleration * time;
	} else if(this->brake) {
		force.m_z -= this->frictionBrakes * time;
	}

	return force;
}


