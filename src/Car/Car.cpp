/*
 * Car.cpp
 *
 *  Created on: Aug 11, 2011
 *      Author: alex
 */

#include "Car.h"

void applyCarMoveForce(const NewtonBody* body, dFloat timestep, int threadIndex) {
	Car* car = (Car*) NewtonBodyGetUserData(body);
	if (car)
	{
		car->update(timestep);
	}
}

void applyCarTransform (const NewtonBody* body, const dFloat* matrix, int threadIndex) {
	Car* car = (Car*) NewtonBodyGetUserData(body);
	if (car)
	{
		matrix4 transform;
		transform.setM(matrix);
		car->carNode->setPosition(transform.getTranslation());
		vector3df rotation = transform.getRotationDegrees();
		car->carNode->setRotation(rotation);
	}
}



Car::Car(GameStateController * controller) : BaseCar(WHEELS_COUNT, controller->getWorld())
{
	this->controller = controller;
	this->initModels(this->controller->getSmgr());
	this->initPhysics();
    this->init();
	this->initVenichlePhysics(this->controller->getWorld());

}

void Car::init()
{
    this->helthPoints = 100;
}

void Car::initModels(ISceneManager * smgr) {
	this->carMeshClean = smgr->getMesh("res/carC.obj");
	this->carMeshDamaged = smgr->getMesh("res/carD.obj");
	this->carNode = smgr->addAnimatedMeshSceneNode(
		this->carMeshClean,
		0,
		-1,
		vector3df(0,1,0)
	);
	this->carNode->setRotation(vector3df(0,180,0));

	this->wheelMesh = smgr->getMesh("res/wheel.obj");

//	float wheelPosY = -0.2235f;
	float wheelPosY = -0.4f;

	this->wheels[0] = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(0.3065f, wheelPosY, 0.619f)
	);
	this->wheels[1] = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(-0.3065f, wheelPosY, 0.619f)
	);
	this->wheels[2] = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(0.3065f, wheelPosY, -0.55f)
	);
	this->wheels[3] = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(-0.3065f, wheelPosY, -0.55f)
	);
}

void Car::initPhysics() {

	NewtonWorld * nWorld = this->controller->getWorld();
	NewtonCollision* collision;
	float mass = 900.0f;

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

	matrix4 m = this->carNode->getRelativeTransformation();
	NewtonConvexHullModifierSetMatrix(collision, m.pointer());
	NewtonBody * body = NewtonCreateBody(nWorld, collision, m.pointer());
	NewtonBodySetUserData(body, this);
	NewtonConvexCollisionCalculateInertialMatrix(collision, &inertia[0], &origin[0]);
	NewtonBodySetMassMatrix(body, mass, mass * inertia.m_x, mass * inertia.m_y, mass * inertia.m_z);
	NewtonBodySetCentreOfMass(body, &origin[0]);
	NewtonBodySetForceAndTorqueCallback(body, applyCarMoveForce);
	NewtonBodySetTransformCallback(body, applyCarTransform);

	this->setCarBodyAndGravity(body, dVector(0,-10,0,0));
	this->setLocalCoordinates(this->createChassisMatrix());
}


void Car::initVenichlePhysics(NewtonWorld *nWorld)
{
	float wheelMass = 30.0f;
	float wheelRaduis = 0.147f;
	float wheelWidth = 0.104f;
	float suspensionLenght = wheelRaduis;
//	float suspensionSpring = 20.0f;
	float suspensionSpring = 20.0f;
	float suspensionDamper = 10.0f;

	for(int i=0; i<WHEELS_COUNT; i++) {
		Tire * t = new Tire(
				dVector(this->wheels[i]->getPosition().X,this->wheels[i]->getPosition().Y,this->wheels[i]->getPosition().Z, 1.0f),
				wheelMass,
				wheelRaduis,
				wheelWidth,
				this->wheels[i]
		);
		this->addSuspensionTire(
			t,
			suspensionLenght,
			suspensionSpring,
			suspensionDamper
		);
	}
	this->setTirePosTest();
}

vector3df Car::getSpeed() const
{
	dVector v = BaseCar::getSpeed();
	return vector3df(v.m_x, v.m_y, v.m_z) ;
}

void Car::updateWheelsPos()
{
	for(int i=0, c=this->getTiresCount(); i<c; i++) {
		dVector tPos = this->getTire(i)->getLocalPos();
		vector3df pos(tPos.m_x, tPos.m_y, tPos.m_z);
		this->wheels[i]->setPosition(pos);
	}
}

void Car::update(dFloat timeSpan)
{
	BaseCar::update(timeSpan);
    this->updateWheelsPos();
	stringw text = "Position";
	text += stringw(" X:") + stringw(this->carNode->getPosition().X);
	text += stringw(" Y:") + stringw(this->carNode->getPosition().Y);
	text += stringw(" Z:") + stringw(this->carNode->getPosition().Z);
	text += "\nRotation";
	text += stringw(" X:") + stringw(this->carNode->getRotation().X);
	text += stringw(" Y:") + stringw(this->carNode->getRotation().Y);
	text += stringw(" Z:") + stringw(this->carNode->getRotation().Z);
	this->controller->getTextField()->setText(text.c_str());
}

vector3df Car::getPosition() const
{
	return this->carNode->getPosition();
}

void Car::setPosition(const vector3df pos)
{
	this->carNode->setPosition(pos);
	NewtonBodySetMatrix(this->getCarBody(), this->carNode->getRelativeTransformation().pointer());
}

void Car::doAccelerate() // TODO fix parameters
{
	this->getTire(2)->setTorque(400);
	this->getTire(3)->setTorque(400);
}

void Car::doReverse() // TODO fix parameters
{
	this->getTire(2)->setTorque(-400);
	this->getTire(3)->setTorque(-400);
}

void Car::doBrake() // TODO fix parameters
{
	int count = this->getTiresCount();
	for(int i=0; i<count; i++) {
		this->getTire(i)->setBrake(1000);
	}
}

void Car::doTurnLeft()
{
	this->getTire(0)->setSteerDirection(-1);
	this->getTire(1)->setSteerDirection(-1);
}

void Car::doTurnRight()
{
	this->getTire(0)->setSteerDirection(1);
	this->getTire(1)->setSteerDirection(1);
}

vector3df Car::getDirection() const
{
	return this->carNode->getRelativeTransformation().getRotationDegrees();
}

float Car::getWheelsTurn() const
{
	return this->wheels[0]->getRotation().Y;
}

dMatrix Car::createChassisMatrix()
{
    // set the vehicle local coordinate system
    dMatrix chassisMatrix;
    chassisMatrix.m_front	= dVector(0.0f, 0.0f, 1.0f, 0.0);
    chassisMatrix.m_up		= dVector(0.0f, 1.0f, 0.0f, 0.0f);
    chassisMatrix.m_right	= chassisMatrix.m_up * chassisMatrix.m_front;
    NewtonBodyGetCentreOfMass(this->getCarBody(), &chassisMatrix.m_posit[0]);
    return chassisMatrix;
}

Car::~Car()
{
}

int Car::getHelthPoints() const
{
    return helthPoints;
}
