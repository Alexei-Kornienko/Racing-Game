/*
 * Car.cpp
 *
 *  Created on: Aug 11, 2011
 *      Author: alex
 */

#include "Car.h"

#include "math.h"

#define JEEP_MASS					(900.0f)

#define JEEP_TIRE_MASS				(30.0f)
#define JEEP_SUSPENSION_LENGTH		(0.2f)
#define JEEP_SUSPENSION_SPRING		(175.0f)
#define JEEP_SUSPENSION_DAMPER		(6.0f)

void applyCarMoveForce(const NewtonBody* body, dFloat timestep, int threadIndex) {
	Car* car = (Car*) NewtonBodyGetUserData(body);
	if (car)
	{
		car->update(timestep, threadIndex);
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



Car::Car(GameStateController * controller) :
	CustomRayCastCar(4,
		this->createChassisMatrix(),
		this->initPhysics(controller),
		dVector(0.0f, -10.f, 0.0f, 1.0f)
	)
{
    this->init();
	this->initVenichlePhysics(this->controller->getWorld());

}

void Car::init()
{
    this->helthPoints = 100;
}

IAnimatedMeshSceneNode * Car::initModels(ISceneManager * smgr) {
	this->carMeshClean = smgr->getMesh("res/carC.obj");
	this->carMeshDamaged = smgr->getMesh("res/carD.obj");
	this->carNode = smgr->addAnimatedMeshSceneNode(
		this->carMeshClean,
		0,
		-1,
		vector3df(0,2,0)
	);
//	this->carNode->setRotation(vector3df(0,30,0));

	this->wheelMesh = smgr->getMesh("res/wheel.obj");

	float wheelPosY = -0.2235f;
//	float wheelPosY = -0.4f;

	this->wheelFR = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(0.3065f, wheelPosY, 0.619f)
	);
	this->wheels[0] = this->wheelFR;
	this->wheelFL = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(-0.3065f, wheelPosY, 0.619f)
	);
	this->wheels[1] = this->wheelFL;
	this->wheelBR = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(0.3065f, wheelPosY, -0.55f)
	);
	this->wheels[2] = this->wheelBR;
	this->wheelBL = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(-0.3065f, wheelPosY, -0.55f)
	);
	this->wheels[3] = this->wheelBL;

	return this->carNode;
}

NewtonBody * Car::initPhysics(GameStateController * controller) {
	this->controller = controller;
	this->initModels(this->controller->getSmgr());
	NewtonWorld * nWorld = this->controller->getWorld();
	NewtonCollision* collision;
	float mass = JEEP_MASS; // TODO mass is equal to 1 Car :)

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

	//	core::quaternion qIrr(rotation);
	//	dQuaternion q(qIrr.X, qIrr.Y, qIrr.Z, qIrr.W);
	//	dVector v(this->carNode->getPosition().X, this->carNode->getPosition().Y, this->carNode->getPosition().Z);
	//	dMatrix matrix;

	//	NewtonConvexHullModifierSetMatrix(collision, &matrix[0][0]);
	//	this->body = NewtonCreateBody(nWorld, collision, &matrix[0][0]);

	matrix4 m = this->carNode->getRelativeTransformation();
	NewtonConvexHullModifierSetMatrix(collision, m.pointer());
	this->body = NewtonCreateBody(nWorld, collision, m.pointer());
	NewtonBodySetUserData(this->body, this);
	NewtonConvexCollisionCalculateInertialMatrix(collision, &inertia[0], &origin[0]);
//	inertia.m_x = 500.0f;
//	inertia.m_y = 500.0f;
//	inertia.m_z = -500.0f;
	NewtonBodySetMassMatrix(this->body, mass, mass * inertia.m_x, mass * inertia.m_y, mass * inertia.m_z);
//	NewtonBodySetMassMatrix(this->body, mass, inertia.m_x, inertia.m_y, inertia.m_z);
	NewtonBodySetCentreOfMass(this->body, &origin[0]);
	NewtonBodySetForceAndTorqueCallback(this->body, applyCarMoveForce);
	NewtonBodySetTransformCallback(this->body, applyCarTransform);
	return this->body;
}


void Car::initVenichlePhysics(NewtonWorld *nWorld)
{

	float wheelMass = JEEP_TIRE_MASS;
	float wheelRaduis = 0.147f;
//	float wheelRaduis = 3.147f;
	float wheelWidth = 0.104f;
	float suspensionLenght = 0.1;

	int castMode = 0;

	this->AddSingleSuspensionTire(
			this->wheelFR,
			dVector(this->wheelFR->getPosition().X,this->wheelFR->getPosition().Y,this->wheelFR->getPosition().Z, 1.0f),
			wheelMass,
			wheelRaduis,
			wheelWidth,
			suspensionLenght,
			JEEP_SUSPENSION_SPRING,
			JEEP_SUSPENSION_DAMPER,
			castMode
	);
	this->AddSingleSuspensionTire(
			this->wheelFL,
			dVector(this->wheelFL->getPosition().X,this->wheelFL->getPosition().Y,this->wheelFL->getPosition().Z, 1.0f),
			wheelMass,
			wheelRaduis,
			wheelWidth,
			suspensionLenght,
			JEEP_SUSPENSION_SPRING,
			JEEP_SUSPENSION_DAMPER,
			castMode
	);
	this->AddSingleSuspensionTire(
			this->wheelBR,
			dVector(this->wheelBR->getPosition().X,this->wheelBR->getPosition().Y,this->wheelBR->getPosition().Z, 1.0f),
			wheelMass,
			wheelRaduis,
			wheelWidth,
			suspensionLenght,
			JEEP_SUSPENSION_SPRING,
			JEEP_SUSPENSION_DAMPER,
			castMode
	);
	this->AddSingleSuspensionTire(
			this->wheelBL,
			dVector(this->wheelBL->getPosition().X,this->wheelBL->getPosition().Y,this->wheelBL->getPosition().Z, 1.0f),
			wheelMass,
			wheelRaduis,
			wheelWidth,
			suspensionLenght,
			JEEP_SUSPENSION_SPRING,
			JEEP_SUSPENSION_DAMPER,
			castMode
	);
}

void Car::SetBrake(float torque)
{
	int count = this->GetTiresCount();
	for(int i=0; i<count; i++) {
		this->SetTireBrake(i,torque);
	}
}

void Car::SetTorque(float torque)
{
	this->SetTireTorque(2, torque);
	this->SetTireTorque(3, torque);
}

void Car::SetSteering(float angle)
{
	// TODO fix parameters
	this->SetTireSteerAngle(0, angle, 10);
	this->SetTireSteerAngle(1, angle, 10);
}

void Car::update(dFloat timeSpan, int index)
{
	this->SubmitConstraints(timeSpan, index);

	int tCount = this->GetTiresCount();
	for(int i=0; i<tCount; i++) {
		Tire t = this->GetTire(i);
		vector3df rot = this->wheels[i]->getRotation();
		rot.Y = t.m_steerAngle;
		rot.X = -t.m_spinAngle * RADTODEG;
		this->wheels[i]->setRotation(rot);

		vector3df pos = this->wheels[i]->getPosition();
		pos.Y = t.m_harpoint.m_y - t.m_posit;
		this->wheels[i]->setPosition(pos);
	}
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
	dQuaternion q(this->carNode->getRotation().X, this->carNode->getRotation().Y, this->carNode->getRotation().Z, 1.f);
	dVector v(pos.X, pos.Y, pos.Z);
	dMatrix matrix(q, v);
	NewtonBodySetMatrix(this->body, &matrix[0][0]);
}

void Car::doAccelerate()
{
	this->SetTorque(-100); // TODO fix parameters
}

void Car::doReverse()
{
	this->SetTorque(50); // TODO fix parameters
}

void Car::doBrake()
{
	this->SetBrake(1000); // TODO fix parameters
}

void Car::doTurnLeft()
{
	this->SetSteering(-30); // TODO fix parameters
}

void Car::doTurnRight()
{
	this->SetSteering(30); // TODO fix parameters
}

vector3df Car::getDirection() const
{
	return vector3df(); // TODO implement me
}

float Car::getWheelsTurn() const
{
	return 0; // TODO implement me
}

dMatrix Car::createChassisMatrix()
{
    // set the vehicle local coordinate system
    dMatrix chassisMatrix;
    // if you vehicle move along the z direction you can use
    chassisMatrix.m_front = dVector(0.0f, 0.0f, 1.0f, 0.0);
    chassisMatrix.m_up = dVector(0.0f, 1.0f, 0.0f, 0.0f); // this is the downward vehicle direction
//    chassisMatrix.m_right = dVector(1.0f, 0.0f, 0.0f, 0.0f); // this is in the side vehicle direction (the plane of the wheels)
//    chassisMatrix.m_right = chassisMatrix.m_front * chassisMatrix.m_up; // this is in the side vehicle direction (the plane of the wheels)
    chassisMatrix.m_right = chassisMatrix.m_up * chassisMatrix.m_front; // this is in the side vehicle direction (the plane of the wheels)
    chassisMatrix.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);
    return chassisMatrix;
}

Car::~Car()
{
}

int Car::getHelthPoints() const
{
    return helthPoints;
}
