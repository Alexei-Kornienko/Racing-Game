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
//#define JEEP_SUSPENSION_SPRING		(175.0f)
//#define JEEP_SUSPENSION_DAMPER		(6.0f)
#define JEEP_SUSPENSION_SPRING		(100.0f)
#define JEEP_SUSPENSION_DAMPER		(6.0f)

void applyCarMoveForce(const NewtonBody* body, dFloat timestep, int threadIndex) {
	Car* car = (Car*) NewtonBodyGetUserData(body);
	if (car)
	{
	}
}

void applyCarTransform (const NewtonBody* body, const dFloat* matrix, int threadIndex) {
	Car* car = (Car*) NewtonBodyGetUserData(body);
	if (car)
	{
		matrix4 transform;
		transform.setM(matrix);
		car->carNode->setPosition(transform.getTranslation());
		car->carNode->setRotation(transform.getRotationDegrees());
	}
}

Car::Car(GameStateController * controller)
{
	this->controller = controller;

	this->position = vector3df(0,5,0);
	this->direction = vector3df(0,0,1);
	this->helthPoints = 100;
	this->speed = 0;

	this->resetMovement();

	// Experimental constants
	this->acceleration = 100.0f;
	this->frictionBrakes = 100.0f;
	this->frictionForward = 0.0f;
	this->frictionSide = 0.0f;

	this->initModels(controller->getSmgr());
	this->initPhysics(controller->getWorld());
	this->initVenichlePhysics(controller->getWorld());

}

void Car::initModels(ISceneManager * smgr) {
	this->carMeshClean = smgr->getMesh("res/carC.obj");
	this->carMeshDamaged = smgr->getMesh("res/carD.obj");
	this->carNode = smgr->addAnimatedMeshSceneNode(
		this->carMeshClean,
		0,
		-1,
		this->position,
		this->direction
	);


	this->wheelMesh = smgr->getMesh("res/wheel.obj");

	float wheelPosY = -0.2235f;
//	float wheelPosY = -0.4f;

	this->wheelFR = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(0.3065f, wheelPosY, 0.619f)
	);

	this->wheelFL = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(-0.3065f, wheelPosY, 0.619f)
	);
	this->wheelBR = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(0.3065f, wheelPosY, -0.55f)
	);
	this->wheelBL = smgr->addAnimatedMeshSceneNode(
		this->wheelMesh,
		this->carNode,
		-1,
		vector3df(-0.3065f, wheelPosY, -0.55f)
	);
}

void Car::initPhysics(NewtonWorld * nWorld) {
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

	dQuaternion q(this->carNode->getRotation().X, this->carNode->getRotation().Y, this->carNode->getRotation().Z, 1.f);
	dVector v(this->getPosition().X, this->getPosition().Y, this->getPosition().Z);
	dMatrix matrix(q, v);

	this->body = NewtonCreateBody(nWorld, collision);
	NewtonBodySetMatrix(this->body, &matrix[0][0]);
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

int Car::getHelthPoints() const
{
    return helthPoints;
}

void Car::setPosition(const vector3df position)
{
	dQuaternion q(this->carNode->getRotation().X, this->carNode->getRotation().Y, this->carNode->getRotation().Z, 1.f);
	vector3df scalePos = position;// * IRR_TO_NEWTON;
	dVector v(scalePos.X, scalePos.Y, scalePos.Z);
	dMatrix matrix(q, v);
	NewtonBodySetMatrix(this->body, &matrix[0][0]);
}

void Car::resetMovement() {
	this->accelerate = false;
	this->reverse = false;
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

void Car::initVenichlePhysics(NewtonWorld *nWorld)
{

	// set the vehicle local coordinate system
	dMatrix chassisMatrix;
	// if you vehicle move along the z direction you can use
	chassisMatrix.m_front = dVector (0.0f, 0.0f, 1.0f, 0.0);
	chassisMatrix.m_up	  = dVector (0.0f, 1.0f, 0.0f, 0.0f);			// this is the downward vehicle direction
	chassisMatrix.m_right = chassisMatrix.m_front * chassisMatrix.m_up;	// this is in the side vehicle direction (the plane of the wheels)
	chassisMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
	this->newtonCar = new CustomRayCastCar( // FIXME
		4,
		chassisMatrix,
		this->body,
		dVector gravityForce(0.0f, -1.f, 0.0f, 1.0f)
	);

	float wheelMass = JEEP_TIRE_MASS;
	float wheelRaduis = 0.147f;
	float wheelWidth = 0.104f;
	float wheelFriction = 2.5f;
	float suspensionLenght = wheelRaduis/4;

	int castMode = 0;

	this->newtonCar->AddSingleSuspensionTire(
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
	this->newtonCar->AddSingleSuspensionTire(
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
	this->newtonCar->AddSingleSuspensionTire(
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
	this->newtonCar->AddSingleSuspensionTire(
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

void Car::doReverse() {
	this->reverse = true;
}
