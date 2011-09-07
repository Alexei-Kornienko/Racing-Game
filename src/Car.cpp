/*
 * Car.cpp
 *
 *  Created on: Aug 11, 2011
 *      Author: alex
 */

#include "Car.h"

#include "math.h"

#define JEEP_MASS					(900.0f)

void applyCarMoveForce(const NewtonBody* body, dFloat timestep, int threadIndex) {
	Car* car = (Car*) NewtonBodyGetUserData(body);
	if (car)
	{
		car->update(timestep);
		car->turnWheels(timestep);

		dFloat velocity[3];
		NewtonBodyGetVelocity(body, velocity);

		vector3df speed(velocity[0],velocity[1],velocity[2]);
		car->speed = speed.getLength();
		f32 distance = car->speed*timestep;
		vector3df rotation = car->carNode->getRotation();

//		CustomDGRayCastCar::Tire tire = car->newtonCar->GetTire(0);
////		dVector v = car->newtonCar->GetChassisMatrixLocal().UntransformVector(tire.m_harpoint);
//		dVector v = tire.m_harpoint;
//		vector3df wPos(v.m_x, v.m_y, v.m_z);
//		car->wheelFR->setPosition(wPos);

		if(car->accelerate) {
			dFloat torque = car->acceleration * timestep * -1000;
			car->newtonCar->SetTireTorque(2,car->newtonCar->GenerateEngineTorque(torque));
			car->newtonCar->SetTireTorque(3,car->newtonCar->GenerateEngineTorque(torque));
		} else if(car->reverse) {
			dFloat torque = car->acceleration * timestep * 500;
			car->newtonCar->SetTireTorque(2,car->newtonCar->GenerateEngineTorque(torque));
			car->newtonCar->SetTireTorque(3,car->newtonCar->GenerateEngineTorque(torque));
		}

		if (car->brake) {
			car->newtonCar->SetTireBrake(0, 1000.f);
			car->newtonCar->SetTireBrake(1, 1000.f);
			car->newtonCar->SetTireBrake(2, 1000.f);
			car->newtonCar->SetTireBrake(3, 1000.f);
		}
		dFloat turnDirection = 0;
		if(car->turnLeft) {
			turnDirection = -1;
		} else if (car->turnRigth) {
			turnDirection = 1;
		}
		dFloat angle = car->newtonCar->GenerateTiresSteerAngle(turnDirection);
		dFloat force = car->newtonCar->GenerateTiresSteerForce(turnDirection);
		car->newtonCar->SetTireSteerAngleForce(2,angle,force);
		car->newtonCar->SetTireSteerAngleForce(3,angle,force);

		dFloat Ixx, Iyy, Izz;
		dFloat mass;


		car->newtonCar->update(timestep, threadIndex);
		NewtonBodyGetMassMatrix(body, &mass, &Ixx, &Iyy, &Izz);
		dVector gravityForce(0.0f, mass * -10.f, 0.0f, 1.0f);
		NewtonBodyAddForce(body, &gravityForce[0]);

		car->resetMovement();
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

	this->position = vector3df(0,0,0);
//	this->direction = vector3df(0.f,0.f,0.9f);
	this->direction = vector3df(0.f,0.f,0.0f);

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

//	float wheelPosY = -0.2235f;
	float wheelPosY = -0.5f;

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
			if(rotation.Y < 30) {
				rotation.Y += turnSpeedDeg;
			}
		}
		if (this->turnLeft) {
			if(rotation.Y > -30) {
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
	this->reverse = false;
	this->brake = false;
	this->turnRigth = false;
	this->turnLeft = false;
}

void Car::doAccelerate()
{
	this->accelerate = true;
}

void Car::doReverse()
{
	this->reverse = true;
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
	chassisMatrix.m_right = dVector (1.0f, 0.0f, 0.0f, 0.0f);	// this is in the side vehicle direction (the plane of the wheels)
	chassisMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
	this->newtonCar = new CustomDGRayCastCar(4, chassisMatrix, this->body);

	float wheelMass = (30.0f);
	float wheelRaduis = 0.147f;
	float wheelWidth = 0.104f;
	float wheelFriction = 2.5f;
	float suspensionLenght = wheelRaduis * 50.0;
	float suspensionSpring = 175.0f;
	float suspensionDamper = 6.0f;


	int castMode = 0;

	this->newtonCar->AddSingleSuspensionTire(
			this->wheelFR,
			dVector(this->wheelFR->getPosition().X,this->wheelFR->getPosition().Y,this->wheelFR->getPosition().Z, 1.0f),
			wheelMass,
			wheelRaduis,
			wheelWidth,
			wheelFriction,
			suspensionLenght,
			suspensionSpring,
			suspensionDamper,
			castMode
	);
	this->newtonCar->AddSingleSuspensionTire(
			this->wheelFL,
			dVector(this->wheelFL->getPosition().X,this->wheelFL->getPosition().Y,this->wheelFL->getPosition().Z, 1.0f),
			wheelMass,
			wheelRaduis,
			wheelWidth,
			wheelFriction,
			suspensionLenght,
			suspensionSpring,
			suspensionDamper,
			castMode
	);
	this->newtonCar->AddSingleSuspensionTire(
			this->wheelBR,
			dVector(this->wheelBR->getPosition().X,this->wheelBR->getPosition().Y,this->wheelBR->getPosition().Z, 1.0f),
			wheelMass,
			wheelRaduis,
			wheelWidth,
			wheelFriction,
			suspensionLenght,
			suspensionSpring,
			suspensionDamper,
			castMode
	);
	this->newtonCar->AddSingleSuspensionTire(
			this->wheelBL,
			dVector(this->wheelBL->getPosition().X,this->wheelBL->getPosition().Y,this->wheelBL->getPosition().Z, 1.0f),
			wheelMass,
			wheelRaduis,
			wheelWidth,
			wheelFriction,
			suspensionLenght,
			suspensionSpring,
			suspensionDamper,
			castMode
	);
}

