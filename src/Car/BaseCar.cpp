/*
 * BaseCar.cpp
 *
 *  Created on: Sep 9, 2011
 *      Author: alex
 */

#include "BaseCar.h"


BaseCar::BaseCar(int tiresCount, NewtonWorld * world)
{
	this->tiresCount = tiresCount;
	this->tiresC = 0;
	this->tires = new SuspensionTire[this->tiresCount];

	this->world = world;
}



BaseCar::~BaseCar()
{
	for(int i=0; i < this->getTiresCount(); i++) {
		delete this->getTire(i);
	}
	delete[] this->tires;
	this->tires = 0;
}



float BaseCar::getTireMassLoad()
{
    return this->mass / this->getTiresCount();
}

dVector BaseCar::applyTireFriction(SuspensionTire & sTire)
{
    dMatrix localCoord = sTire.t->getLocalCoordinates();
    dFloat frontSpeed = sTire.tireSpeed % localCoord.m_front;
    dVector friction(0,0,0,0);
    if(dAbs(frontSpeed) > 0.1) {
    	friction += localCoord.m_front.Scale(frontSpeed * 0.2);
	} else if(dAbs(frontSpeed) > 0) {
		friction += localCoord.m_front.Scale(frontSpeed * 0.9);
	}
    dFloat sideSpeed = this->speed % localCoord.m_right;
    if(dAbs(sideSpeed) > 0.1) {
    	friction += localCoord.m_right.Scale(sideSpeed * 0.5);
	} else if(dAbs(sideSpeed) > 0) {
		friction += localCoord.m_right.Scale(sideSpeed * 0.9);
	}
    return friction.Scale(1/this->getTiresCount());
}

void BaseCar::applyOmegaFriction()
{
    dFloat turnAngle = this->getTire(0)->getTurnAngle();
    dFloat expectedAnglularSpeed = 0;
    if(turnAngle != 0) {
		dFloat turnRaduis = this->distanceBetweenFrontAndRearTire / sin(turnAngle * DEGTORAD);
		expectedAnglularSpeed = (this->speed % this->localCoordinates.m_front) / turnRaduis;
	}
    dFloat actualAngularSpeed = this->angularSpeed % this->localCoordinates.m_up;
    if(dAbs(actualAngularSpeed - expectedAnglularSpeed) > 0.1) {
		if(actualAngularSpeed > 0.1) {
			this->angularSpeed = this->angularSpeed.Scale(0.5);
		} else {
			this->angularSpeed = this->angularSpeed.Scale(0.1);
		}
		NewtonBodySetOmega(this->carBody, &this->angularSpeed[0]);
	}
}

void BaseCar::update(const float timeSpan)
{
	dMatrix globalPos;
	NewtonBodyGetMatrix(this->carBody, &globalPos[0][0]);


	// get the chassis instantaneous linear and angular velocity in the local space of the chassis
	NewtonBodyGetVelocity(this->carBody, &this->speed[0]);
	NewtonBodyGetOmega(this->carBody, &this->angularSpeed[0]);
	this->speed = globalPos.UnrotateVector(this->speed);
	this->angularSpeed = globalPos.UnrotateVector(this->angularSpeed);
	dVector lineralSpeed = this->speed;
	for(int i=0, c=this->getTiresCount(); i < c; i++) {
		SuspensionTire sTire = this->tires[i];
		TireRayCast * tireCast = new TireRayCast(this, globalPos, sTire.t, sTire.suspensionLenght);
		tireCast->castRay();
		if(tireCast->hasContact()) {
			dFloat suspensionPosition = sTire.suspensionLenght * tireCast->getHitDistance();
			sTire.t->setSuspension(suspensionPosition);
			sTire.tireSpeed = this->speed + this->angularSpeed * sTire.t->getLocalPos();
			dFloat tireSuspensionSpeed = sTire.tireSpeed % this->localCoordinates.m_up;
			dFloat suspensionCompression = (1 - tireCast->getHitDistance()) / 2;

			dFloat tireLoad = NewtonCalculateSpringDamperAcceleration (timeSpan, sTire.suspensionSpring, -suspensionCompression, sTire.suspensionDamper, tireSuspensionSpeed);
			tireLoad *= this->getTireMassLoad();
			dVector tireForce = globalPos.m_up.Scale(tireLoad);
			tireForce += this->localCoordinates.m_front.Scale(sTire.t->getTorque()*5);
			dVector torque = tireForce * (this->massCenter - sTire.t->getLocalPos());

			tireForce = globalPos.RotateVector(tireForce);
			torque = globalPos.RotateVector(torque);
			NewtonBodyAddForce(this->carBody, &tireForce[0]);
			NewtonBodyAddTorque(this->carBody, &torque[0]);
			lineralSpeed -= this->applyTireFriction(sTire);
		} else {
			sTire.t->setSuspension(sTire.suspensionLenght);
		}
		delete tireCast;

		sTire.t->setTorque(0);
	}
    this->applyOmegaFriction();

	NewtonBodySetVelocity(this->carBody, &lineralSpeed[0]); // apply tire friction
	NewtonBodyAddForce(this->carBody, &this->gravity[0]);
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
	return this->tires[index].t;
}



void BaseCar::addSuspensionTire(Tire *t, const float suspensionLenght, const float suspensionSpring, const float suspensionDamper)
{
	for(int i=0; i<this->tiresC; i++) {
		dFloat distance1 = this->getTire(i)->getLocalPos() % this->getLocalCoordinates().m_front;
		dFloat distance2 = t->getLocalPos() % this->getLocalCoordinates().m_front;
		dFloat distance = dAbs(distance1 - distance2);
		if(distance > distance1 && distance > this->distanceBetweenFrontAndRearTire) {
			this->distanceBetweenFrontAndRearTire = distance;
		}
	}
	if(this->tiresC < this->tiresCount) {
		t->setLocalCoordinates(this->getLocalCoordinates());
		this->tires[this->tiresC].t = t;
		this->tires[this->tiresC].suspensionLenght = suspensionLenght;
		this->tires[this->tiresC].suspensionSpring = suspensionSpring;
		this->tires[this->tiresC].suspensionDamper = suspensionDamper;
		this->tiresC++;
	}
}

NewtonBody *BaseCar::getCarBody() const
{
    return carBody;
}

void BaseCar::setTirePosTest()
{

}

void BaseCar::setCarBodyAndGravity(NewtonBody *carBody, const dVector &gravity)
{
    this->carBody = carBody;
    dFloat Ixx;
    dFloat Iyy;
    dFloat Izz;
    NewtonBodyGetMassMatrix(this->carBody, &this->mass, &Ixx, &Iyy, &Izz);
    NewtonBodyGetCentreOfMass(this->carBody, &this->massCenter[0]);
//    this->mass = 200;
    this->gravity = dVector(
    	gravity.m_x * this->mass,
    	gravity.m_y * this->mass,
    	gravity.m_z * this->mass,
    	1.0f
    );
}

dMatrix BaseCar::getLocalCoordinates() const
{
    return localCoordinates;
}

void BaseCar::setLocalCoordinates(dMatrix localCoordinates)
{
    this->localCoordinates = localCoordinates;
}

NewtonWorld *BaseCar::getWorld() const
{
    return world;
}







