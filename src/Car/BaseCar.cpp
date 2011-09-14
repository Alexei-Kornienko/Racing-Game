/*
 * BaseCar.cpp
 *
 *  Created on: Sep 9, 2011
 *      Author: alex
 */

#include "BaseCar.h"


BaseCar::BaseCar(int tiresCount, NewtonWorld * world, GameController * controller)
{
	this->tiresCount = tiresCount;
	this->tiresC = 0;
	this->tires = new SuspensionTire[this->tiresCount];

	this->world = world;
	this->controller = controller;
}



BaseCar::~BaseCar()
{
	for(int i=0; i < this->getTiresCount(); i++) {
		delete this->getTire(i);
	}
	delete[] this->tires;
	this->tires = 0;
	this->tiresCount = 0;
}



float BaseCar::getTireMassLoad(const SuspensionTire & sTire)
{
    return this->mass / this->getTiresCount();
}

dVector BaseCar::applyTireFriction(SuspensionTire & sTire)
{
    dMatrix localCoord = sTire.t->getLocalCoordinates();
    dVector friction(0,0,0,0);
    dVector carForceSum = this->speed.Scale(this->getTireMassLoad(sTire));
    dFloat frontForce = carForceSum % localCoord.m_front;

    dFloat rollintFriction = 0.002/sTire.t->getRaduis() * sTire.tireLoad;
    if(frontForce > 0) {
    	friction -= localCoord.m_front.Scale(rollintFriction);
    } else {
    	friction += localCoord.m_front.Scale(rollintFriction);
    }

    dFloat sideForce = carForceSum % localCoord.m_right;
    dFloat slideFriction = 0.5 * sTire.tireLoad * 2;
    if(dAbs(sideForce) > 0.01) {
    	dVector sF = localCoord.m_right.Scale(slideFriction);
		if(sideForce > 0) {
			friction -= sF;
		} else {
			friction += sF;
		}
    }
    return friction;
}

dVector BaseCar::applyOmegaFriction(SuspensionTire & sTire)
{
	dVector resultTorque(0,0,0,0);

    dFloat expectedAnglularSpeed = 0;
    if(sTire.t->getTurnAngle() != 0) {
		dFloat turnRaduis = this->distanceBetweenFrontAndRearTire / sin(sTire.t->getTurnAngle() * DEGTORAD);
		expectedAnglularSpeed = (this->speed % this->localCoordinates.m_front) / turnRaduis;
	}
    dFloat actualAngularSpeed = this->angularSpeed % this->localCoordinates.m_up;
    dFloat speedDiff = actualAngularSpeed - expectedAnglularSpeed;
    if(dAbs(speedDiff) > 0.1) {
    	dFloat slideFriction = 0.5 * sTire.tireLoad;
		if(speedDiff > 0) {
			resultTorque -= this->localCoordinates.m_up.Scale(slideFriction);
		} else {
			resultTorque += this->localCoordinates.m_up.Scale(slideFriction);
		}

	}
    return resultTorque;
}

dVector BaseCar::applyTireForce(const Tire * t)
{
	return t->getLocalCoordinates().m_front.Scale(t->getTorque()*10);
}

dVector BaseCar::applyTireLoad(SuspensionTire & sTire, const TireRayCast * tireCast, const float timeSpan)
{
    sTire.t->setSuspension(sTire.suspensionLenght * tireCast->getHitDistance());
    sTire.tireLoad = NewtonCalculateSpringDamperAcceleration(
				timeSpan,
				sTire.suspensionSpring,
				-((1 - tireCast->getHitDistance()) / 2),
				sTire.suspensionDamper,
				sTire.tireSpeed % this->localCoordinates.m_up
			);
    sTire.tireLoad *= this->getTireMassLoad(sTire);
    dVector tireForce = this->localCoordinates.m_up.Scale(sTire.tireLoad);
    return tireForce;
}

void BaseCar::getUpdatedGlobalState()
{
    NewtonBodyGetMatrix(this->carBody, &this->globalCoordinates[0][0]);

    // get the chassis instantaneous linear and angular velocity in the local space of the chassis
    NewtonBodyGetVelocity(this->carBody, &this->speed[0]);
    this->speed = this->globalCoordinates.UnrotateVector(this->speed);

    NewtonBodyGetOmega(this->carBody, &this->angularSpeed[0]);
    this->angularSpeed = this->globalCoordinates.UnrotateVector(this->angularSpeed);

    NewtonBodyGetForce(this->carBody, &this->currentBodyForce[0]);
    this->currentBodyForce = this->globalCoordinates.UnrotateVector(this->currentBodyForce);

    NewtonBodyGetTorque(this->carBody, &this->currentBodyTorque[0]);
    this->currentBodyTorque = this->globalCoordinates.UnrotateVector(this->currentBodyTorque);
}

void BaseCar::update(const float timeSpan)
{
	GameController::VectorDraw draw;

    this->getUpdatedGlobalState();

	dVector resultForce(0,0,0,0);
	dVector resultTorque(0,0,0,0);
	for(int i=0, c=this->getTiresCount(); i < c; i++) {
		dVector tireForce(0,0,0,0);
		dVector tireTorque(0,0,0,0);
		SuspensionTire sTire = this->tires[i];
		sTire.tireSpeed = this->speed + this->angularSpeed * sTire.t->getLocalPos();
		TireRayCast * tireCast = new TireRayCast(this, this->globalCoordinates, sTire.t, sTire.suspensionLenght);
		tireCast->castRay();
		if(tireCast->hasContact()) {
			tireForce += this->applyTireLoad(sTire, tireCast, timeSpan);
			tireForce += this->applyTireForce(sTire.t);
			tireForce += this->applyTireFriction(sTire);

			tireTorque += tireForce * (this->massCenter - sTire.t->getLocalPos());
			tireTorque += this->applyOmegaFriction(sTire);
			if (dAbs(this->speed % this->localCoordinates.m_front)!=0.0f && sTire.t->getTurnAngle() != 0) {
				dVector turnForce = this->localCoordinates.m_right.Scale(sTire.t->getTurnAngle()*20) * this->speed;
				tireTorque -= turnForce;
			}
			resultForce += tireForce;
			resultTorque += tireTorque;
		} else {
			sTire.t->setSuspension(sTire.suspensionLenght);
		}
		delete tireCast;

		sTire.t->setTorque(0);
	}

	resultForce = this->globalCoordinates.RotateVector(resultForce);
	resultTorque = this->globalCoordinates.RotateVector(resultTorque);
	resultForce += this->gravity;
	NewtonBodySetForce(this->carBody, &resultForce[0]);
	NewtonBodySetTorque(this->carBody, &resultTorque[0]);
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







