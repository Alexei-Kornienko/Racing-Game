/*
 * Tire.cpp
 *
 *  Created on: Sep 9, 2011
 *      Author: alex
 */

#include "Tire.h"

Tire::Tire(dVector localPos, dFloat tireMass, dFloat tireRaduis, dFloat tireWidth, void *userData)
{
	this->harpoint = localPos;
	this->localPos = localPos;
	this->mass = tireMass;
	this->raduis = tireRaduis;
	this->width = tireWidth;
	this->userData = userData;
}

Tire::~Tire() {

}



void Tire::setLocalCoordinates(dMatrix localCoordinates)
{
    this->localCoordinates = localCoordinates;
    this->localCoordinates.m_posit = dVector(0, 0, 0, 0);
}

void Tire::setSuspension(const dFloat value)
{
	this->localPos = this->getHarpoint() - this->getLocalCoordinates().m_up.Scale(value);
}

float Tire::getAngularSpeed() const
{
    return angularSpeed;
}

float Tire::getSpinAngle() const
{
    return spinAngle;
}

void Tire::setAngularSpeed(float angularSpeed)
{
    this->angularSpeed = angularSpeed;
}

void Tire::setSpinAngle(float spinAngle)
{
    this->spinAngle = spinAngle;
}

dFloat Tire::generateTiresSteerAngle (dFloat value) const
{
	dFloat steerAngle = this->turnAngle;
	dFloat m_maxSteerAngle = 40.0f ;
	dFloat m_maxSteerRate = 3;
	if ( value > 0.0f ) {
		steerAngle += m_maxSteerRate;
		if ( steerAngle > m_maxSteerAngle ) {
			steerAngle = m_maxSteerAngle;
		}
	} else if ( value < 0.0f ) {
		steerAngle -= m_maxSteerRate;
		if ( steerAngle < -m_maxSteerAngle ) {
			steerAngle = -m_maxSteerAngle;
		}
	} else {
		if ( steerAngle > 0.0f ) {
			steerAngle -= m_maxSteerRate;
			if ( steerAngle < 0.0f ) {
				steerAngle = 0.0f;
			}
		} else if ( steerAngle < 0.0f ) {
			steerAngle += m_maxSteerRate;
			if ( steerAngle > 0.0f ) {
				steerAngle = 0.0f;
			}
		}
	}

	return steerAngle;
}

void Tire::setTorque(const dFloat torque)
{
	this->torque = torque;
}

void Tire::setBrake(const dFloat torque)
{
	this->brakeTorque = -torque;
}

void Tire::setSteerDirection(const dFloat direction)
{
	this->turnAngle = this->generateTiresSteerAngle(direction);
}

void Tire::setLocalPos(dVector localPos)
{
    this->localPos = localPos;
}

dVector Tire::getHarpoint() const
{
    return this->harpoint;
}

dMatrix Tire::getLocalCoordinates() const
{
    return localCoordinates;
}

dVector Tire::getLocalPos() const
{
    return localPos;
}

dFloat Tire::getMass() const
{
    return mass;
}

dFloat Tire::getRaduis() const
{
    return raduis;
}

dFloat Tire::getTorque() const
{
    return torque;
}

dFloat Tire::getTurnAngle() const
{
    return turnAngle;
}

void *Tire::getUserData() const
{
    return userData;
}

dFloat Tire::getWidth() const
{
    return width;
}
