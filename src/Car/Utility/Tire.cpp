/*
 * Tire.cpp
 *
 *  Created on: Sep 9, 2011
 *      Author: alex
 */

#include "Tire.h"

Tire::Tire(dVector localPos, dFloat tireMass, dFloat tireRaduis, dFloat tireWidth, void *userData)
{
	this->localPos = localPos;
	this->mass = tireMass;
	this->raduis = tireRaduis;
	this->width = tireWidth;
	this->userData = userData;
}

Tire::~Tire() {

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
