/*
 * Tire.h
 *
 *  Created on: Sep 9, 2011
 *      Author: alex
 */

#ifndef TIRE_H_
#define TIRE_H_

#include "racingGame.h"

class Tire {
public:
	Tire(dVector localPos, dFloat tireMass, dFloat tireRaduis, dFloat tireWidth, void *userData);
    ~Tire();
    void setTorque(const dFloat torque);
    void setBrake(const dFloat torque);
    void setSteerDirection(const dFloat direction);
    dFloat getMass() const;
    dFloat getRaduis() const;
    dFloat getTorque() const;
    dFloat getTurnAngle() const;
    void *getUserData() const;
    dFloat getWidth() const;
    dVector getLocalPos() const;
    void setLocalPos(dVector localPos);
    dVector getHarpoint() const;
    void setSuspension(const dFloat value);
    dMatrix getLocalCoordinates() const;
    void setLocalCoordinates(dMatrix localCoordinates);
    dFloat getAngularSpeed() const;
    dFloat getSpinAngle() const;
    void setAngularSpeed(dFloat angularSpeed);
    void setSpinAngle(dFloat spinAngle);
protected:
private:
    dMatrix localCoordinates; // local coordinate system of the tire
    dVector harpoint; // initial position of the tire (without suspension)
    dVector localPos; // local position of the tire (with suspension)
    dFloat turnAngle;
    dFloat angularSpeed;
    dFloat spinAngle;
    dFloat mass;
    dFloat raduis;
    dFloat width;
    dFloat torque;
    dFloat brakeTorque;
    void *userData;
    dFloat generateTiresSteerAngle(const dFloat value) const;

};

#endif /* TIRE_H_ */
