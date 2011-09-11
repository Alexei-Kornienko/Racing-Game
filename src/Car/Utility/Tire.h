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
	Tire(
		dVector localPos, float tireMass, float tireRaduis, float tireWidth, void *userData);
    ~Tire();
    void setTorque(const float torque);
    void setBrake(const float torque);
    void setSteerDirection(const float direction);
    float getMass() const;
    float getRaduis() const;
    float getTorque() const;
    float getTurnAngle() const;
    void *getUserData() const;
    float getWidth() const;
    dVector getLocalPos() const;
    void setLocalPos(dVector localPos);
    dVector getHarpoint() const;
    void setSuspension(const float value);
    dMatrix getLocalCoordinates() const;
    void setLocalCoordinates(dMatrix localCoordinates);
    float getAngularSpeed() const;
    float getSpinAngle() const;
    void setAngularSpeed(float angularSpeed);
    void setSpinAngle(float spinAngle);
protected:
private:
    dMatrix localCoordinates; // local coordinate system of the tire
    dVector harpoint; // initial position of the tire (without suspension)
    dVector localPos; // local position of the tire (with suspension)
    float turnAngle;
    float angularSpeed;
    float spinAngle;
    float mass;
    float raduis;
    float width;
    float torque;
    float brakeTorque;
    void *userData;
    float generateTiresSteerAngle(const float value) const;

};

#endif /* TIRE_H_ */
