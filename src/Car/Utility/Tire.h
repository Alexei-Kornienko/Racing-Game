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
protected:
private:
    dVector localPos;
    float turnAngle;
    float mass;
    float raduis;
    float width;
    float torque;
    float brakeTorque;
    void *userData;
    float generateTiresSteerAngle(const float value) const;

};

#endif /* TIRE_H_ */
