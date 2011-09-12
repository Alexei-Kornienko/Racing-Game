/*
 * BaseCar.h
 *
 *  Created on: Sep 9, 2011
 *      Author: alex
 */

#ifndef BASECAR_H_
#define BASECAR_H_

#include "racingGame.h"
#include "Utility/Tire.h"
#include "Utility/TireRayCast.h"

class TireRayCast;

class BaseCar {
public:
	BaseCar(int tiresCount, NewtonWorld * world);
	virtual ~BaseCar();

	virtual void update(const float timeSpan);
    void setTirePosTest();
    dVector getSpeed() const;
    int getTiresCount() const;
    Tire *getTire(const int index) const;
    void addSuspensionTire(Tire *t, const float suspensionLenght, const float suspensionSpring, const float suspensionDamper);
    NewtonBody *getCarBody() const;
    dMatrix getLocalCoordinates() const;
    void setLocalCoordinates(dMatrix localCoordinates);
    NewtonWorld *getWorld() const;
protected:
    void setCarBodyAndGravity(NewtonBody *carBody, const dVector & gravity);
private:
    dVector speed;
    dVector angularSpeed;
    dVector massCenter;
    int tiresCount;
    int tiresC;
    float distanceBetweenFrontAndRearTire;
    struct SuspensionTire
    {
        Tire *t;
        float suspensionLenght;
        float suspensionSpring;
        float suspensionDamper;
        dVector tireSpeed;
    } *tires;
    NewtonWorld *world;
    NewtonBody *carBody;
    float mass;
    dVector gravity;
    dMatrix localCoordinates;
    void applyOmegaFriction();
    dVector applyTireFriction(SuspensionTire & sTire);
    float getTireMassLoad();
};

#endif /* BASECAR_H_ */
