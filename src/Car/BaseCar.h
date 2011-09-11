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

	virtual void update(const dFloat timeSpan);
	void setTirePosTest();
    dVector getSpeed() const;
    int getTiresCount() const;
    Tire *getTire(const int index) const;
    void addSuspensionTire(Tire *t, const dFloat suspensionLenght, const dFloat suspensionSpring, const dFloat suspensionDamper);
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
    struct SuspensionTire
    {
        Tire *t;
        dFloat suspensionLenght;
        dFloat suspensionSpring;
        dFloat suspensionDamper;
        dVector tireSpeed;
    };
    int tiresC;
    SuspensionTire *tires;
    NewtonWorld *world;
    NewtonBody *carBody;
    dFloat mass;
	dVector gravity;

	dMatrix localCoordinates;
    void applyTireFriction(SuspensionTire & sTire);
    dFloat getTireMassLoad();
};

#endif /* BASECAR_H_ */
