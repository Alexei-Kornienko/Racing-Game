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
protected:
    void setCarBodyAndGravity(NewtonBody *carBody, const dVector &gravity);
private:
    dVector speed;
    int tiresCount;
    struct SuspensionTire
    {
        Tire *t;
        float suspensionLenght;
        float suspensionSpring;
        float suspensionDamper;
	};

	int tiresC;
	SuspensionTire * tires;

	NewtonWorld * world;
	NewtonBody * carBody;
	dFloat mass;
	dVector gravity;

	dMatrix localCoordinates;
};

#endif /* BASECAR_H_ */
