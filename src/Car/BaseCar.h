/*
 * BaseCar.h
 *
 *  Created on: Sep 9, 2011
 *      Author: alex
 */

#ifndef BASECAR_H_
#define BASECAR_H_

#include "racingGame.h"
#include "Tire.h"

class BaseCar {
public:
	BaseCar(int tiresCount);
	~BaseCar();

	void update(const float timeSpan);

    dVector getSpeed() const;
    int getTiresCount() const;
    Tire *getTire(const int index) const;
    void addSuspensionTire(Tire *t, const float suspensionLenght, const float suspensionSpring, const float suspensionDamper);

    NewtonBody *getCarBody() const;
protected:
    void setCarBody(NewtonBody *carBody);
private:
	dVector speed;
	int tiresCount;
	Tire ** tires;

	NewtonBody * carBody;
};

#endif /* BASECAR_H_ */
