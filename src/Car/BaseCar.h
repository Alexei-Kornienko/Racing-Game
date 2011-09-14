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
#include "GameStateController.h"

class TireRayCast;
class GameController;

class BaseCar {
public:
	BaseCar(int tiresCount, NewtonWorld * world, GameController * controller);
	virtual ~BaseCar();

	virtual void update(const dFloat timeSpan);

	void addSuspensionTire(Tire *t, const dFloat suspensionLenght, const dFloat suspensionSpring, const dFloat suspensionDamper);

    dVector getSpeed() const;
    int getTiresCount() const;
    Tire *getTire(const int index) const;
    NewtonBody *getCarBody() const;
    NewtonWorld *getWorld() const;
    dMatrix getLocalCoordinates() const;
    void setLocalCoordinates(dMatrix localCoordinates);

protected:
    void setCarBodyAndGravity(NewtonBody *carBody, const dVector & gravity);
private:
    GameController * controller;
    dVector speed;
    dVector angularSpeed;
    dVector massCenter;
    int tiresCount;
    int tiresC;
    dFloat distanceBetweenFrontAndRearTire;
    struct SuspensionTire
    {
        Tire *t;
        dFloat suspensionLenght;
        dFloat suspensionSpring;
        dFloat suspensionDamper;
        dVector tireSpeed;
        dFloat tireLoad;
    } *tires;
    NewtonWorld *world;
    NewtonBody *carBody;
    dFloat mass;
    dVector gravity;
    dMatrix localCoordinates;
    dMatrix globalCoordinates;
    dVector currentBodyForce;
    dVector currentBodyTorque;

    void getUpdatedGlobalState();
    dVector applyTireLoad(SuspensionTire & sTire, const TireRayCast & tireCast, const dFloat timeSpan);
    dVector applyTireForce(const Tire * t);
    dVector applyOmegaFriction(SuspensionTire & sTire);
    dVector applyTireFriction(SuspensionTire & sTire);
    dFloat getTireMassLoad(const SuspensionTire & sTire);
};

#endif /* BASECAR_H_ */
