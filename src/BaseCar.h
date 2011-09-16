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
#include "TireRayCast.h"
#include "GameStateController.h"

class TireRayCast;
class GameController;

class BaseCar {
public:
	BaseCar(int tiresCount, NewtonWorld * world, GameController * controller);
	virtual ~BaseCar();

	virtual void update(const float timeSpan);
    void addSuspensionTire(Tire *t, const float suspensionLenght, const float suspensionSpring, const float suspensionDamper);
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
    GameController *controller;
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
        float tireLoad;
    } *tires;
    NewtonWorld *world;
    NewtonBody *carBody;
    float mass;
    dVector gravity;
    dMatrix localCoordinates;
    dMatrix globalCoordinates;
    dVector currentBodyForce;
    dVector currentBodyTorque;
    void updateTireSpin(SuspensionTire & sTire, const float timeSpan);
    void getUpdatedGlobalState();
    dVector applyTireLoad(SuspensionTire & sTire, const TireRayCast & tireCast, const float timeSpan);
    dVector applyTireForce(const Tire *t);
    dVector applyOmegaFriction(SuspensionTire & sTire);
    dVector applyTireFriction(SuspensionTire & sTire);
    float getTireMassLoad(const SuspensionTire & sTire);
};

#endif /* BASECAR_H_ */
