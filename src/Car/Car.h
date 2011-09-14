/*
 * Car.h
 *
 *  Created on: 23.07.2011
 *      Author: Alex
 */

#ifndef CAR_H_
#define CAR_H_

#include "racingGame.h"
//#include "GameStateController.h"

#include "Car/BaseCar.h"

#define WHEELS_COUNT 4

class BaseCar;
class GameController;


void applyCarMoveForce(const NewtonBody* body, dFloat timestep, int threadIndex);
void applyCarTransform (const NewtonBody* body, const dFloat* matrix, int threadIndex);
void applyCarCollisionForce(const NewtonJoint* contact, dFloat timestep, int threadIndex);

class Car : private BaseCar {
friend void applyCarMoveForce(const NewtonBody* body, float timestep, int threadIndex);
friend void applyCarTransform(const NewtonBody *body, const float *matrix, int threadIndex);
friend void applyCarCollisionForce(const NewtonJoint* contact, float timestep, int threadIndex);
public:
    Car(GameController *controller);
    virtual ~Car();
    virtual void update(float timeSpan);
    int getHelthPoints() const;
    vector3df getPosition() const;
    vector3df getDirection() const;
    void setPosition(const vector3df pos);
    float getWheelsTurn() const;
    vector3df getSpeed() const;

protected:
    GameController *controller;
    IAnimatedMeshSceneNode *carNode;
    IAnimatedMeshSceneNode *wheels[WHEELS_COUNT];

    void doAccelerate();
    void doReverse();
    void doBrake();
    void doTurnLeft();
    void doTurnRight();
    void centerWheels();
    virtual void carDestroyed() = 0;

    void applyDamagePoints(int helthPoints);
private:
    void init();
    void initModels(ISceneManager*);
    void initPhysics();
    void initVenichlePhysics(NewtonWorld*);

	dMatrix createChassisMatrix();

    void updateWheelsPos();



	int helthPoints;
	bool damaged;

	IAnimatedMesh * carMeshClean;
	IAnimatedMesh * carMeshDamaged;
	IAnimatedMesh * wheelMesh;
};
#endif /* CAR_H_ */
