/*
 * Car.h
 *
 *  Created on: 23.07.2011
 *      Author: Alex
 */

#ifndef CAR_H_
#define CAR_H_

#include "racingGame.h"
#include "GameStateController.h"
#include "lib/CustomDGRayCastCar.h"

class GameStateController;

void applyCarMoveForce(const NewtonBody* body, dFloat timestep, int threadIndex);
void applyCarTransform (const NewtonBody* body, const dFloat* matrix, int threadIndex);

class Car  {
friend void applyCarMoveForce(const NewtonBody* body, dFloat timestep, int threadIndex);
friend void applyCarTransform (const NewtonBody* body, const dFloat* matrix, int threadIndex);

public:
	Car(GameStateController * controller);
	virtual ~Car();
	virtual void update(dFloat timeSpan) = 0;
	float getWheelsTurn() const;
    vector3df getDirection() const;
    int getHelthPoints() const;
    vector3df getPosition() const;
    float getSpeed() const;


protected:
    void setDirection(const vector3df direction);
    void setPosition(const vector3df position);

	void doAccelerate();
	void doReverse();
	void doBrake();
	void doTurnLeft();
	void doTurnRight();

	GameStateController * controller;
	IAnimatedMeshSceneNode * carNode;

	IAnimatedMeshSceneNode * wheelFL;
	IAnimatedMeshSceneNode * wheelFR;
	IAnimatedMeshSceneNode * wheelBL;
	IAnimatedMeshSceneNode * wheelBR;

	bool floatEqual(float, float);

private:
	void initModels(ISceneManager *);
	void initPhysics(NewtonWorld *);
	void initVenichlePhysics(NewtonWorld *);

	void resetMovement();

	void turnWheels(float);
	void rotateWheels(float);

	vector3df getAccelerationForce(dFloat time);
    vector3df position;
	vector3df direction;
	float speed;
	float maxSpeed;
	float acceleration;
	float frictionForward;
	float frictionSide;
	float frictionBrakes;

	bool accelerate;
	bool reverse;
	bool brake;
	bool turnRigth;
	bool turnLeft;

	int helthPoints;

	NewtonBody* body;
	CustomDGRayCastCar * newtonCar;

	IAnimatedMesh * carMeshClean;
	IAnimatedMesh * carMeshDamaged;
	IAnimatedMesh * wheelMesh;
};
#endif /* CAR_H_ */
