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

class GameStateController;

class Car  {
public:
	Car(GameStateController * controller);
	virtual ~Car();
	virtual void update(u32 timeSpan);
	float getWheelsTurn() const;
    vector3df getDirection() const;
    int getHelthPoints() const;
    vector3df getPosition() const;
    float getSpeed() const;


protected:
    void setDirection(const vector3df direction);
    void setPosition(const vector3df position);

	void doAccelerate();
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

	void resetMovement();

	void turnWheels(float);
	void rotateWheels(float);

	void updateSpeed();
    vector3df position;
	vector3df direction;
	float speed;
	float maxSpeed;
	float acceleration;
	float frictionForward;
	float frictionSide;
	float frictionBrakes;

	bool accelerate;
	bool brake;
	bool turnRigth;
	bool turnLeft;

	int helthPoints;

	IAnimatedMesh * carMeshClean;
	IAnimatedMesh * carMeshDamaged;
	IAnimatedMesh * wheelMesh;
};
#endif /* CAR_H_ */
