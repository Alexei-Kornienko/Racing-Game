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
#include "lib/CustomRayCastCar.h"

class GameStateController;

void applyCarMoveForce(const NewtonBody* body, dFloat timestep, int threadIndex);
void applyCarTransform (const NewtonBody* body, const dFloat* matrix, int threadIndex);

class Car {
friend void applyCarMoveForce(const NewtonBody* body, dFloat timestep, int threadIndex);
friend void applyCarTransform (const NewtonBody* body, const dFloat* matrix, int threadIndex);

public:
	Car(GameStateController * controller);
	virtual ~Car();
	virtual void update(dFloat timeSpan) = 0;

protected:

	GameStateController * controller;
	IAnimatedMeshSceneNode * carNode;

	IAnimatedMeshSceneNode * wheelFL;
	IAnimatedMeshSceneNode * wheelFR;
	IAnimatedMeshSceneNode * wheelBL;
	IAnimatedMeshSceneNode * wheelBR;

private:
	void initModels(ISceneManager *);
	void initPhysics(NewtonWorld *);
	void initVenichlePhysics(NewtonWorld *);

	int helthPoints;

	NewtonBody* body;

	IAnimatedMesh * carMeshClean;
	IAnimatedMesh * carMeshDamaged;
	IAnimatedMesh * wheelMesh;
};
#endif /* CAR_H_ */
