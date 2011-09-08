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

class Car : public CustomRayCastCar {
friend void applyCarMoveForce(const NewtonBody* body, float timestep, int threadIndex);
    friend void applyCarTransform(const NewtonBody *body, const float *matrix, int threadIndex);
public:
    Car(GameStateController *controller);
    virtual ~Car();
    virtual void update(float timeSpan, int index);
    int getHelthPoints() const;
    vector3df getPosition() const;
    vector3df getDirection() const;
    float getWheelsTurn() const;
    void setPosition(const vector3df pos);
    virtual void SetBrake(float torque);
    virtual void SetTorque(float torque);
    virtual void SetSteering(float angle);
protected:
    GameStateController *controller;
    IAnimatedMeshSceneNode *carNode;
    IAnimatedMeshSceneNode *wheels[4];
    IAnimatedMeshSceneNode *wheelFL;
    IAnimatedMeshSceneNode *wheelFR;
    IAnimatedMeshSceneNode *wheelBL;
    IAnimatedMeshSceneNode *wheelBR;
    void doAccelerate();
    void doReverse();
    void doBrake();
    void doTurnLeft();
    void doTurnRight();
private:
    void init();
    IAnimatedMeshSceneNode *initModels(ISceneManager*);
    NewtonBody *initPhysics(GameStateController *controller);
    void initVenichlePhysics(NewtonWorld*);
    dMatrix createChassisMatrix();
    float generateTiresSteerAngle(float value);
    dFloat generateTiresSteerForce (dFloat value);
    void updateWheelsPos();



	int helthPoints;

	NewtonBody* body;

	IAnimatedMesh * carMeshClean;
	IAnimatedMesh * carMeshDamaged;
	IAnimatedMesh * wheelMesh;
};
#endif /* CAR_H_ */
