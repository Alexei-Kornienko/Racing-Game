/*
 * PlayerCar.cpp
 *
 *  Created on: Sep 5, 2011
 *      Author: alex
 */

#include "PlayerCar.h"

PlayerCar::PlayerCar(GameController *controller) : Car(controller)
{
	// add camera
//	this->camera = this->controller->getSmgr()->addCameraSceneNode(
//		this->carNode,
//		vector3df(0,3,-5),
//		vector3df(0,0,0)
//	);
	// add camera
	this->camera = this->controller->getSmgr()->addCameraSceneNodeFPS(0,100.0f,0.01f);
	this->camera->setPosition(core::vector3df(0,10,-10));
	this->camera->setTarget(core::vector3df(0,0,0));
	this->camera->setFarValue(100.0f);
}

PlayerCar::~PlayerCar()
{
}

void PlayerCar::update(dFloat timeSpan)
{
	if(this->controller->isKeyDown(KEY_KEY_W)) {
		this->doAccelerate();
	}
	if(this->controller->isKeyDown(KEY_KEY_S)) {
		this->doReverse();
	}
	if(this->controller->isKeyDown(KEY_SPACE)) {
		this->doBrake();
	}
	if(this->controller->isKeyDown(KEY_KEY_A)) {
		this->doTurnLeft();
	} else if(this->controller->isKeyDown(KEY_KEY_D)) {
		this->doTurnRight();
	} else {
		this->centerWheels();
	}
	Car::update(timeSpan);
	//this->camera->setTarget(this->carNode->getPosition());
}

ICameraSceneNode *PlayerCar::getCamera() const
{
    return camera;
}

void PlayerCar::carDestroyed()
{
	this->controller->gameOver(false);
}





