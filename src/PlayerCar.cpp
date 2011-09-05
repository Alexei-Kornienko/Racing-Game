/*
 * PlayerCar.cpp
 *
 *  Created on: Sep 5, 2011
 *      Author: alex
 */

#include "PlayerCar.h"

PlayerCar::PlayerCar(GameStateController *controller) : Car(controller)
{
	this->controller->addEventReceiver(this);
	// add camera
	this->camera = this->controller->getSmgr()->addCameraSceneNode(
		this->carNode,
		vector3df(0,3,-5),
		vector3df(0,0,0)
	);
	ISceneNodeAnimator * anim = this->controller->getSmgr()->createFlyCircleAnimator();

	this->camera->setFarValue(100.0f);
}

PlayerCar::~PlayerCar()
{
	this->controller->removeEventReceiver(this);
}

void PlayerCar::update(u32 timeSpan)
{
	Car::update(timeSpan);
	this->camera->setTarget(this->carNode->getPosition());
}

bool PlayerCar::OnEvent(const SEvent & event)
{
	if (event.EventType == EET_KEY_INPUT_EVENT) {
		if(this->controller->isKeyDown(KEY_KEY_W)) {
			this->doAccelerate();
		}
		if(this->controller->isKeyDown(KEY_KEY_S)) {
			this->doBrake();
		}
		if(this->controller->isKeyDown(KEY_KEY_A)) {
			this->doTurnLeft();
		}
		if(this->controller->isKeyDown(KEY_KEY_D)) {
			this->doTurnRight();
		}
	}
	return false;
}

