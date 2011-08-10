/*
 * GameStateController.cpp
 *
 *  Created on: 22.07.2011
 *      Author: Alex
 */

#include "GameStateController.h"

#define	MENU_NEW_GAME 1
#define	MENU_OPTIONS 2
#define MENU_EXIT 3

#define PMENU_RESUME 4
#define PMENU_MAIN 5

bool GameStateController::OnEvent(const SEvent & event)
{
    if (event.EventType == irr::EET_KEY_INPUT_EVENT) {
		this->keysDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

		if(!this->paused && (this->isKeyDown(KEY_KEY_P) || this->isKeyDown(KEY_ESCAPE))) {
			this->pause();
			return true;
		}
	}

	if(event.EventType == EET_GUI_EVENT && event.GUIEvent.EventType == EGET_BUTTON_CLICKED) {
		s32 id = event.GUIEvent.Caller->getID();
		switch(id) {
		case MENU_NEW_GAME:
			this->newGame();
			break;
		case MENU_OPTIONS:
			this->pause();
			break;
		case MENU_EXIT:
			this->exit();
			break;
		case PMENU_RESUME:
			this->pause();
			break;
		case PMENU_MAIN:
			this->paused = false;
			this->mainMenu();
			break;
		default:
			return false;
		}
		return true;
	}
	return false;
}

// This is used to check whether a key is being held down
bool GameStateController::isKeyDown(EKEY_CODE keyCode) const
{
		return this->keysDown[keyCode];
}

void GameStateController::exit()
{
	if(!this->timer->isStopped()) {
		this->timer->stop();
	}
	this->device->closeDevice();
}



GameStateController::GameStateController()
{
	this->device = 0;
	this->driver = 0;
	this->guienv = 0;
	this->smgr = 0;
	this->timer = 0;
	this->lastUpdate = 0;

	this->camera = 0;

	this->updateInterval = CONTROLLER_UPDATE_INTERVAL;

	this->paused = false;

	for (u32 i = 0; i < KEY_KEY_CODES_COUNT; ++i)
		this->keysDown[i] = false;
}



void GameStateController::pause()
{
	dimension2du size = this->driver->getScreenSize();

	if(this->paused) { // Resume
		this->paused = false;
		if(this->timer->isStopped()) {
			this->timer->start();
		}
		device->getCursorControl()->setVisible(false);
		device->getCursorControl()->setPosition(
			vector2df(size.Width>>1, size.Height>>1)
		);
		this->camera->setInputReceiverEnabled(true);
		this->guienv->clear();
	} else { // Pause (show pause menu)
		this->paused = true;
		if(!this->timer->isStopped()) {
			this->timer->stop();
		}
		this->camera->setInputReceiverEnabled(false);
		device->getCursorControl()->setVisible(true);
		this->guienv->clear();



		s32 left = size.Width * 0.2f;
		s32 top = size.Height * 0.20f;
		size.Width *= 0.6f;
		size.Height *= 0.2f;

		position2di pos(left,top);

		this->guienv->addButton(
			rect<s32>(pos, size),
			0,
			PMENU_RESUME,
			L"Resume",
			L"Resume game"
		);
		pos.Y += size.Height + (size.Height>>1);
		this->guienv->addButton(
			rect<int>(pos, size),
			0,
			PMENU_MAIN,
			L"Main menu",
			L"Return to main menu"
		);



	}
}



GameStateController::~GameStateController()
{
}



void GameStateController::init(IrrlichtDevice *device)
{
	this->device = device;
	this->driver = device->getVideoDriver();
	this->smgr = device->getSceneManager();
	this->guienv = device->getGUIEnvironment();
	this->timer = device->getTimer();

	this->device->setEventReceiver(this);
	this->guienv->setUserEventReceiver(this);
}



void GameStateController::update(u32 timeSpan)
{
}



void GameStateController::mainLoop()
{
	u32 timeSpan = 0;
	while(this->device->run()) {
		timeSpan = this->timer->getTime() - this->lastUpdate;
//		if(timeSpan < this->updateInterval) {
//			this->device->yield();
//			continue;
//		}
		this->update(timeSpan);

		this->driver->beginScene(true, true, SColor(255,100,101,140));
		this->smgr->drawAll();
		this->guienv->drawAll();
		this->driver->endScene();

		this->lastUpdate = this->timer->getTime();
	}
}



void GameStateController::newGame()
{
	if(!this->timer->isStopped()) {
		this->timer->stop();
	}
	this->timer->setTime(0);
	this->smgr->clear();
	this->guienv->clear();
	//this->driver->

	ISceneNode * skydome = this->smgr->addSkyDomeSceneNode(
		this->driver->getTexture("res/skydome.jpg"),
		64,
		32,
		0.95f,
		1.5f,
		90.f
	);

	IAnimatedMesh * mesh = this->smgr->getMesh("res/car.3DS");
	IAnimatedMeshSceneNode * node = this->smgr->addAnimatedMeshSceneNode(mesh);

//	IAnimatedMesh * wheel = this->smgr->getMesh("res/cleanwheel_01.3DS");
//	this->smgr->addAnimatedMeshSceneNode(
//		wheel,
//		node,
//		-1,
//		vector3df(0.306f, -0.619f, -0.198f)
//	);

	// add camera
	this->camera = this->smgr->addCameraSceneNodeFPS(0,100.0f,0.01f);
	this->camera->setPosition(core::vector3df(0,10,-10));
	this->camera->setTarget(core::vector3df(0,0,0));
	this->camera->setFarValue(100.0f);
	device->getCursorControl()->setVisible(false);

	this->timer->start();


}



void GameStateController::mainMenu()
{
	this->smgr->clear();
	this->guienv->clear();

	dimension2du size = this->driver->getScreenSize();

	s32 left = size.Width * 0.2f;
	s32 top = size.Height * 0.10f;
	size.Width *= 0.6f;
	size.Height *= 0.2f;

	position2di pos(left,top);

	this->guienv->addButton(
		rect<s32>(pos, size),
		0,
		MENU_NEW_GAME,
		L"New Game",
		L"Start new game"
	);
	pos.Y += size.Height + (size.Height>>1);
	this->guienv->addButton(
		rect<int>(pos, size),
		0,
		MENU_OPTIONS,
		L"Options",
		L"Not implemented!"
	);

	pos.Y += size.Height + (size.Height>>1);
	this->guienv->addButton(
		rect<int>(pos, size),
		0,
		MENU_EXIT,
		L"Exit",
		L"Exit the game..."
	);
}

