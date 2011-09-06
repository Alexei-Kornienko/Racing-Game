/*
 * GameStateController.cpp
 *
 *  Created on: 22.07.2011
 *      Author: Alex
 */

#include "GameStateController.h"
#include "PlayerCar.h"
#include "AI_Car.h"

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
	u32 size = this->eventReceivers.size();
	bool result = false;
	for(u32 i=0; i<size && !result; i++) {
		result = this->eventReceivers[i]->OnEvent(event);
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
	this->lastFPS = -1;

	this->car = 0;
	this->nWorld = 0;

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
			(s32)(size.Width>>1),
			(s32)(size.Height>>1)
		);
		if(this->car) {
			// FIXME possible issue
			((PlayerCar*)this->car)->getCamera()->setInputReceiverEnabled(true);
		}

		this->guienv->clear();
	} else { // Pause (show pause menu)
		this->paused = true;
		if(!this->timer->isStopped()) {
			this->timer->stop();
		}
		if(this->car) {
			// FIXME possible issue
			((PlayerCar*)this->car)->getCamera()->setInputReceiverEnabled(false);
		}

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
	this->releaseCars();
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



ISceneManager *GameStateController::getSmgr() const
{
    return smgr;
}

bool GameStateController::addEventReceiver(IEventReceiver * receiver)
{
	if(this->eventReceivers.binary_search(receiver) == -1) {
		this->eventReceivers.push_back(receiver);
		return true;
	}
	return false;
}

bool GameStateController::removeEventReceiver(IEventReceiver * receiver)
{
	s32 index = this->eventReceivers.binary_search(receiver);
	if(index != -1) {
		this->eventReceivers.erase(index);
		return true;
	}
	return false;
}

void GameStateController::setSmgr(ISceneManager *smgr)
{
    this->smgr = smgr;
}

void GameStateController::update()
{
	u32 timeSpan = this->timer->getTime() - this->lastUpdate;
	float seconds = timeSpan / 1000.f;
	if(seconds < this->updateInterval) {
		return;
	}

	if(this->car) {
		NewtonUpdate(this->nWorld,1.0f/this->driver->getFPS());
//		NewtonCreate
		this->car->update(timeSpan);
		u32 size = this->aiCars.size();
		for(u32 i =0; i<size; i++) {
			this->aiCars[i]->update(timeSpan);
		}
	}

	this->lastUpdate = this->timer->getTime();
}



void GameStateController::mainLoop()
{
	while(this->device->run()) {
		this->update();
		this->driver->beginScene(true, true, SColor(255,100,101,140));
		this->smgr->drawAll();
		this->guienv->drawAll();
		this->driver->endScene();

		u32 fps = this->driver->getFPS();
		if(this->lastFPS != fps) {
			stringw title(L"Flatout :) - FPS:");
			title += stringw(fps);
			this->device->setWindowCaption(title.c_str());
			this->lastFPS = fps;
		}
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

	this->smgr->addSkyDomeSceneNode(
		this->driver->getTexture("res/skydome.jpg"),
		16,
		8,
		0.75f,
		1.2f,
		90.f
	);

	IAnimatedMesh * floor = this->smgr->getMesh("res/floor.obj");
	this->smgr->addAnimatedMeshSceneNode(floor); // TODO create normal level

	device->getCursorControl()->setVisible(false);

	this->nWorld = NewtonCreate();

	// set a fix world size
	dVector minSize (-500.0f, 0.f, -500.0f);
	dVector maxSize ( 500.0f,  50.0f,  500.0f);
	NewtonSetWorldSize(this->nWorld, &minSize[0], &maxSize[0]);

	// configure the Newton world to use iterative solve mode 0
	// this is the most efficient but the less accurate mode
	NewtonSetSolverModel(this->nWorld, 1);

	// set the adaptive friction model for faster speed
	NewtonSetFrictionModel(this->nWorld, 1);

	//this->releaseCars();
	this->car = new PlayerCar(this);
	for(u32 i =0; i<AI_COUNT; i++) {
		this->aiCars.push_back(new AI_Car(this, this->car));
	}

	device->getCursorControl()->setVisible(false);

	this->timer->start();
}



void GameStateController::mainMenu()
{
	this->smgr->clear();
	this->guienv->clear();

	this->releaseCars();

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

NewtonWorld *GameStateController::getWorld() const
{
    return nWorld;
}

void GameStateController::releaseCars()
{
	if(this->nWorld) {
		NewtonDestroyAllBodies(this->nWorld);
		NewtonDestroy(this->nWorld);
		this->nWorld = 0;
	}

	if(this->car) {
		delete this->car;
		this->car = 0;
	}
	u32 size = this->aiCars.size();
	for(u32 i =0; i<size; i++) {
		if(this->aiCars[i]) {
			delete this->aiCars[i];
		}
	}
	this->aiCars.erase(0, size);
}



