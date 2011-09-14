/*
 * GameStateController.cpp
 *
 *  Created on: 22.07.2011
 *      Author: Alex
 */

#include "GameStateController.h"
#include "Car/PlayerCar.h"
#include "Car/AI_Car.h"

bool GameController::OnEvent(const SEvent & event)
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

	return false;
}

// This is used to check whether a key is being held down
bool GameController::isKeyDown(EKEY_CODE keyCode) const
{
	return this->keysDown[keyCode];
}

void GameController::exit()
{
	if(!this->timer->isStopped()) {
		this->timer->stop();
	}
	this->device->closeDevice();
}

GameController::GameController(IrrlichtDevice *device)
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

	this->init(device);
}

void GameController::pause()
{
	dimension2du size = this->driver->getScreenSize();

	if(this->paused) { // Resume
		this->paused = false;
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

GameController::~GameController()
{
	this->releaseCars();
}

void GameController::init(IrrlichtDevice *device)
{
	this->device = device;
	this->driver = device->getVideoDriver();
	SColorf color(1,1,1);
	this->driver->setAmbientLight(color);

	this->smgr = device->getSceneManager();
	this->guienv = device->getGUIEnvironment();
	this->timer = device->getTimer();

	this->device->setEventReceiver(this);
	this->guienv->setUserEventReceiver(this);
}

ISceneManager *GameController::getSmgr() const
{
    return smgr;
}

void GameController::update(u32 timeSpan)
{
	if(this->paused) {
		return;
	}
	if(this->nWorld) {
		NewtonUpdate(this->nWorld,1.0f/this->driver->getFPS());
	}
}

void GameController::updateFPS()
{
    u32 fps = this->driver->getFPS();
    if(this->lastFPS != fps){
        stringw title(L"Flatout :) - FPS:");
        title += stringw(fps);
        this->device->setWindowCaption(title.c_str());
        this->lastFPS = fps;
    }
}

void GameController::mainLoop()
{
	this->timer->start();
	while(this->device->run()) {
		u32 timeSpan = this->timer->getTime() - this->lastUpdate;

		this->device->sleep(20); // FIXME

		this->driver->beginScene(true, true, SColor(255,100,101,140));
		this->smgr->drawAll();
		this->guienv->drawAll();
		this->driver->endScene();

		this->update(timeSpan);

		this->updateFPS();

		this->lastUpdate = this->timer->getTime();
	}
}

void GameController::createGameLevel()
{
    IAnimatedMesh *floor = this->smgr->getMesh("res/floor.obj");
    float scale = 1.0;
    vector3df floorScale(scale, scale, scale);
    IAnimatedMeshSceneNode *floorNode = this->smgr->addAnimatedMeshSceneNode(
    	floor, 0, 0,
    	vector3df(0, 0, 0),
    	vector3df(0, 0, 0),
    	floorScale
    );

    vector3df v1 = floorNode->getBoundingBox().MinEdge * scale;
    vector3df v2 = floorNode->getBoundingBox().MaxEdge * scale;
    dVector minBox(v1.X, v1.Y, v1.Z);
    dVector maxBox(v2.X, v2.Y, v2.Z);
    dVector size(maxBox - minBox);
    dVector origin((maxBox + minBox).Scale(0.5f));
    float floorPlaneWidth = 0.1f;
    size.m_y = floorPlaneWidth;
    size.m_w = 1.0f;
    origin.m_w = 1.0f;
    dMatrix offset(GetIdentityMatrix());
    offset.m_posit = origin;
    NewtonCollision *collision = NewtonCreateBox(this->nWorld, size.m_x, size.m_y, size.m_z, 0, &offset[0][0]);
    this->createFloorBody(collision, floorNode, origin);

    IAnimatedMeshSceneNode *wall1 = this->smgr->addAnimatedMeshSceneNode(
    	floor, 0, 0,
    	vector3df(0, 3, 18),
    	vector3df(-20, 0, 0),
    	floorScale
    );
    this->createFloorBody(collision, wall1, origin);

    IAnimatedMeshSceneNode *wall2 = this->smgr->addAnimatedMeshSceneNode(
    	floor, 0, 0,
    	vector3df(0, 3, -18),
    	vector3df(20, 0, 0),
    	floorScale
    );
    this->createFloorBody(collision, wall2, origin);

    IAnimatedMeshSceneNode *wall3 = this->smgr->addAnimatedMeshSceneNode(
    	floor, 0, 0,
    	vector3df(18, 3, 0),
    	vector3df(0, 0, 20),
    	floorScale
    );
    this->createFloorBody(collision, wall3, origin);

    IAnimatedMeshSceneNode *wall4 = this->smgr->addAnimatedMeshSceneNode(
    	floor, 0, 0,
    	vector3df(-18, 3, 0),
    	vector3df(0, 0, -20),
    	floorScale
    );
    this->createFloorBody(collision, wall4, origin);
}

void GameController::newGame()
{
	this->clearScene();

	this->timer->setTime(0);

	this->smgr->addLightSceneNode(0, vector3df(10,10,10), SColorf(1,1,1),10);
	this->smgr->addLightSceneNode(0, vector3df(10,10,-10), SColorf(1,1,1),10);
	this->smgr->addLightSceneNode(0, vector3df(-10,10,10), SColorf(1,1,1),10);
	this->smgr->addLightSceneNode(0, vector3df(-10,10,-10), SColorf(1,1,1),10);

	device->getCursorControl()->setVisible(false);

	this->smgr->addSkyDomeSceneNode(
		this->driver->getTexture("res/skydome.jpg"),
		16,
		8,
		0.75f,
		1.2f,
		90.f
	);

	this->nWorld = NewtonCreate();

	// set a fixed world size
	dVector minSize (-500.0f, -10.f, -500.0f);
	dVector maxSize ( 500.0f,  30.0f,  500.0f);
	NewtonSetWorldSize(this->nWorld, &minSize[0], &maxSize[0]);

	// configure the Newton world to use iterative solve mode 0
	// this is the most efficient but the less accurate mode
	NewtonSetSolverModel(this->nWorld, 1);

	NewtonSetFrictionModel(this->nWorld, 1);
    this->createGameLevel(); // TODO create normal level

	this->car = new PlayerCar(this);
	for(u32 i =0; i<AI_COUNT; i++) {
		this->aiCars.push_back(new AI_Car(this, this->car));
	}

	device->getCursorControl()->setVisible(false);
	this->paused = false;
}

void GameController::gameOver(bool win)
{
	this->paused = true;
	if(this->car) {
		// FIXME possible issue
		((PlayerCar*)this->car)->getCamera()->setInputReceiverEnabled(false);
	}
	dimension2du size = this->driver->getScreenSize();
	device->getCursorControl()->setVisible(true);
	this->guienv->clear();

	ITexture * texture;
	if(win) {
		texture = this->driver->getTexture("res/win.jpg");
	} else {
		texture = this->driver->getTexture("res/lose.jpg");
	}

	this->guienv->addImage(
		texture,
		position2di(size.Width * 0.2f, size.Height * 0.10f)
	);

	s32 left = size.Width * 0.2f;
	s32 top = size.Height * 0.40f;
	size.Width *= 0.6f;
	size.Height *= 0.2f;

	position2di pos(left,top);

	this->guienv->addButton(
		rect<s32>(pos, size),
		0,
		MENU_NEW_GAME,
		L"New Game",
		L"Start a new game"
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

void GameController::aiDeath(Car *car)
{
	for(int i=0, c=this->aiCars.size(); i<c; i++) {
		if(car == this->aiCars[i]) {
			delete this->aiCars[i];
			this->aiCars[i] = 0;
			this->aiCars.erase(i);
			break;
		}
	}
	if(this->aiCars.size() == 0) {
		this->gameOver(true);
	}
}

void GameController::createFloorBody(NewtonCollision *collision, IAnimatedMeshSceneNode *floorNode, dVector origin)
{
    NewtonBody *floorBody = NewtonCreateBody(nWorld, collision, floorNode->getRelativeTransformation().pointer());
    NewtonBodySetUserData(floorBody, 0);
    dVector inertia;
    NewtonConvexCollisionCalculateInertialMatrix(collision, &inertia[0], &origin[0]);
    NewtonBodySetMassMatrix(floorBody, 0, 0, 0, 0);
    NewtonBodySetCentreOfMass(floorBody, &origin[0]);
}

void GameController::clearScene()
{
    this->smgr->clear();
    this->guienv->clear();
}

void GameController::mainMenu()
{
	this->releaseCars();
    this->clearScene();

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

NewtonWorld *GameController::getWorld() const
{
    return nWorld;
}

void GameController::releaseCars()
{
	if(this->car) {
		delete this->car;
		this->car = 0;
	}
	u32 size = this->aiCars.size();
	for(u32 i =0; i<size; i++) {
		if(this->aiCars[i]) {
			delete this->aiCars[i];
			this->aiCars[i] = 0;
		}
	}
	this->aiCars.clear();
	if(this->nWorld) {
		NewtonDestroyAllBodies(this->nWorld);
		NewtonDestroy(this->nWorld);
		this->nWorld = 0;
	}
}



