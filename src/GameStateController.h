/*
 * Controller.h
 *
 *  Created on: 21.07.2011
 *      Author: Alex
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define AI_COUNT 4

#define	MENU_NEW_GAME 1
#define	MENU_OPTIONS 2
#define MENU_EXIT 3

#define PMENU_RESUME 4
#define PMENU_MAIN 5

#include "racingGame.h"

class Car;
class GameController : public IEventReceiver {
public:
	GameController(IrrlichtDevice *device);
	virtual ~GameController();

	void init(IrrlichtDevice * device);

	void mainLoop();

	bool OnEvent(const SEvent& event);
	bool isKeyDown(EKEY_CODE keyCode) const;
	// Game states
	void newGame();
	void mainMenu();
	void pause();
	void exit();
	void gameOver(bool win);

	void aiDeath(Car * car);
    ISceneManager *getSmgr() const;
    NewtonWorld *getWorld() const;

protected:
	u32 lastUpdate;
	u32 updateInterval;

	void update(u32 timeSpan);
private:
	IrrlichtDevice * device;
	IVideoDriver * driver;
	ISceneManager * smgr;
	IGUIEnvironment * guienv;
	void releaseCars();
    void createFloorBody(NewtonCollision *collision, IAnimatedMeshSceneNode *floorNode, dVector origin);
    void createGameLevel();
    void updateFPS();
    void clearScene();

	ITimer * timer;
	bool paused;

	NewtonWorld * nWorld;
	Car * car;
	array<Car *> aiCars;

	// We use this array to store the current state of each key
	bool keysDown[KEY_KEY_CODES_COUNT];

	u32 lastFPS;

};

#endif /* CONTROLLER_H_ */
