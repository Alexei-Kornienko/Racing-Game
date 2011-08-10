/*
 * Controller.h
 *
 *  Created on: 21.07.2011
 *      Author: Alex
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "racingGame.h"

class GameStateController : public IEventReceiver {
public:
	GameStateController();
	virtual ~GameStateController();

	void init(IrrlichtDevice * device);

	void mainLoop();

	bool OnEvent(const SEvent& event);
	bool isKeyDown(EKEY_CODE keyCode) const;
	// Game states
	void newGame();
	void mainMenu();
	void pause();
	void exit();

protected:
	u32 lastUpdate;
	u32 updateInterval;

	void update(u32 timeSpan);

private:
	IrrlichtDevice * device;
	IVideoDriver * driver;
	ISceneManager * smgr;
	IGUIEnvironment * guienv;

	ITimer * timer;
	bool paused;

	ICameraSceneNode * camera;

	// We use this array to store the current state of each key
	bool keysDown[KEY_KEY_CODES_COUNT];

};

#endif /* CONTROLLER_H_ */
