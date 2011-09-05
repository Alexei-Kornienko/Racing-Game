/*
 * Controller.h
 *
 *  Created on: 21.07.2011
 *      Author: Alex
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "racingGame.h"
#include "Car.h"

class Car;
class GameStateController : public IEventReceiver {
public:
	GameStateController();
	virtual ~GameStateController();

	void init(IrrlichtDevice * device);

	void mainLoop();

	bool addEventReceiver(IEventReceiver * receiver);
	bool removeEventReceiver(IEventReceiver * receiver);
	bool OnEvent(const SEvent& event);
	bool isKeyDown(EKEY_CODE keyCode) const;
	// Game states
	void newGame();
	void mainMenu();
	void pause();
	void exit();
    ISceneManager *getSmgr() const;


protected:
	u32 lastUpdate;
	u32 updateInterval;

	void update(u32 timeSpan);
	void setSmgr(ISceneManager *smgr);
private:
	IrrlichtDevice * device;
	IVideoDriver * driver;
	ISceneManager * smgr;
	IGUIEnvironment * guienv;

	ITimer * timer;
	bool paused;

	core::array<IEventReceiver*> eventReceivers;

	Car * car;

	// We use this array to store the current state of each key
	bool keysDown[KEY_KEY_CODES_COUNT];

};

#endif /* CONTROLLER_H_ */
