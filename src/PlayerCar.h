/*
 * PlayerCar.h
 *
 *  Created on: Sep 5, 2011
 *      Author: alex
 */

#ifndef PLAYERCAR_H_
#define PLAYERCAR_H_

#include "Car.h"

class GameStateController;
class PlayerCar : public Car, public IEventReceiver {
public:
	PlayerCar(GameStateController * controller);
	virtual ~PlayerCar();
	virtual void update(u32 timeSpan);

	bool OnEvent(const SEvent& event);

protected:

private:
	ICameraSceneNode * camera;
};

#endif /* PLAYERCAR_H_ */
