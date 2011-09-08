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
class PlayerCar : public Car {
public:
	PlayerCar(GameStateController * controller);
	virtual ~PlayerCar();
	virtual void update(dFloat timeSpan, int index);

    ICameraSceneNode *getCamera() const;

protected:

private:
	ICameraSceneNode * camera;
};

#endif /* PLAYERCAR_H_ */
