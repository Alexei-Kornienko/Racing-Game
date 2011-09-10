/*
 * AI_Car.h
 *
 *  Created on: Sep 5, 2011
 *      Author: alex
 */

#ifndef AI_CAR_H_
#define AI_CAR_H_

#include "Car.h"

class AI_Car : public Car {
public:
	AI_Car(GameStateController * controller, Car * targetCar);
	virtual ~AI_Car();

	virtual void update(dFloat timeSpan);

protected:

private:
	Car * targetCar;
};

#endif /* AI_CAR_H_ */
