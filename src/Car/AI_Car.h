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
	AI_Car(GameController * controller, Car * targetCar);
	virtual ~AI_Car();

	void update(dFloat timeSpan);

protected:
	void carDestroyed();
private:
	Car * targetCar;
};

#endif /* AI_CAR_H_ */
