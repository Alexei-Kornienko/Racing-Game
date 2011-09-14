/*
 * main.cpp
 *
 *  Created on: 21.07.2011
 *      Author: Alex
 */

#include "racingGame.h"
#include "GameStateController.h"

IrrlichtDevice * initWindow(video::E_DRIVER_TYPE type) {
	IrrlichtDevice * device = createDevice(
		type,
		dimension2d<u32>(800, 600),
		16,
		false,
		false,
		false,
		0
	);

	if (!device) {
		printf("Failed to create device");
		return 0;
	}

	device->setWindowCaption(L"Flatout :)");

	return device;
}

int main() {
	video::E_DRIVER_TYPE type = video::EDT_SOFTWARE;
	IrrlichtDevice * device = initWindow(type);
	if(!device) {
		return 1;
	}

	GameController controller(device);
	controller.mainMenu();
	controller.mainLoop();

    device->drop();
	return 0;
}
