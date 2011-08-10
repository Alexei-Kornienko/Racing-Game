/*
 * main.cpp
 *
 *  Created on: 21.07.2011
 *      Author: Alex
 */

#include "racingGame.h"
#include "GameStateController.h"
IrrlichtDevice * device;

int initWindow() {
	device = createDevice(
		video::EDT_OPENGL,
		dimension2d<u32>(800, 600),
		16,
		false,
		false,
		false,
		0
	);

	if (!device) {
		printf("Failed to create device");
		return 1;
	}

	device->setWindowCaption(L"Asphalt-6 :)");

	return 0;
}

int main() {
	if(initWindow()) {
		return 1;
	}

	GameStateController controller;

	controller.init(device);

	controller.mainMenu();
	controller.mainLoop();

    device->drop();
	return 0;
}
