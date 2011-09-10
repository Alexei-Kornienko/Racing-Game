/*
 * racingGame.h
 *
 *  Created on: 21.07.2011
 *      Author: Alex
 */

#ifndef RACINGGAME_H_
#define RACINGGAME_H_

#define CONTROLLER_UPDATE_INTERVAL 200

#include <stdio.h>

#include "irrlicht.h"
#include "Newton.h"
#include "dMathDefines.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"

// use scale factor between Newton and Irrlicht
#define NEWTON_TO_IRR 32.0f
#define IRR_TO_NEWTON (1.0f / NEWTON_TO_IRR)

using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

#endif /* RACINGGAME_H_ */
