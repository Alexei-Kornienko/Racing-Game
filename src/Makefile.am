# what flags you want to pass to the C compiler & linker
ACLOCAL_AMFLAGS = ${ACLOCAL_FLAGS}
AM_CFLAGS = -O3 -g -Wall -c -fmessage-length=0
# LDFLAGS =
#
# # this lists the binaries to produce, the (non-PHONY, binary) targets in
build_PROGRAMS = RacingGame
RacingGame_SOURCES = \
		main.cpp \
		GameStateController.cpp
BUILT_SOURCES = \
		Car/AI_Car.cpp \
		Car/BaseCar.cpp \
		Car/Car.cpp \
		Car/PlayerCar.cpp \
		Car/Utility/Tire.cpp \
		Car/Utility/TireRayCast.cpp

