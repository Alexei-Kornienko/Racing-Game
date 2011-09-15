# what flags you want to pass to the C compiler & linker
ACLOCAL_AMFLAGS = ${ACLOCAL_FLAGS}
AM_CFLAGS = -O3 -g -Wall -c -fmessage-length=0
# LDFLAGS =
#
# # this lists the binaries to produce, the (non-PHONY, binary) targets in
myexecdir = $(abs_top_builddir)/@PACKAGE@
myexec_PROGRAMS = RacingGame
RacingGame_SOURCES = \
		main.cpp \
		GameStateController.cpp \
		AI_Car.cpp \
		BaseCar.cpp \
		Car.cpp \
		PlayerCar.cpp \
		Tire.cpp \
		TireRayCast.cpp

