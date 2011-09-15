#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.67])
AC_INIT([RacingGame], [0.4], [alexei.kornienko@gmail.com])
AM_INIT_AUTOMAKE([RacingGame], [0.4])
AC_CONFIG_SRCDIR([src/main.cpp])
AC_CONFIG_HEADERS([config.h])

# Checks for programs.
AC_PROG_CXX()
AC_LANG(C++)

# Checks for libraries.
LDFLAGS+=" -lGL"
AC_CHECK_LIB([GL],[glBegin],[],AC_MSG_ERROR([Missing Irrlicht lib. Try option --with-irrlicht-lib]))
LDFLAGS+=" -lGLU"
AC_CHECK_LIB([GLU],[gluNewQuadric],[],AC_MSG_ERROR([Missing Irrlicht lib. Try option --with-irrlicht-lib]))

LDFLAGS+=" -lIrrlicht"
AC_ARG_WITH([irrlicht-lib],AS_HELP_STRING([--with-irrlicht-lib	path to irrliht lib file]),
	[LDFLAGS+=" -L$withval"]
)
AC_CHECK_LIB(
	[Irrlicht], 
	[createDevice],
   	[],
	AC_MSG_ERROR([Missing Irrlicht lib. Try option --with-irrlicht-lib])
)

LDFLAGS+=" -lpthread"
AC_CHECK_LIB(
	[pthread], 
	[pthread_create],
   	[],
	AC_MSG_ERROR([Missing Pthreads lib])
)

LDFLAGS+=" -lNewton"
AC_ARG_WITH([newton-lib],AS_HELP_STRING([--with-newton-lib	path to Newton lib file]),
	[LDFLAGS+=" -L$withval"]
)
AC_CHECK_LIB(
	[Newton], 
	[NewtonCreate],
   	[],
	AC_MSG_ERROR([Missing Newton lib. Try option --with-newton-lib])
)

LDFLAGS+=" -ldMath"
AC_ARG_WITH([dmath-lib],AS_HELP_STRING([--with-dmath-lib	path to dMath lib file]),
	[LDFLAGS+=" -L$withval"]
)
AC_CHECK_LIB(
	[dMath], 
	[main],
   	[],
	AC_MSG_ERROR([Missing dMath lib. Try option --with-dmath-lib])
)

# Checks for header files.
CXXFLAGS+=" -I$srcdir/src"
CPPFLAGS+=" -I$srcdir/src"
AC_ARG_WITH([irrlicht-include],AS_HELP_STRING([--with-irrlicht-include	path to irrliht headers]),
	[CXXFLAGS+=" -I$withval"; CPPFLAGS+=" -I$withval" ]
)
AC_CHECK_HEADERS([irrlicht.h], [], AC_MSG_ERROR([Missing irrlicht.h. Try option --with-irrlicht-include]) )

AC_ARG_WITH([newton-include],AS_HELP_STRING([--with-newton-include	path to Newton headers]),
	[CXXFLAGS+=" -I$withval"; CPPFLAGS+=" -I$withval" ]
)
AC_CHECK_HEADERS([Newton.h], [], AC_MSG_ERROR([Missing Newton.h. Try option. --with-newton-include]) )

AC_ARG_WITH([dmath-include],AS_HELP_STRING([--with-dmath-include	path to dMath headers]),
	[CXXFLAGS+=" -I$withval"; CPPFLAGS+=" -I$withval" ]
)
AC_CHECK_HEADERS([dMathDefines.h], [], AC_MSG_ERROR([Missing dMath headers. Try option --with-dmath-include]) )
AC_CHECK_HEADERS([dVector.h], [], AC_MSG_ERROR([Missing dMath headers. Try option --with-dmath-include]) )
AC_CHECK_HEADERS([dMatrix.h], [], AC_MSG_ERROR([Missing dMath headers. Try option --with-dmath-include]) )
AC_CHECK_HEADERS([dQuaternion.h], [], AC_MSG_ERROR([Missing dMath headers. Try option --with-dmath-include]) )

AC_CHECK_HEADERS([racingGame.h], [], AC_MSG_ERROR([Missing racingGame.h. Try to add src/ dir to include path]) )

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL

AC_OUTPUT(Makefile src/Makefile res/Makefile)
