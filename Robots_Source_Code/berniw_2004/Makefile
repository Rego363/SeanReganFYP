ROBOT = berniw_2004


MODULE = ${ROBOT}.so

MODULEDIR = drivers/${ROBOT}

SOURCES   = ${ROBOT}.cpp spline.cpp trackdesc.cpp pathfinder.cpp mycar.cpp

SHIPDIR   = drivers/${ROBOT}

SHIP      = ${ROBOT}.xml logo.rgb

SHIPSUBDIRS = 1 2

PKGSUBDIRS	= $(SHIPSUBDIRS)

src-robots-berniw_2004_PKGFILES	= $(shell find * -maxdepth 0 -type f -print)

src-robots-berniw_2004_PKGDIR	= ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

# Mandatory Line Do Not Remove
include ${MAKE_DEFAULT}

INCLUDE = -I"/usr/src/torcs/torcs-1.3.7/src/drivers/berniw_2004/libtorch/include"/
	
