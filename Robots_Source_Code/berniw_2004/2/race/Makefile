##############################################################################
#
#    file                 : Makefile
#    created              : Wed Jan 8 18:31:16 CET 2003
#    copyright            : (C) 2002 Bernhard Wymann
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = berniw_2004
SHIPDIR     = drivers/${ROBOT}/2/race
SHIP        = $(shell find *.xml -maxdepth 0 -type f -print)

src-robots-berniw_2004_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-berniw_2004_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))
include ${MAKE_DEFAULT}

