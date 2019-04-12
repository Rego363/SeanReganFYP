##############################################################################
#
#    file                 : Makefile
#    created              : Sat Mar 18 23:11:09 CET 2000
#    copyright            : (C) 2000 by Eric Espie                         
#    email                : torcs@free.fr   
#    version              : $Id: Makefile,v 1.7 2002/06/30 20:20:07 torcs Exp $                                  
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################


ROBOT = berniw_2004

SHIPDIR   = drivers/${ROBOT}/1

SHIP      = car5-trb1.rgb default.xml

SHIPSUBDIRS = practice qualifying race

src-robots-berniw_2004_PKGFILES	= $(shell find * -maxdepth 0 -type f -print)

src-robots-berniw_2004_PKGDIR	= ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}




