/***************************************************************************

    file                 : berniw.cpp
    created              : Mon Apr 17 13:51:00 CET 2000
    copyright            : (C) 2000-2006 by Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: berniw.cpp,v 1.37 2003/11/12 01:45:30 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#include "berniw.h"
#include <torch/script.h>

// My Code
const float LOOKAHEAD_CONST = 17.0;    /* [m] Variable for getting target when going around a corner */
const float LOOKAHEAD_FACTOR = 0.33;   /* [1/s] Variable for getting target when going around a corner*/
static tTrack	*curTrack; // Pointer to current track 
static std::shared_ptr<torch::jit::script::Module> module;	// NN model
static float sensors[36];	// Array to collect inputs to be put into model



// Function prototypes.
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation * situation);
static void drive(int index, tCarElt* car, tSituation *situation);
static void newRace(int index, tCarElt* car, tSituation *situation);
static int  InitFuncPt(int index, void *pt);
static int  pitcmd(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
float getClutch(MyCar* myc, tCarElt* car);

static const char* botname[BOTS] = {
	"berniw 2004 1", "berniw 2004 2", "berniw 2004 3", "berniw 2004 4", "berniw 2004 5",
	"berniw 2004 6", "berniw 2004 7", "berniw 2004 8", "berniw 2004 9", "berniw 2004 10"
};

static const char* botdesc[BOTS] = {
	"berniw 2004 1", "berniw 2004 2", "berniw 2004 3", "berniw 2004 4", "berniw 2004 5",
	"berniw 2004 6", "berniw 2004 7", "berniw 2004 8", "berniw 2004 9", "berniw 2004 10"
};

// Module entry point.
extern "C" int berniw_2004(tModInfo *modInfo)
{
	for (int i = 0; i < BOTS; i++) {
		modInfo[i].name = strdup(botname[i]);	// Name of the module (short).
		modInfo[i].desc = strdup(botdesc[i]);	// Description of the module (can be long).
		modInfo[i].fctInit = InitFuncPt;		// Init function.
		modInfo[i].gfId    = ROB_IDENT;			// Supported framework version.
		modInfo[i].index   = i+1;
	}
	return 0;
}


// Initialize function (callback) pointers for torcs.
static int InitFuncPt(int index, void *pt)
{
	tRobotItf *itf = (tRobotItf *)pt;

	itf->rbNewTrack = initTrack;	// Init new track.
	itf->rbNewRace  = newRace;		// Init new race.
	itf->rbDrive    = drive;		// Drive during race.
	itf->rbShutdown	= shutdown;		// Called for cleanup per driver.
	itf->rbPitCmd   = pitcmd;		// Pit command.
	itf->index      = index;
	return 0;
}


static MyCar* mycar[BOTS] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };
static OtherCar* ocar = NULL;
static TrackDesc* myTrackDesc = NULL;
static double currenttime;
static const tdble waitToTurn = 1.0; // How long should i wait till i try to turn backwards.


// Release resources when the module gets unloaded.
static void shutdown(int index) {
	int i = index - 1;
	if (mycar[i] != NULL) {
		delete mycar[i];
		mycar[i] = NULL;
	}
	if (myTrackDesc != NULL) {
		delete myTrackDesc;
		myTrackDesc = NULL;
	}
	if (ocar != NULL) {
		delete [] ocar;
		ocar = NULL;
	}
}


// Initialize track data, called for every selected driver.
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation * situation)
{	
	// My code 
	// Get track
	curTrack = track;





	if ((myTrackDesc != NULL) && (myTrackDesc->getTorcsTrack() != track)) {
		delete myTrackDesc;
		myTrackDesc = NULL;
	}
	if (myTrackDesc == NULL) {
		myTrackDesc = new TrackDesc(track);
	}

	char buffer[BUFSIZE];
	char* trackname = strrchr(track->filename, '/') + 1;

	switch (situation->_raceType) {
		case RM_TYPE_PRACTICE:
			snprintf(buffer, BUFSIZE, "drivers/berniw_2004/%d/practice/%s", index, trackname);
			break;
		case RM_TYPE_QUALIF:
			snprintf(buffer, BUFSIZE, "drivers/berniw_2004/%d/qualifying/%s", index, trackname);
			break;
		case RM_TYPE_RACE:
			snprintf(buffer, BUFSIZE, "drivers/berniw_2004/%d/race/%s", index, trackname);
			break;
		default:
			break;
	}

	*carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
	if (*carParmHandle == NULL) {
		snprintf(buffer, BUFSIZE, "drivers/berniw_2004/%d/default.xml", index);
		*carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
    }

	// Load and set parameters.
	float fuel = GfParmGetNum(*carParmHandle, BERNIW_SECT_PRIV, BERNIW_ATT_FUELPERLAP,
		(char*)NULL, track->length*MyCar::MAX_FUEL_PER_METER);
	//printf("fuelperlap: %f\n", fuel);

	float fuelmargin = (situation->_raceType == RM_TYPE_RACE) ? 1.0 : 0.0;

// Temporary fix
float tankvol = GfParmGetNum(carHandle, SECT_CAR, PRM_TANK, (char*)NULL, 50);	
if (index == 1 && situation->_raceType == RM_TYPE_RACE)
fuel = (tankvol - 10.0);
else
fuel *= (situation->_totLaps + fuelmargin);

	//fuel *= (situation->_totLaps + fuelmargin);
	GfParmSetNum(*carParmHandle, SECT_CAR, PRM_FUEL, (char*)NULL, MIN(fuel, 100.0));
}


// Initialize driver for the race, called for every selected driver.
static void newRace(int index, tCarElt* car, tSituation *situation)
{	
	// My code
	// Load the NN model.
	//module = torch::jit::load("/usr/src/torcs/torcs-1.3.7/src/drivers/berniw_2004/model.pt");	// Uncomment this and the robots won't load into the race but it will compile
	





	if (ocar != NULL) {
		delete [] ocar;
	}
	ocar = new OtherCar[situation->_ncars];
	for (int i = 0; i < situation->_ncars; i++) {
		ocar[i].init(myTrackDesc, situation->cars[i], situation);
	}

	if (mycar[index-1] != NULL) {
		delete mycar[index-1];
	}
	mycar[index-1] = new MyCar(myTrackDesc, car, situation);

	currenttime = situation->currentTime;
}

/* Compute the length to the end of the segment */
float getDistToSegEnd(tCarElt* car)
{
	if (car->_trkPos.seg->type == TR_STR) {
		return car->_trkPos.seg->length - car->_trkPos.toStart;
	} else {
		return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
}

static v2d getTargetPoint(tCarElt* car)
{
	tTrackSeg *seg = car->_trkPos.seg;
	float lookahead = LOOKAHEAD_CONST + car->_speed_x*LOOKAHEAD_FACTOR;
	float length = getDistToSegEnd(car);
	while (length < lookahead)
	{		
	        seg = seg->next;
        	length += seg->length;	
	}
	length = lookahead - length + seg->length;
	v2d s;
	s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0;
	s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0;
	if ( seg->type == TR_STR) 
	{
		v2d d;
		d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
		d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
		return s + d*length;
	}
	else 
	{
		v2d c;
		c.x = seg->center.x;
		c.y = seg->center.y;
		float arc = length/seg->radius;
		float arcsign = (seg->type == TR_RGT) ? -1 : 1;
		arc = arc*arcsign;
		return s.rotate(c, arc);
	}
}

/* compute speed component parallel to the track */
static float getSpeed(tCarElt *car)
{
	v2d speed, dir;
	float trackangle = RtTrackSideTgAngleL(&(car->_trkPos));

	speed.x = car->_speed_X;
	speed.y = car->_speed_Y;
	dir.x = cos(trackangle);
	dir.y = sin(trackangle);
	return speed*dir;
}

// Controls the car.
static void drive(int index, tCarElt* car, tSituation *situation)
{	
	
	// Update some values needed.
	
	tdble angle;
	tdble brake;
	tdble b1;							// Brake value in case we are to fast HERE and NOW.
	tdble b2;							// Brake value for some brake point in front of us.
	tdble b3;							// Brake value for control (avoid loosing control).
	tdble b4;							// Brake value for avoiding high angle of attack.
	tdble b5;							// Brake for teh pit.
	tdble steer, targetAngle, shiftaccel;

	MyCar* myc = mycar[index-1];
	Pathfinder* mpf = myc->getPathfinderPtr();

	b1 = b2 = b3 = b4 = b5 = 0.0;
	shiftaccel = 0.0;


	// Decide how we want to drive, choose a behaviour.
	if ( car->_dammage < myc->undamaged/3 && myc->bmode != myc->NORMAL) {
		myc->loadBehaviour(myc->NORMAL);
	} else if (car->_dammage > myc->undamaged/3 && car->_dammage < (myc->undamaged*2)/3 && myc->bmode != myc->CAREFUL) {
		myc->loadBehaviour(myc->CAREFUL);
	} else if (car->_dammage > (myc->undamaged*2)/3 && myc->bmode != myc->SLOW) {
		myc->loadBehaviour(myc->SLOW);
	}

	// Update the other cars just once.
	if (currenttime != situation->currentTime) {
		currenttime = situation->currentTime;
		for (int i = 0; i < situation->_ncars; i++) ocar[i].update();
	}

	// Startmode.
	if (myc->trtime < 5.0 && myc->bmode != myc->START) {
		myc->loadBehaviour(myc->START);
		myc->startmode = true;
	}
	if (myc->startmode && myc->trtime > 5.0) {
		myc->startmode = false;
		myc->loadBehaviour(myc->NORMAL);
	}

	// Compute path according to the situation.
	mpf->plan(myc->getCurrentSegId(), car, situation, myc, ocar);

	// Clear ctrl structure with zeros and set the current gear.
	memset(&car->ctrl, 0, sizeof(tCarCtrl));
	car->_gearCmd = car->_gear;

	// Uncommenting the following line causes pitstop on every lap.
	//if (!mpf->getPitStop()) mpf->setPitStop(true, myc->getCurrentSegId());

	// Compute fuel consumption.
	if (myc->getCurrentSegId() >= 0 && myc->getCurrentSegId() < 5 && !myc->fuelchecked) {
		if (car->race.laps > 0) {
			myc->fuelperlap = MAX(myc->fuelperlap, (myc->lastfuel+myc->lastpitfuel-car->priv.fuel));
		}
		myc->lastfuel = car->priv.fuel;
		myc->lastpitfuel = 0.0;
		myc->fuelchecked = true;
	} else if (myc->getCurrentSegId() > 5) {
		myc->fuelchecked = false;
	}

	// Decide if we need a pit stop.
	// First look at the team situation
	int pitcarindex = 0;
	int runningcars = 0;
	tCarElt* carwithlowestfuel = car;
	while (pitcarindex < TR_PIT_MAXCARPERPIT && car->_pit->freeCarIndex > pitcarindex) {
		tCarElt* pitcar = car->_pit->car[pitcarindex];
		if (pitcar->_state & (RM_CAR_STATE_DNF | RM_CAR_STATE_PULLUP | RM_CAR_STATE_PULLSIDE | RM_CAR_STATE_PULLDN | RM_CAR_STATE_NO_SIMU)) {
			// Car is gone, ignore
		} else {
			// Count running cars (for early stop estimate)
			runningcars++;
			// Find car with lowest fuel (TODO: consider position along track)
			if (pitcar->priv.fuel < carwithlowestfuel->priv.fuel) {
				carwithlowestfuel = pitcar;
			}
		}
		pitcarindex++;
	}
	
	// If I am the car with the lowest fuel, I might do an early stop
	
	
	// Then check for the individual 
	if (!mpf->getPitStop() && (car->_remainingLaps-car->_lapsBehindLeader) > 0 && (car->_dammage > myc->MAXDAMMAGE ||
		(car->priv.fuel < (myc->fuelperlap*(1.0+myc->FUEL_SAFETY_MARGIN)) &&
		 car->priv.fuel < (car->_remainingLaps-car->_lapsBehindLeader)*myc->fuelperlap)))
	{
		mpf->setPitStop(true, myc->getCurrentSegId());
	}

	if (mpf->getPitStop()) {
		car->_raceCmd = RM_CMD_PIT_ASKED;
	}

	// Steer toward the next target point.
	targetAngle = atan2(myc->dynpath->getLoc(myc->destpathsegid)->y - car->_pos_Y, myc->dynpath->getLoc(myc->destpathsegid)->x - car->_pos_X);
	targetAngle -= car->_yaw;
	NORM_PI_PI(targetAngle);
    steer = targetAngle / car->_steerLock;

	// Steer P (proportional) controller. We add a steer correction proportional to the distance error
	// to the path.
	// Steer angle has usual meaning, therefore + is to left (CCW) and - to right (CW).
	// derror sign is + to right and - to left.
	if (!mpf->getPitStop()) {
		steer = steer + MIN(myc->STEER_P_CONTROLLER_MAX, myc->derror*myc->STEER_P_CONTROLLER_GAIN)*myc->getErrorSgn();
		if (fabs(steer) > 1.0) {
			steer/=fabs(steer);
		}
	} else {
		tdble dl, dw;
		RtDistToPit(car, myTrackDesc->getTorcsTrack(), &dl, &dw);
		if (dl < 2.0f) {
			b5 = 1.0f;
		}
	}

	// Try to control angular velocity with a D (differential) controller.
	double omega = myc->getSpeed()/myc->dynpath->getRadius(myc->currentpathsegid);
	steer += myc->STEER_D_CONTROLLER_GAIN*(omega - myc->getCarPtr()->_yaw_rate);


	// Brakes.
    tdble brakecoeff = 1.0/(2.0*g*myc->currentseg->getKfriction()*myc->CFRICTION);
    tdble brakespeed, brakedist;
	tdble lookahead = 0.0;
	int i = myc->getCurrentSegId();
	brake = 0.0;

	while (lookahead < brakecoeff * myc->getSpeedSqr()) {
		lookahead += myc->dynpath->getLength(i);
		brakespeed = myc->getSpeedSqr() - myc->dynpath->getSpeedsqr(i);
		if (brakespeed > 0.0) {
			tdble gm, qb, qs;
			gm = myTrackDesc->getSegmentPtr(myc->getCurrentSegId())->getKfriction()*myc->CFRICTION*myTrackDesc->getSegmentPtr(myc->getCurrentSegId())->getKalpha();
			qs = myc->dynpath->getSpeedsqr(i);

			brakedist = brakespeed*(myc->mass/(2.0*gm*g*myc->mass + qs*(gm*myc->ca + myc->cw)));

			if (brakedist > lookahead - myc->getWheelTrack()) {
				qb = brakespeed*brakecoeff/brakedist;
				if (qb > b2) {
					b2 = qb;
				}
			}
		}
		i = (i + 1 + mpf->getnPathSeg()) % mpf->getnPathSeg();
	}

	if (myc->getSpeedSqr() > myc->dynpath->getSpeedsqr(myc->currentpathsegid)) {
		b1 = (myc->getSpeedSqr() - myc->dynpath->getSpeedsqr(myc->currentpathsegid)) / (myc->getSpeedSqr());
	}

	// Try to avoid flying.
	if (myc->getDeltaPitch() > myc->MAXALLOWEDPITCH && myc->getSpeed() > myc->FLYSPEED) {
		b4 = 1.0;
	}

	// Check if we are on the way.
	if (myc->getSpeed() > myc->TURNSPEED && myc->tr_mode == 0) {
		if (myc->derror > myc->PATHERR) {
			vec2d *cd = myc->getDir();
			vec2d *pd = myc->dynpath->getDir(myc->currentpathsegid);
			float z = cd->x*pd->y - cd->y*pd->x;
			// If the car points away from the path brake.
			if (z*myc->getErrorSgn() >= 0.0) {
				targetAngle = atan2(myc->dynpath->getDir(myc->currentpathsegid)->y, myc->dynpath->getDir(myc->currentpathsegid)->x);
				targetAngle -= car->_yaw;
				NORM_PI_PI(targetAngle);
				double toborder = MAX(1.0, myc->currentseg->getWidth()/2.0 - fabs(myTrackDesc->distToMiddle(myc->getCurrentSegId(), myc->getCurrentPos())));
				b3 = (myc->getSpeed()/myc->STABLESPEED)*(myc->derror-myc->PATHERR)/toborder;
			}
		}
	}

	// Anti wheel locking and brake code.
	if (b1 > b2) brake = b1; else brake = b2;
	if (brake < b3) brake = b3;
	if (brake < b4) {
		brake = MIN(1.0, b4);
		tdble abs_mean;
		abs_mean = (car->_wheelSpinVel(REAR_LFT) + car->_wheelSpinVel(REAR_RGT))*car->_wheelRadius(REAR_LFT)/myc->getSpeed();
		abs_mean /= 2.0;
    	brake = brake * abs_mean;
	} else {
		brake = MIN(1.0, brake);
		tdble abs_min = 1.0;
		for (int i = 0; i < 4; i++) {
			tdble slip = car->_wheelSpinVel(i) * car->_wheelRadius(i) / myc->getSpeed();
			if (slip < abs_min) abs_min = slip;
		}
    	brake = brake * abs_min;
	}

	// Reduce brake value to the approximate normal force available on the wheels.
	float weight = myc->mass*G;
	float maxForce = weight + myc->ca*myc->MAX_SPEED*myc->MAX_SPEED;
	float force = weight + myc->ca*myc->getSpeedSqr();
	brake = brake*MIN(1.0, force/maxForce);
	if (b5 > 0.0f) {
		brake = b5;
	}


	// Gear changing.
	if (myc->tr_mode == 0) {
		if (car->_gear <= 0) {
			car->_gearCmd =  1;
		} else {
			float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
			float omega = car->_enginerpmRedLine/gr_up;
			float wr = car->_wheelRadius(2);

			if (omega*wr*myc->SHIFT < car->_speed_x) {
				car->_gearCmd++;
			} else {
				float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
				omega = car->_enginerpmRedLine/gr_down;
				if (car->_gear > 1 && omega*wr*myc->SHIFT > car->_speed_x + myc->SHIFT_MARGIN) {
					car->_gearCmd--;
				}
			}
		}
	}


	// Acceleration / brake execution.
	tdble cerror, cerrorh;
	cerrorh = sqrt(car->_speed_x*car->_speed_x + car->_speed_y*car->_speed_y);
	if (cerrorh > myc->TURNSPEED) {
		cerror = fabs(car->_speed_x)/cerrorh;
	} else {
		cerror = 1.0;
	}

	if (myc->tr_mode == 0) {
		if (brake > 0.0) {
			myc->accel = 0.0;
			car->_accelCmd = myc->accel;
			car->_brakeCmd = brake*cerror;
		} else {
			if (myc->getSpeedSqr() < myc->dynpath->getSpeedsqr(myc->getCurrentSegId())) {
				if (myc->accel < myc->ACCELLIMIT) {
					myc->accel += myc->ACCELINC;
				}
				car->_accelCmd = myc->accel/cerror;
			} else {
				if (myc->accel > 0.0) {
					myc->accel -= myc->ACCELINC;
				}
				// TODO: shiftaccel always 0 at the moment...
				car->_accelCmd = myc->accel = MIN(myc->accel/cerror, shiftaccel/cerror);
			}
			tdble slipspeed = myc->querySlipSpeed(car);
			if (slipspeed > myc->TCL_SLIP) {
				car->_accelCmd = car->_accelCmd - MIN(car->_accelCmd, (slipspeed - myc->TCL_SLIP)/myc->TCL_RANGE);
			}
		}
	}


	// Check if we are stuck, try to get unstuck.
	tdble bx = myc->getDir()->x, by = myc->getDir()->y;
	tdble cx = myc->currentseg->getMiddle()->x - car->_pos_X, cy = myc->currentseg->getMiddle()->y - car->_pos_Y;
	tdble parallel = (cx*bx + cy*by) / (sqrt(cx*cx + cy*cy)*sqrt(bx*bx + by*by));

	if ((myc->getSpeed() < myc->TURNSPEED) && (parallel < cos(90.0*PI/180.0))  && (mpf->dist2D(myc->getCurrentPos(), myc->dynpath->getLoc(myc->getCurrentSegId())) > myc->TURNTOL)) {
		myc->turnaround += situation->deltaTime;
	} else myc->turnaround = 0.0;
	if ((myc->turnaround >= waitToTurn) || (myc->tr_mode >= 1)) {
		if (myc->tr_mode == 0) {
			myc->tr_mode = 1;
		}
        if ((car->_gearCmd > -1) && (myc->tr_mode < 2)) {
			car->_accelCmd = 0.0;
			if (myc->tr_mode == 1) {
				car->_gearCmd--;
			}
			car->_brakeCmd = 1.0;
		} else {
			myc->tr_mode = 2;
			if (parallel < cos(90.0*PI/180.0) && (mpf->dist2D(myc->getCurrentPos(), myc->dynpath->getLoc(myc->getCurrentSegId())) > myc->TURNTOL)) {
				angle = queryAngleToTrack(car);
				car->_steerCmd = ( -angle > 0.0) ? 1.0 : -1.0;
				car->_brakeCmd = 0.0;

				if (myc->accel < 1.0) {
					myc->accel += myc->ACCELINC;
				}
				car->_accelCmd = myc->accel;
				tdble slipspeed = myc->querySlipSpeed(car);
				if (slipspeed < -myc->TCL_SLIP) {
					car->_accelCmd = car->_accelCmd - MIN(car->_accelCmd, (myc->TCL_SLIP - slipspeed)/myc->TCL_RANGE);
				}
			} else {
				if (myc->getSpeed() < 1.0) {
					myc->turnaround = 0;
					myc->tr_mode = 0;
					myc->loadBehaviour(myc->START);
					myc->startmode = true;
					myc->trtime = 0.0;
				}
				car->_brakeCmd = 1.0;
				car->_steerCmd = 0.0;
				car->_accelCmd = 0.0;
			}
		}
	}

	if (myc->tr_mode == 0) car->_steerCmd = steer;
	car->_clutchCmd = getClutch(myc, car);
	
	
		
	/*
	// My code 
	std::vector<int> opCars; // Vector to hold opposition cars index in the cars array
	// Loop and get opposition car's indexs
	for(int i = 0; i < situation->_ncars; i++)
	{	
		if(situation->cars[i]->index == car->index)	// Compare current car index to this car's index
		{
			// This car so skip
			continue;
		}
		else
		{
			opCars.push_back(i);	// Opposition
		}
	}	
	
	// Get opCar 1 current speed, distance, direction and steering
	// Initialise variables 
	int curCar = opCars[0];	// Index of first car
	int direction = -1;	// Direction of the first car
	int speed = -1;		// Speed of the first car
	int distance = 200;	// Distance of the first car away from this car
	float steering = 0;	// The current turning value of first car
	// if Distance to this car is less than 200
	if(distance < 200)	
	{
		// Get Speed
		speed = getSpeed(situation->cars[curCar]);
		
		// Direction
		if(car->race.distFromStartLine < situation->cars[curCar]->race.distFromStartLine)
		{
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				// Ahead and Left of this car;
				direction = 0;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				// Ahead and right of this car
				direction = 2;
			}
			else
			{
				// Straight ahead of this car
				direction = 1;
			}
		}
		else if(car->race.distFromStartLine > situation->cars[curCar]->race.distFromStartLine)
		{	
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				// Behind and left of this car
				direction = 6;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				// Behind and right of this car
				direction = 8;
			}
			else
			{
				// Directly behind this car
				direction = 7;
			}
		}
		else
		{
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				// Directly left of this car
				direction = 3;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				// Directly right of this car
				direction = 5;
			}
			else
			{
				// On top/underneath this car (impossible?) 
				direction = 4;
			}
		}
		
		

		// Distance to this car
		distance = (situation->cars[curCar]->race.distFromStartLine + (curTrack->length * situation->cars[curCar]->race.laps)) - (car->race.distFromStartLine + (curTrack->length * car->race.laps));

		// Get Steering
		steering = situation->cars[curCar]->_steerCmd;
	}
	else
	{	
		// Values given to the sensor variables if the car is too far away to matter
		// Speed 
		speed = 200;			

		// direction
		direction = 1;

		// Distance
		distance = 200;

		// Steering
		steering = 0.0f;
	}

	// Set variables to there positions in the sensors array
	sensors[0] = speed;
	sensors[1] = direction;
	sensors[2] = distance;
	sensors[3] = steering;

	// Get opCar 2 current speed, distance, direction and steering
	// Initialise variables 
	curCar = opCars[1];	// Index of the second car
	direction = -1;		// Direction of the second car
	speed = -1;		// Speed of the second car
	distance = 200;		// Distance of the seond car to this car
	steering = -1;		// The current steering value of the second car
	// if Distance to this is less than 200
	if(distance < 200)
	{
		// Get Speed
		speed = getSpeed(situation->cars[curCar]);
		
		// Direction
		if(car->race.distFromStartLine < situation->cars[curCar]->race.distFromStartLine)
		{
			
			//std::cout << "Ahead ";
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Left" << std::endl;
				direction = 0;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Right" << std::endl;
				direction = 2;
			}
			else
			{
				//std::cout << "Tie" << std::endl;
				direction = 1;
			}
		}
		else if(car->race.distFromStartLine > situation->cars[curCar]->race.distFromStartLine)
		{	
			//std::cout << "Behind ";
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Left" << std::endl;
				direction = 6;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Right" << std::endl;
				direction = 8;
			}
			else
			{
				//std::cout << "Tie" << std::endl;
				direction = 7;
			}
		}
		else
		{
			//std::cout << "Level ";
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Left" << std::endl;
				direction = 3;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Right" << std::endl;
				direction = 5;
			}
			else
			{
				//std::cout << "Tie" << std::endl;
				direction = 4;
			}
		}
		
		

		// Distance
		distance = (situation->cars[curCar]->race.distFromStartLine + (curTrack->length * situation->cars[curCar]->race.laps)) - (car->race.distFromStartLine + (curTrack->length * car->race.laps));

		// Get Steering
		steering = situation->cars[curCar]->_steerCmd;
	}
	else
	{
		// Speed 
		speed = 200;			

		// direction
		direction = 1;

		// Distance
		distance = 200;

		// Steering
		steering = 0.0f;
	}

	// Set variables to there positions in the sensors array
	sensors[4] = speed;
	sensors[5] = direction;
	sensors[6] = distance;
	sensors[7] = steering;
	
	// Get opCar 3 current speed, distance, direction and steering
	// Initialise variables 
	curCar = opCars[2];	// Index of the third car
	direction = -1;		// Direction of the third car
	speed = -1;		// Speed of the third car
	distance = 200;		// Distance of the third car to this car
	steering = -1;		// The current steering value of the third car
	// if Distance to this is less than 200
	if(distance < 200)
	{
		// Get Speed
		speed = getSpeed(situation->cars[curCar]);
		
		// Direction
		if(car->race.distFromStartLine < situation->cars[curCar]->race.distFromStartLine)
		{
			
			//std::cout << "Ahead ";
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Left" << std::endl;
				direction = 0;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Right" << std::endl;
				direction = 2;
			}
			else
			{
				//std::cout << "Tie" << std::endl;
				direction = 1;
			}
		}
		else if(car->race.distFromStartLine > situation->cars[curCar]->race.distFromStartLine)
		{	
			//std::cout << "Behind ";
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Left" << std::endl;
				direction = 6;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Right" << std::endl;
				direction = 8;
			}
			else
			{
				//std::cout << "Tie" << std::endl;
				direction = 7;
			}
		}
		else
		{
			//std::cout << "Level ";
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Left" << std::endl;
				direction = 3;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Right" << std::endl;
				direction = 5;
			}
			else
			{
				//std::cout << "Tie" << std::endl;
				direction = 4;
			}
		}
		
		

		// Distance
		distance = (situation->cars[curCar]->race.distFromStartLine + (curTrack->length * situation->cars[curCar]->race.laps)) - (car->race.distFromStartLine + (curTrack->length * car->race.laps));

		// Get Steering
		steering = situation->cars[curCar]->_steerCmd;
	}
	else
	{
		// Speed 
		speed = 200;			

		// direction
		direction = 1;

		// Distance
		distance = 200;

		// Steering
		steering = 0.0f;
	}

	// Set variables to there positions in the sensors array
	sensors[8] = speed;
	sensors[9] = direction;
	sensors[10] = distance;
	sensors[11] = steering;
	
	// Get opCar 4 current speed, distance, direction and steering
	// Initialise variables 
	curCar = opCars[3];	// Index of the fourth car
	direction = -1;		// Direction of the fourth car
	speed = -1;		// Speed of the fourth car
	distance = 200;		// Distance of the fourth car to this car
	steering = -1;		// The current steering value of the fourth car
	// if Distance to player is less than 200
	if(distance < 200)
	{
		// Get Speed
		speed = getSpeed(situation->cars[curCar]);
		
		// Direction
		if(car->race.distFromStartLine < situation->cars[curCar]->race.distFromStartLine)
		{
			
			//std::cout << "Ahead ";
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Left" << std::endl;
				direction = 0;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Right" << std::endl;
				direction = 2;
			}
			else
			{
				//std::cout << "Tie" << std::endl;
				direction = 1;
			}
		}
		else if(car->race.distFromStartLine > situation->cars[curCar]->race.distFromStartLine)
		{	
			//std::cout << "Behind ";
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Left" << std::endl;
				direction = 6;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Right" << std::endl;
				direction = 8;
			}
			else
			{
				//std::cout << "Tie" << std::endl;
				direction = 7;
			}
		}
		else
		{
			//std::cout << "Level ";
			if(car->_trkPos.toLeft > situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Left" << std::endl;
				direction = 3;
			}
			else if(car->_trkPos.toLeft < situation->cars[curCar]->_trkPos.toLeft)
			{
				//std::cout << "Right" << std::endl;
				direction = 5;
			}
			else
			{
				//std::cout << "Tie" << std::endl;
				direction = 4;
			}
		}
		
		

		// Distance
		distance = (situation->cars[curCar]->race.distFromStartLine + (curTrack->length * situation->cars[curCar]->race.laps)) - (car->race.distFromStartLine + (curTrack->length * car->race.laps));

		// Get Steering
		steering = situation->cars[curCar]->_steerCmd;
	}
	else
	{
		// Speed 
		speed = 200;			

		// direction
		direction = 1;

		// Distance
		distance = 200;

		// Steering
		steering = 0.0f;
	}

	// Set variables to there positions in the sensors array
	sensors[12] = speed;
	sensors[13] = direction;
	sensors[14] = distance;
	sensors[15] = steering;	
	
	// Player data
	// Angle to Track
	sensors[16] = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;

	// Dist to seg end
	sensors[17] = getDistToSegEnd(car);

	// Dist to seg left
	sensors[18] = car->_trkPos.toLeft;

	// Dist to seg right 
	sensors[19] = car->_trkPos.toRight;

	// _yaw
	sensors[20] = car->_yaw;

	// Target angle
	float targetAngle2;
	v2d target = getTargetPoint(car);	// Get target point position
	targetAngle2 = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
	targetAngle2 -= car->_yaw;	// Subtract car rotation
	sensors[21] = targetAngle2;

	// Speed
	sensors[22] = getSpeed(car);

	// Dist from start
	sensors[23] = curTrack->length - car->race.distFromStartLine;	

	// Track Segments ahead
	
	tTrackSeg  * seg22 = car->_trkPos.seg;	// Segment close
	tTrackSeg  * seg44 = car->_trkPos.seg;	// Segment moderate
	tTrackSeg  * seg66 = car->_trkPos.seg;	// Segment far 

	// Segment close
	// Assign current segment to be that of the one 22 places ahead of this car
	for(int i = 0; i < 22; i++)
	{	
		seg22 = seg22->next; // assign current segment to be the next 
	}
	
	sensors[24] = seg22->type;	// Close segment type
	sensors[25] = seg22->width;	// Close segment width
	sensors[26] = seg22->length;	// Close segment length
	sensors[27] = seg22->arc;	// Close segment arc

	
	// Segment moderate
	seg44 = seg22;	// assign close segment to moderate segment to make 
	// Assign current segment to be that of the one 44 places ahead of this car
	for(int i = 0; i < 22; i++)
	{
		seg44 = seg44->next; // assign current segment to be the next 
	}
	
	sensors[28] = seg44->type;	// Moderate segment type
	sensors[29] = seg44->width;	// Moderate segment width
	sensors[30] = seg44->length;	// Moderate segment length
	sensors[31] = seg44->arc;	// Moderate segment arc
	
	// Segment far
	seg66 = seg44; // Assign moderate segment to far segment to make
	// Assign current segment to be that of the one 66 places ahead of this car
	for(int i = 0; i < 22; i++)
	{
		seg66 = seg66->next;	// Assign current segment to be the next
	}
	
	sensors[32] =  seg66->type;	// Far segment type
	sensors[33] =  seg66->width;	// Far segment width
	sensors[34] =  seg66->length;	// Far segment length
	sensors[35] =  seg66->arc;	// Far segment arc
	
	std::vector<torch::jit::IValue> inputs; // Create a vector of inputs
	
	// Put inputs into tensor
	auto t = torch::tensor({sensors[0], sensors[1], sensors[2],sensors[3],
				sensors[4], sensors[5], sensors[6],sensors[7],
				sensors[8], sensors[9], sensors[10],sensors[11],
				sensors[12], sensors[13], sensors[14],sensors[15],
				sensors[16], sensors[17], sensors[18],sensors[19],
				sensors[20], sensors[21], sensors[22],sensors[23],
				sensors[24], sensors[25], sensors[26],sensors[27],
				sensors[28], sensors[29], sensors[30],sensors[31],
				sensors[32], sensors[33], sensors[34],sensors[35]});
	// New shape of tensor
	torch::IntArrayRef s = torch::IntArrayRef{18,36, 4, 4};
	t.resize_(s); 	// resize tensor
	inputs.push_back(t); // Put tensor into inputs vector
		
	// Pass the sensors through the model and outputs into a tensor
	auto output = module->forward(inputs).toTensor();

	float acl = *output[0].data<float>();	// Convert acceleration output to a float 
	float brk = *output[1].data<float>();	// Convert brake output to a float
	float str = *output[2].data<float>();	// Convert steering to a float

	// Applying caps to outputs
	// Acceleration
	if(acl < 0.0f)
	{
		acl = 0.0f;
	}
	else if(acl > 0.9f)
	{
		acl = 1.0f;
	}
	
	// Braking
	if(brk < 0.0f)
	{
		brk = 0.0f;
	}
	else if(brk > 0.9f)
	{
		brk = 1.0f;
	}	
	
	// Steering
	if(str > -0.1f && str < 0.1f)
	{
		str = 0.0f;
	}
	else if(str > 1.0f)
	{
		str = 1.0f;
	}
	else if(str < -1.0f)
	{
		str = -1.0f;
	}

	// Apply acceleration to car
	car->_accelCmd = acl;

	// Apply braking to car
	car->_brakeCmd = brk;

	// Apply steering to car
	car->_steerCmd = str;
	
	*/

}	


// Pitstop callback.
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
	MyCar* myc = mycar[index-1];
	Pathfinder* mpf = myc->getPathfinderPtr();

	float fullracedist = (myTrackDesc->getTorcsTrack()->length*s->_totLaps);
	float remaininglaps = (fullracedist - car->_distRaced)/myTrackDesc->getTorcsTrack()->length;

	car->_pitFuel = MAX(MIN(myc->fuelperlap*(remaininglaps+myc->FUEL_SAFETY_MARGIN) - car->_fuel, car->_tank - car->_fuel), 0.0);
	myc->lastpitfuel = MAX(car->_pitFuel, 0.0);
	car->_pitRepair = car->_dammage;
	mpf->setPitStop(false, myc->getCurrentSegId());
	myc->loadBehaviour(myc->START);
	myc->startmode = true;
	myc->trtime = 0.0;

	return ROB_PIT_IM; // Return immediately.
}


// Compute the clutch value.
// Does not work great when braking to 0 and accelerating again...
// TODO: Improve.
float getClutch(MyCar* myc, tCarElt* car)
{
	if (car->_gear > 1) {
		myc->clutchtime = 0.0;
		return 0.0;
	} else {
		float drpm = car->_enginerpm - car->_enginerpmRedLine/2.0;
		myc->clutchtime = MIN(myc->CLUTCH_FULL_MAX_TIME, myc->clutchtime);
		float clutcht = (myc->CLUTCH_FULL_MAX_TIME - myc->clutchtime)/myc->CLUTCH_FULL_MAX_TIME;
		if (car->_gear == 1 && car->_accelCmd > 0.0) {
			myc->clutchtime += (float) RCM_MAX_DT_ROBOTS;
		}

		//printf("ct: %f, car->_gear: %d, car->_gearCmd: %d, drpm: %f\n", myc->clutchtime, car->_gear, car->_gearCmd, drpm);
		if (drpm > 0) {
			float speedr;
			if (car->_gearCmd == 1) {
				// Compute corresponding speed to engine rpm.
				float omega = car->_enginerpmRedLine/car->_gearRatio[car->_gear + car->_gearOffset];
				float wr = car->_wheelRadius(2);
				speedr = (myc->CLUTCH_SPEED + MAX(0.0, car->_speed_x))/fabs(wr*omega);
				float clutchr = MAX(0.0, (1.0 - speedr*2.0*drpm/car->_enginerpmRedLine));
				return MIN(clutcht, clutchr);
			} else {
				// For the reverse gear.
				myc->clutchtime = 0.0;
				return 0.0;
			}
		} else {
			return clutcht;
		}
	}
}
