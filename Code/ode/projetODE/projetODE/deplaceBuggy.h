#pragma once
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "math.h"
#include <cmath>
#include "creationBuggy.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


void deplacementBuggy(Buggy* buggy);
void speedAndSteer(dJointID jointChassis_roues, MoveBuggy moveBuggy);
void retournerBuggy(Buggy buggy);
void camPos(Buggy buggy, static float* xyz, static float* hpr);
void arreterBuggy(Buggy buggy);
void tirer(Buggy buggy, float sphereRadius, dSpaceID space, dWorldID world);