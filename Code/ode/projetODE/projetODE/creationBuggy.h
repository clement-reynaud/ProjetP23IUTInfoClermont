#pragma once
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "math.h"
#include <cmath>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

typedef struct moveBuggy {
    dReal speedBuggy = 0;
    dReal steerBuggy = 0;
    bool lock_cam = false;
}MoveBuggy;

typedef struct buggy{
    dBodyID chassis[1];
    dBodyID roues[4];
    dJointID jointChassis_roues[4];
    dSpaceID car_space;
    dGeomID box[1];
    dGeomID sphere[4];

    MoveBuggy moveBuggy;

    //for the canon 
    dBodyID turrbody[1];
    dGeomID turrgeom[1];
    dJointID joint_turr[1];
}Buggy;

void drawBuggy(Buggy buggy, dReal* sides, float radius);
void createABuggy(Buggy* buggy, dMass m, float cMass, float wMass, float length, float width, float heigh, float radius, int x, int y, int z, const dVector3 yunit, const dVector3 zunit, dSpaceID space, dWorldID world);
void destroyBuggy(Buggy buggy);