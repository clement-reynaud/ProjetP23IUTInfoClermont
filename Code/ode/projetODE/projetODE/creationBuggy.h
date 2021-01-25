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


typedef struct bullet {
    dBodyID body;            // the body
    dGeomID geom;        // geometries representing this body
}Bullet;

typedef struct bulletMax{
    int num ;
    int nextobj;
    Bullet obj[5]; //Max number of sphere per buggy
}BulletMax; 

typedef struct turret {
    dGeomID canonGeom;
    dBodyID canonBody;
    dJointID jointBox_Canon;
    dGeomID boxGeom;
    dBodyID boxBody;
    dJointID jointChassis_Box;
}Turret;

typedef struct buggy{
    dBodyID chassis[1];
    dBodyID roues[4];
    dJointID jointChassis_roues[4];
    dSpaceID car_space;
    dGeomID box[1];
    dGeomID sphere[4];
    int colors[3];

    MoveBuggy moveBuggy;

    BulletMax bulletMax;
    Turret turret;
}Buggy;



void drawBuggy(Buggy buggy, dReal* sides, float boxLenght, float boxWidth, float boxHeight, float cylinderLenght, float cylinderRadius, float sphereRadius, float radius);
const dReal** sendPosToDraw(Buggy buggy);
void createABuggy(Buggy* buggy, dMass m, float bMass, float cMass, float wMass, float length, float width, float heigh, float boxLenght, float boxWidth, float boxHeight, float cylinderLenght, float cylinderRadius, float radius, int x, int y, int z, const dVector3 yunit, const dVector3 zunit, dSpaceID space, dWorldID world);
void destroyBuggy(Buggy buggy);
