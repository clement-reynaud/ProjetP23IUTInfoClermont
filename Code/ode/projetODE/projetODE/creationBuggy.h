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

typedef struct buggy {
    dBodyID chassis[1];
    dBodyID roues[4];
    dJointID jointChassis_roues[4];
    dSpaceID car_space;
    dGeomID box[1];
    dGeomID sphere[4];

    //for the canon 
    dBodyID turrbody[1];
    dGeomID turrgeom[1];
    dJointID joint_turr[1];
}Buggy;
void drawBuggy(dBodyID* chassis, dBodyID* roues, dBodyID* turrbody, dReal* sides, float radius);
void createABuggy(dBodyID* chassis, dBodyID* roues, dBodyID* turrbody, dJointID* jointChassis_roues, dJointID* joint_turr, dSpaceID car_space, dGeomID* box, dGeomID* sphere, dGeomID* turrgeom, dMass m, float cMass, float wMass, float length, float width, float heigh, float radius, int x, int y, int z,const dVector3 yunit, const dVector3 zunit, dSpaceID space, dWorldID world);
void destroyBuggy(dGeomID* box, dGeomID* sphere, dGeomID* turrgeom);