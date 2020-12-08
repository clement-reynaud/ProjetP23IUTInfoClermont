#include "creationBuggy.h"

void drawBuggy(dBodyID* chassis, dBodyID* roues, dBodyID* turrbody, dReal* sides, float radius) {
    dsDrawBox(dBodyGetPosition(chassis[0]), dBodyGetRotation(chassis[0]), sides);
    dsSetColor(1, 1, 1);
    for (int i = 0; i <= 3; i++) dsDrawCylinder(dBodyGetPosition(roues[i]),
        dBodyGetRotation(roues[i]), 0.2f, radius);
}

void createABuggy(dBodyID* chassis, dBodyID* roues, dBodyID* turrbody, dJointID* jointChassis_roues, dJointID* joint_turr, dSpaceID car_space, dGeomID* box, dGeomID* sphere, dGeomID* turrgeom, dMass m, float cMass, float wMass,float length, float width, float heigh, float radius,int x, int y, int z,const dVector3 yunit,const dVector3 zunit, dSpaceID space, dWorldID world) {
    int i;
    // chassis body
    chassis[0] = dBodyCreate(world);
    dBodySetPosition(chassis[0], x, y, z);
    dMassSetBox(&m, 1, length, width, heigh);
    dMassAdjust(&m, cMass);
    dBodySetMass(chassis[0], &m);
    box[0] = dCreateBox(0, length, width, heigh);
    dGeomSetBody(box[0], chassis[0]);

    // wheel bodies
    for (i = 0; i <= 3; i++) {
        roues[i] = dBodyCreate(world);
        dQuaternion q;
        dQFromAxisAndAngle(q, 1, 0, 0, M_PI * 0.5);
        dBodySetQuaternion(roues[i], q);
        dMassSetSphere(&m, 1, radius);
        dMassAdjust(&m, wMass);
        dBodySetMass(roues[i], &m);
        sphere[i] = dCreateSphere(0, radius);
        dGeomSetBody(sphere[i], roues[i]);
    }
    dBodySetPosition(roues[0], 0.5 * length + x, width * 0.6 + y, z - heigh * 0.5);
    dBodySetPosition(roues[1], 0.5 * length + x, -width * 0.6 + y, z - heigh * 0.5);
    dBodySetPosition(roues[2], -0.5 * length + x, width * 0.6 + y, z - heigh * 0.5);
    dBodySetPosition(roues[3], -0.5 * length + x, -width * 0.6 + y, z - heigh * 0.5);

    // front and back wheel hinges 
    for (i = 0; i <= 3; i++) {
        jointChassis_roues[i] = dJointCreateHinge2(world, 0);
        dJointAttach(jointChassis_roues[i], chassis[0], roues[i]);
        const dReal* a = dBodyGetPosition(roues[i]);
        dJointSetHinge2Anchor(jointChassis_roues[i], a[0], a[1], a[2]);
        dJointSetHinge2Axes(jointChassis_roues[i], zunit, yunit);
    }

    // set joint suspension 
    for (i = 0; i < 4; i++) {
        dJointSetHinge2Param(jointChassis_roues[i], dParamSuspensionERP, 0.4);
        dJointSetHinge2Param(jointChassis_roues[i], dParamSuspensionCFM, 0.8);
    }

    // lock back wheels along the steering axis
    for (i = 0; i < 4; i++) {
        // set stops to make sure wheels always stay in alignment
        dJointSetHinge2Param(jointChassis_roues[i], dParamLoStop, 0);
        dJointSetHinge2Param(jointChassis_roues[i], dParamHiStop, 0);
    }

    // create car space and add it to the top level space
    car_space = dSimpleSpaceCreate(space);
    dSpaceSetCleanup(car_space, 0);
    dSpaceAdd(car_space, box[0]);
    dSpaceAdd(car_space, sphere[0]);
    dSpaceAdd(car_space, sphere[1]);
    dSpaceAdd(car_space, sphere[2]);
    dSpaceAdd(car_space, sphere[3]);
    //dSpaceAdd(space, turrgeom[0]);
}

void destroyBuggy(dGeomID* box, dGeomID* sphere, dGeomID* turrgeom) {
    dGeomDestroy(box[0]);
    dGeomDestroy(sphere[0]);
    dGeomDestroy(sphere[1]);
    dGeomDestroy(sphere[2]);
    dGeomDestroy(sphere[3]);
    //dGeomDestroy(turrgeom[0]);
}