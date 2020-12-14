#include "creationBuggy.h"

void drawBuggy(Buggy buggy, dReal* sides, float radius){
    dsDrawBox(dBodyGetPosition(buggy.chassis[0]), dBodyGetRotation(buggy.chassis[0]), sides);
    dsSetColor(1, 1, 1);
    for (int i = 0; i <= 3; i++) dsDrawCylinder(dBodyGetPosition(buggy.roues[i]),
        dBodyGetRotation(buggy.roues[i]), 0.2f, radius);
}

void createABuggy(Buggy* buggy, dMass m, float cMass, float wMass,float length, float width, float heigh, float radius,int x, int y, int z,const dVector3 yunit,const dVector3 zunit, dSpaceID space, dWorldID world){
    int i;

    // chassis body
    buggy->chassis[0] = dBodyCreate(world);
    dBodySetPosition(buggy->chassis[0], x, y, z);
    dMassSetBox(&m, 1, length, width, heigh);
    dMassAdjust(&m, cMass);
    dBodySetMass(buggy->chassis[0], &m);
    buggy->box[0] = dCreateBox(0, length, width, heigh);
    dGeomSetBody(buggy->box[0], buggy->chassis[0]);

    // wheel bodies
    for (i = 0; i < 4; i++) {
        buggy->roues[i] = dBodyCreate(world);
        dQuaternion q;
        dQFromAxisAndAngle(q, 1, 0, 0, M_PI * 0.5);
        dBodySetQuaternion(buggy->roues[i], q);
        dMassSetSphere(&m, 1, radius);
        dMassAdjust(&m, wMass);
        dBodySetMass(buggy->roues[i], &m);
        buggy->sphere[i] = dCreateSphere(0, radius);
        dGeomSetBody(buggy->sphere[i], buggy->roues[i]);
    }
    dBodySetPosition(buggy->roues[0], 0.5 * length + x, width * 0.6 + y, z - heigh * 0.5);
    dBodySetPosition(buggy->roues[1], 0.5 * length + x, -width * 0.6 + y, z - heigh * 0.5);
    dBodySetPosition(buggy->roues[2], -0.5 * length + x, width * 0.6 + y, z - heigh * 0.5);
    dBodySetPosition(buggy->roues[3], -0.5 * length + x, -width * 0.6 + y, z - heigh * 0.5);

    // front and back wheel hinges 
    for (i = 0; i < 4; i++) {
        buggy->jointChassis_roues[i] = dJointCreateHinge2(world, 0);
        dJointAttach(buggy->jointChassis_roues[i], buggy->chassis[0], buggy->roues[i]);
        const dReal* a = dBodyGetPosition(buggy->roues[i]);
        dJointSetHinge2Anchor(buggy->jointChassis_roues[i], a[0], a[1], a[2]);
        dJointSetHinge2Axes(buggy->jointChassis_roues[i], zunit, yunit);
    }

    // set joint suspension 
    for (i = 0; i < 4; i++) {
        dJointSetHinge2Param(buggy->jointChassis_roues[i], dParamSuspensionERP, 0.4);
        dJointSetHinge2Param(buggy->jointChassis_roues[i], dParamSuspensionCFM, 0.8);
    }

    // lock back wheels along the steering axis
    for (i = 0; i < 4; i++) {
        // set stops to make sure wheels always stay in alignment
        dJointSetHinge2Param(buggy->jointChassis_roues[i], dParamLoStop, 0);
        dJointSetHinge2Param(buggy->jointChassis_roues[i], dParamHiStop, 0);
    }

    // create car space and add it to the top level space
    buggy->car_space = dSimpleSpaceCreate(space);
    dSpaceSetCleanup(buggy->car_space, 0);
    dSpaceAdd(buggy->car_space, buggy->box[0]);
    dSpaceAdd(buggy->car_space, buggy->sphere[0]);
    dSpaceAdd(buggy->car_space, buggy->sphere[1]);
    dSpaceAdd(buggy->car_space, buggy->sphere[2]);
    dSpaceAdd(buggy->car_space, buggy->sphere[3]);
    //dSpaceAdd(space, turrgeom[0]);
}

void destroyBuggy(Buggy buggy){
    dGeomDestroy(buggy.box[0]);
    dGeomDestroy(buggy.sphere[0]);
    dGeomDestroy(buggy.sphere[1]);
    dGeomDestroy(buggy.sphere[2]);
    dGeomDestroy(buggy.sphere[3]);
    //dGeomDestroy(buggy.turrgeom[0]);
}