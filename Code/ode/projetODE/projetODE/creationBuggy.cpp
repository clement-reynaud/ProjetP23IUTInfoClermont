#include "creationBuggy.h"

void drawBuggy(Buggy buggy,Turret turr, dReal* sides, float boxLenght, float boxWidth, float boxHeight, float cylinderLenght, float cylinderRadius, float sphereRadius, float radius, int* colors){
    dsSetColor(colors[0], colors[1], colors[2]);
    dsDrawBox(dBodyGetPosition(buggy.chassis[0]), dBodyGetRotation(buggy.chassis[0]), sides);
    dsSetColor(1, 1, 1);
    for (int i = 0; i <= 3; i++) dsDrawCylinder(dBodyGetPosition(buggy.roues[i]),
        dBodyGetRotation(buggy.roues[i]), 0.2f, radius);
    
    //Cylinder
    dsSetColor(colors[0], colors[1], colors[2]);
    //Turret
    const dReal* CPos = dBodyGetPosition(turr.canonBody);
    const dReal* CRot = dBodyGetRotation(turr.canonBody);
    const double cpos[3] = { CPos[0], CPos[1], CPos[2] };
    const double crot[12] = { CRot[0], CRot[1], CRot[2], CRot[3], CRot[4], CRot[5], CRot[6], CRot[7], CRot[8], CRot[9], CRot[10], CRot[11] };
    dsDrawCylinder
    (
        cpos,
        crot,
        cylinderLenght,
        cylinderRadius
    ); // single precision

    //Box
    dsSetColor(1, 1, 1);
    dReal boxsides[3] = { boxLenght,boxWidth,boxHeight };
    dsDrawBox(dBodyGetPosition(turr.boxBody), dBodyGetRotation(turr.boxBody), boxsides);

    //Bullet
    dsSetColor(colors[0], colors[1], colors[2]);
    for (int i = 0; i < buggy.bulletMax.num; i++) {
        const dReal* SPos = dBodyGetPosition(buggy.bulletMax.obj[i].body);
        const dReal* SRot = dBodyGetRotation(buggy.bulletMax.obj[i].body);
        const double spos[3] = { SPos[0], SPos[1], SPos[2] };
        const double srot[12] = { SRot[0], SRot[1], SRot[2], SRot[3], SRot[4], SRot[5], SRot[6], SRot[7], SRot[8], SRot[9], SRot[10], SRot[11] };
        dsDrawSphere(spos,srot,sphereRadius);
    }
}

void createABuggy(Buggy* buggy, Turret* turret, dMass m,float bMass, float cMass, float wMass, float length, float width, float heigh, float boxLenght,float boxWidth, float boxHeight, float cylinderLenght, float cylinderRadius, float radius,int x, int y, int z,const dVector3 yunit,const dVector3 zunit, dSpaceID space, dWorldID world){
    int i;

    // chassis body
    buggy->chassis[0] = dBodyCreate(world);
    dBodySetPosition(buggy->chassis[0], x, y, z);
    dMassSetBox(&m, 1, length, width, heigh);
    dMassAdjust(&m, cMass);
    dBodySetMass(buggy->chassis[0], &m);
    buggy->box[0] = dCreateBox(0, length, width, heigh);
    dGeomSetBody(buggy->box[0], buggy->chassis[0]);

    // wheel 1 & 2 bodies
    for (i = 0; i < 2; i++) {
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
    //wheel 3 & 4 bodies
    for (i = 2; i < 4; i++) {
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

    //TurretBox
    turret->boxBody = dBodyCreate(world);
    dBodySetPosition(turret->boxBody, x, y, z + 1);
    dMassSetBox(&m, 1, boxLenght, boxWidth, boxHeight);
    dMassAdjust(&m, bMass);
    dBodySetMass(turret->boxBody, &m);
    turret->boxGeom = dCreateBox(0, boxLenght, boxWidth, boxHeight);
    dGeomSetBody(turret->boxGeom, turret->boxBody);
    dBodySetPosition(turret->boxBody, x, y, z + heigh);
    dSpaceAdd(space, turret->boxGeom);

    //Joint TurretBox and chassis
    turret->jointChassis_Box = dJointCreateFixed(world, 0);
    dJointAttach(turret->jointChassis_Box, buggy->chassis[0], turret->boxBody);
    dJointSetFixed(turret->jointChassis_Box);

    //Canon
    turret->canonBody = dBodyCreate(world);
    dQuaternion q;
    dMassSetCylinder(&m, 1.0, 3, cylinderRadius, cylinderLenght);
    dBodySetMass(turret->canonBody, &m);
    dMassAdjust(&m, bMass);
    turret->canonGeom = dCreateCylinder(0, cylinderRadius, cylinderLenght);
    dGeomSetBody(turret->canonGeom, turret->canonBody);
    dBodySetPosition(turret->canonBody, x, y, z + heigh + boxHeight - 0.1);
    dSpaceAdd(space, turret->canonGeom);
    dMatrix3 Rbox;
    dRFromAxisAndAngle(Rbox, 0, 90, 0, 80);
    dGeomSetRotation(turret->canonGeom, Rbox);

    // joint TurretBox Canon
    turret->jointBox_Canon = dJointCreateFixed(world, 0);
    dJointAttach(turret->jointBox_Canon, turret->canonBody, turret->boxBody);
    dJointSetFixed(turret->jointBox_Canon);

    // create car space and add it to the top level space
    buggy->car_space = dSimpleSpaceCreate(space);
    dSpaceSetCleanup(buggy->car_space, 0);
    dSpaceAdd(buggy->car_space, buggy->box[0]);
    dSpaceAdd(buggy->car_space, buggy->sphere[0]);
    dSpaceAdd(buggy->car_space, buggy->sphere[1]);
    dSpaceAdd(buggy->car_space, buggy->sphere[2]);
    dSpaceAdd(buggy->car_space, buggy->sphere[3]);
}

void destroyBuggy(Buggy buggy,Turret turr){
    dGeomDestroy(buggy.box[0]);
    dGeomDestroy(buggy.sphere[0]);
    dGeomDestroy(buggy.sphere[1]);
    dGeomDestroy(buggy.sphere[2]);
    dGeomDestroy(buggy.sphere[3]);
    dGeomDestroy(turr.canonGeom);
    dGeomDestroy(turr.boxGeom);
}