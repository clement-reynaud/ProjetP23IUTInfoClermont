#include "creationBuggy.h"

void drawBuggy(Buggy buggy, dReal* sides, float boxLenght, float boxWidth, float boxHeight, float cylinderLenght, float cylinderRadius, float sphereRadius, float radius){
    dsSetColor(buggy.colors[0], buggy.colors[1], buggy.colors[2]);
    dsDrawBox(dBodyGetPosition(buggy.chassis[0]), dBodyGetRotation(buggy.chassis[0]), sides);
    dsSetColor(1, 1, 1);
    for (int i = 0; i <= 3; i++) dsDrawCylinder(dBodyGetPosition(buggy.roues[i]),
        dBodyGetRotation(buggy.roues[i]), 0.2f, radius);
    
    //Cylinder
    dsSetColor(buggy.colors[0], buggy.colors[1], buggy.colors[2]);
    //Turret
    const dReal* CPos = dBodyGetPosition(buggy.turret.canonBody);
    const dReal* CRot = dBodyGetRotation(buggy.turret.canonBody);
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
    dsDrawBox(dBodyGetPosition(buggy.turret.boxBody), dBodyGetRotation(buggy.turret.boxBody), boxsides);

    //Bullet
    dsSetColor(buggy.colors[0], buggy.colors[1], buggy.colors[2]);
    for (int i = 0; i < buggy.bulletMax.num; i++) {
        const dReal* SPos = dBodyGetPosition(buggy.bulletMax.obj[i].body);
        const dReal* SRot = dBodyGetRotation(buggy.bulletMax.obj[i].body);
        const double spos[3] = { SPos[0], SPos[1], SPos[2] };
        const double srot[12] = { SRot[0], SRot[1], SRot[2], SRot[3], SRot[4], SRot[5], SRot[6], SRot[7], SRot[8], SRot[9], SRot[10], SRot[11] };
        dsDrawSphere(spos,srot,sphereRadius);
    }
}

//L'ordre des positions dans le tableau est important.
const dReal** sendPosToDraw(Buggy buggy) {
    const dReal* positions[24];
    positions[0] = dBodyGetPosition(buggy.chassis[0]);
    positions[1] = dBodyGetRotation(buggy.chassis[0]);

    positions[2] = dBodyGetPosition(buggy.roues[0]);
    positions[3] = dBodyGetRotation(buggy.roues[0]);

    positions[4] = dBodyGetPosition(buggy.roues[1]);
    positions[5] = dBodyGetRotation(buggy.roues[1]);

    positions[6] = dBodyGetPosition(buggy.roues[2]);
    positions[7] = dBodyGetRotation(buggy.roues[2]);

    positions[8] = dBodyGetPosition(buggy.roues[3]);
    positions[9] = dBodyGetRotation(buggy.roues[3]);

    positions[10] = dBodyGetPosition(buggy.turret.canonBody);
    positions[11] = dBodyGetRotation(buggy.turret.canonBody);

    positions[12] = dBodyGetPosition(buggy.turret.boxBody);
    positions[13] = dBodyGetRotation(buggy.turret.boxBody);

    int i = 14;
    for (int cpt = 0; cpt < buggy.bulletMax.num; cpt++) {
        positions[i] = dBodyGetPosition(buggy.bulletMax.obj[cpt].body);
            i++;
        positions[i] = dBodyGetRotation(buggy.bulletMax.obj[cpt].body);
            i++;
    }
    return positions;
}

void createABuggy(Buggy* buggy, dMass m,float bMass, float cMass, float wMass, float length, float width, float heigh, float boxLenght,float boxWidth, float boxHeight, float cylinderLenght, float cylinderRadius, float radius,int x, int y, int z,const dVector3 yunit,const dVector3 zunit, dSpaceID space, dWorldID world){
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
    buggy->turret.boxBody = dBodyCreate(world);
    dBodySetPosition(buggy->turret.boxBody, x, y, z + 1);
    dMassSetBox(&m, 1, boxLenght, boxWidth, boxHeight);
    dMassAdjust(&m, bMass);
    dBodySetMass(buggy->turret.boxBody, &m);
    buggy->turret.boxGeom = dCreateBox(0, boxLenght, boxWidth, boxHeight);
    dGeomSetBody(buggy->turret.boxGeom, buggy->turret.boxBody);
    dBodySetPosition(buggy->turret.boxBody, x, y, z + heigh);
    dSpaceAdd(space, buggy->turret.boxGeom);

    //Joint TurretBox and chassis
    buggy->turret.jointChassis_Box = dJointCreateFixed(world, 0);
    dJointAttach(buggy->turret.jointChassis_Box, buggy->chassis[0], buggy->turret.boxBody);
    dJointSetFixed(buggy->turret.jointChassis_Box);

    //Canon
    buggy->turret.canonBody = dBodyCreate(world);
    dQuaternion q;
    dMassSetCylinder(&m, 1.0, 3, cylinderRadius, cylinderLenght);
    dBodySetMass(buggy->turret.canonBody, &m);
    dMassAdjust(&m, bMass);
    buggy->turret.canonGeom = dCreateCylinder(0, cylinderRadius, cylinderLenght);
    dGeomSetBody(buggy->turret.canonGeom, buggy->turret.canonBody);
    dBodySetPosition(buggy->turret.canonBody, x, y, z + heigh + boxHeight - 0.1);
    dSpaceAdd(space, buggy->turret.canonGeom);
    dMatrix3 Rbox;
    dRFromAxisAndAngle(Rbox, 0, 90, 0, 80);
    dGeomSetRotation(buggy->turret.canonGeom, Rbox);

    // joint TurretBox Canon
    buggy->turret.jointBox_Canon = dJointCreateFixed(world, 0);
    dJointAttach(buggy->turret.jointBox_Canon, buggy->turret.canonBody, buggy->turret.boxBody);
    dJointSetFixed(buggy->turret.jointBox_Canon);

    // create car space and add it to the top level space
    buggy->car_space = dSimpleSpaceCreate(space);
    dSpaceSetCleanup(buggy->car_space, 0);
    dSpaceAdd(buggy->car_space, buggy->box[0]);
    dSpaceAdd(buggy->car_space, buggy->sphere[0]);
    dSpaceAdd(buggy->car_space, buggy->sphere[1]);
    dSpaceAdd(buggy->car_space, buggy->sphere[2]);
    dSpaceAdd(buggy->car_space, buggy->sphere[3]);
}

void destroyBuggy(Buggy buggy){
    dGeomDestroy(buggy.box[0]);
    dGeomDestroy(buggy.sphere[0]);
    dGeomDestroy(buggy.sphere[1]);
    dGeomDestroy(buggy.sphere[2]);
    dGeomDestroy(buggy.sphere[3]);
    dGeomDestroy(buggy.turret.canonGeom);
    dGeomDestroy(buggy.turret.boxGeom);
}