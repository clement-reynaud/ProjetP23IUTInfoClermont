/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

 /*

 buggy with suspension.
 this also shows you how to use geom groups.

 */


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "math.h"
#include <cmath>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

 // select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


// some constants

#define LENGTH 2	// chassis length
#define WIDTH 1	// chassis width
#define HEIGHT 0.7	// chassis height
#define RADIUS 0.6	// wheel radius
#define CMASS 1		// chassis mass
#define WMASS 0.2	// wheel mass
#define STARTX1 -5   //Start x of chassis 1
#define STARTY1 0   //Start y of chassis 1
#define STARTZ 0.5	// starting height of chassis
#define STARTX2 -10   //Start x of chassis 2
#define STARTY2 0   //Start y of chassis 2

//turret
#define TURRRADIUS    0.07
#define TURRLENGTH    0.3

//Cannon deffinition
#define CANNON_BALL_MASS 0.1	// mass of the cannon ball
#define CANNON_BALL_RADIUS 0.05 // size of the cannon ball


static const dVector3 yunit = { 0, 1, 0 }, zunit = { 0, 0, 1 };


// dynamics and collision objects (chassis, 4 wheels, environment)

static dWorldID world;
static dSpaceID space;
static dBodyID chassis1[1];
//static dJointID jointChassis[1];	// joint[0] is the front wheel
static dBodyID roues1[4];
static dBodyID chassis2[1];
static dBodyID roues2[4];
static dJointID jointChassis_roues1[4]; // jointChassis_roues[0] & jointChassis_roues[1] are the front wheels
static dJointID jointChassis_roues2[4];
static dJointID jointChassis_cannon[2];
static dJointID jointCannon[2]; // The cannon is composed of a box and a cylinder
static dGeomID cannon_ball_geom;
static dBodyID cannon_ball_body;
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space1;
static dSpaceID car_space2;
static dGeomID box1[1]; //Box of the chassis 1
static dGeomID box2[1]; //Box of the chassis 2
static dGeomID sphere1[4]; //Sphere of wheels 1
static dGeomID sphere2[4]; //Sphere of wheels 2

//for the canon 
static dBodyID turrbody1[1];
static dGeomID turrgeom1[1];
static dJointID joint_turr1[1];
static dBodyID turrbody2[1];
static dGeomID turrgeom2[1];
static dJointID joint_turr2[1];
static dGeomID ground_box;

/*
//turret
    const dReal* CPos = dBodyGetPosition(turrbody[0]);
    const dReal* CRot = dBodyGetRotation(turrbody[0]);
    const double cpos[3] = { CPos[0], CPos[1], CPos[2] };
    const double crot[12] = { CRot[0], CRot[1], CRot[2], CRot[3], CRot[4], CRot[5], CRot[6], CRot[7], CRot[8], CRot[9], CRot[10], CRot[11] };
    dsDrawCylinder
    (
        cpos,
        crot,
        TURRLENGTH,
        TURRRADIUS
    ); // single precision

//TURRET
    turrbody[0] = dBodyCreate(world);
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 0, 0, M_PI * -0.77);
    dBodySetQuaternion(turrbody[0], q);
    dMassSetCylinder(&m, 1.0, 3, TURRRADIUS, TURRLENGTH);
    dBodySetMass(turrbody[0], &m);
    turrgeom = dCreateCylinder(0, TURRRADIUS, TURRLENGTH);
    dGeomSetBody(turrgeom, turrbody[0]);
    dBodySetPosition(turrbody[0], 0,0, STARTZ + HEIGHT );
    dSpaceAdd(space, turrgeom);

// joint turret
    joint_turr = dJointCreateHinge2(world, 0);
    dJointAttach(joint_turr, chassis[0], turrbody[0]);
    const dReal* a = dBodyGetPosition(turrbody[0]);
    dJointSetHinge2Anchor(joint_turr, a[0], a[1], a[2]);
    dJointSetHinge2Axes(joint_turr, zunit, yunit);
    */


// things that the user controls

static dReal speed = 0, steer = 0;	// user commands
dReal speedBuggy1 = 0;
dReal speedBuggy2 = 0;
dReal steerBuggy1 = 0;
dReal steerBuggy2 = 0;

static bool lock_cam1 = true;
static bool lock_cam2 = false;
static dReal cannon_angle = 0, cannon_elevation = -1.2;

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback(void*, dGeomID o1, dGeomID o2)
{
    int i, n;

    const int N = 10;
    dContact contact[N];
    n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
    if (n > 0) {
        for (i = 0; i < n; i++) {
            contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                dContactSoftERP | dContactSoftCFM | dContactApprox1;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.slip1 = 0.1;
            contact[i].surface.slip2 = 0.1;
            contact[i].surface.soft_erp = 0.5;
            contact[i].surface.soft_cfm = 0.3;
            dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
            dJointAttach(c,
                dGeomGetBody(contact[i].geom.g1),
                dGeomGetBody(contact[i].geom.g2));
        }
    }
}
/*
static void nearCallback(void*, dGeomID o1, dGeomID o2) {
    int i, n;

    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnected(b1, b2))
        return;

    const int N = 50;
    dContact contact[N];
    n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
    if (n > 0) {
        for (i = 0; i < n; i++) {
            contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
            if (dGeomGetClass(o1) == dSphereClass || dGeomGetClass(o2) == dSphereClass)
                contact[i].surface.mu = 20;
            else
                contact[i].surface.mu = 0.5;
            contact[i].surface.slip1 = 0.0;
            contact[i].surface.slip2 = 0.0;
            contact[i].surface.soft_erp = 0.8;
            contact[i].surface.soft_cfm = 0.01;
            dJointID c = dJointCreateContact(world, contactgroup, contact + i);
            dJointAttach(c, dGeomGetBody(o1), dGeomGetBody(o2));
        }
    }
}
*/

// start simulation - set viewpoint

static float xyz[3] = { 0,-90,0 };
static float hpr[3] = { 0.8317f,-0.9817f,0.8000f };

static void start()
{
    dAllocateODEDataForThread(dAllocateMaskAll);

    dsSetViewpoint(xyz, hpr);
    printf("Press:\t'a' to increase speed.\n"
        "\t's' to decrease speed.\n"
        "\t'q' to steer left.\n"
        "\t'd' to steer right.\n"
        "\t'l' to lock the camera or to unlock it.\n"
        "\t' ' to reset speed and steering.\n"
        "\tUse caps to control the second buggy.\n"
        "\t'5' to turn the buggy over.\n"
        "\t'6' to turn the second buggy over.\n"
        "\t'1' to save the current state to 'state.dif'.\n"
        /*"\t'2' to lower the cannon.\n"
        "\t'3' to raise the cannon.\n"
        "\t'4' to fire with the cannon.\n"
        */
    );
}

float calCos(float xx, float xy, float yx, float yy, const dReal* rota) {
    float relx, relxx;//float de distance relative en x
    float rely, relyy;//float de distance relative en y
    float alphaCos;// angle à la sortie du cosinus
    float hyp;//float de l'hypothnuse
    float teta;// angle teta du calcul du adjacent/hypoténuse


    relx = xx - yx; //calcul de la distance x entre le centre de la voiture et la caméra
    rely = xy - yy; //calcul de la distance y entre le centre de la voiture et la caméra
    relxx = relx * relx; // calcul de la distance relative x au carré
    relyy = rely * rely; // calcul de la distance relative y au carré
    hyp = relxx + relyy; //calcul de l'hypoténuse au carré
    hyp = sqrt(hyp);// calcul de l'hupoténuse
    teta = (relx / hyp);//calcul de l'angle adj/hyp
    alphaCos = acos(teta) * 180 / M_PI;//calcul de l'angle alpha demandé

    if (rota[1] < 0) {
        return alphaCos; // renvoie de la valeur demandée
    }
    return -1 * alphaCos; // renvoie de la valeur demandée
}

void camPos(dBodyID obj_body) // print a position
{

    const dReal* pos;
    float fx, fy, fz;
    pos = dBodyGetPosition(obj_body); // on prend les coordonnées du chassis
    fx = (float)pos[0]; // x
    fy = (float)pos[1]; // y
    fz = (float)pos[2]; // z

    //printf("x=%f y=%f z=%f \n", x, y, z);
    const dReal* rota = dBodyGetRotation(obj_body); // on prend l'orientation du chassis
    xyz[0] = fx + (rota[0] * (-2)); // on ajoute l
    xyz[1] = fy + (rota[1] * (2));
    xyz[2] = fz + 1;


    float result_cos = calCos(fx, fy, xyz[0], xyz[1], rota);//float result_cos = calCos(fx, fy, xyz[0], xyz[1], rota);
    hpr[0] = result_cos;

    //printf("%f\n", result_cos);
    dsSetViewpoint(xyz, hpr);
    //printf("rota0: %f\trota1: %f\n", rota[0], rota[1]);

}

void retourner(dBodyID obj_body) {
    steer = 0;
    speed = 0;
    const dReal* pos;
    pos = dBodyGetPosition(obj_body);
    dBodySetPosition(obj_body, pos[0], pos[1], pos[2] + 1);

    //const dReal* rota = dBodyGetRotation(obj_body);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 0, 0, 0);
    dBodySetRotation(obj_body, R);
}

// called when a key pressed

static void command(int cmd)
{
    switch (cmd) {
    case 'z':
        if (!(speedBuggy1 >= 3.3)) {
            speedBuggy1 += 0.3;
        }
        break;
    case 'Z':
        if (!(speedBuggy2 >= 3.3)) {
            speedBuggy2 += 0.3;
        }
        break;
    case 's':
        if (!(speedBuggy1 <= -3.3)) {
            speedBuggy1 -= 0.3;
        }
        break;
    case 'S':
        if (!(speedBuggy2 <= -3.3)) {
            speedBuggy2 -= 0.3;
        }
        break;
    case 'q':
        steerBuggy1 -= 0.3;
        break;
    case 'Q' :
        steerBuggy2 -= 0.3;
        break;
    case 'd':
        steerBuggy1 += 0.3;
        break;
    case 'D' :
        steerBuggy2 += 0.3;
        break;
    case 'l':
        if (lock_cam2 == true) {
            lock_cam2 = false;
        }
        lock_cam1 = !lock_cam1;
        break;
    case 'L':
        if (lock_cam1 == true) {
            lock_cam1 = false;
        }
        lock_cam2 = !lock_cam2;
        break;
    case '5':
        retourner(chassis1[0]);
        break;
    case '6':
        retourner(chassis2[0]);
    case ' ':
        speedBuggy1 = 0;
        steerBuggy1 = 0;
        speedBuggy2 = 0;
        steerBuggy2 = 0;

        break;
    case '1': {
        FILE* f = fopen("state.dif", "wt");
        if (f) {
            dWorldExportDIF(world, f, "");
            fclose(f);
        }
    }
    }
}

void speedAndSteer(dJointID jointChassis_roues,dReal speedBuggy, dReal steerBuggy) {
    dJointSetHinge2Param(jointChassis_roues, dParamVel2, -speedBuggy);
    dJointSetHinge2Param(jointChassis_roues, dParamFMax2, 0.1);
    dReal s = steerBuggy - dJointGetHinge2Angle1(jointChassis_roues);
    if (s > 0.1) s = 0.1;
    if (s < -0.1) s = -0.1;
    s *= 10.0;
    dJointSetHinge2Param(jointChassis_roues, dParamVel, s);
    dJointSetHinge2Param(jointChassis_roues, dParamFMax, 0.2);
    dJointSetHinge2Param(jointChassis_roues, dParamLoStop, -0.75);
    dJointSetHinge2Param(jointChassis_roues, dParamHiStop, 0.75);
    dJointSetHinge2Param(jointChassis_roues, dParamFudgeFactor, 0.1);
}

void drawBuggy(dBodyID* chassis, dBodyID* roues, dBodyID* turrbody, dReal* sides) {
    dsDrawBox(dBodyGetPosition(chassis[0]), dBodyGetRotation(chassis[0]), sides);
    dsSetColor(1, 1, 1);
    for (int i = 0; i <= 3; i++) dsDrawCylinder(dBodyGetPosition(roues[i]),
        dBodyGetRotation(roues[i]), 0.2f, RADIUS);
    
    //turret
    const dReal* CPos = dBodyGetPosition(turrbody[0]);
    const dReal* CRot = dBodyGetRotation(turrbody[0]);
    const double cpos[3] = { CPos[0], CPos[1], CPos[2] };
    const double crot[12] = { CRot[0], CRot[1], CRot[2], CRot[3], CRot[4], CRot[5], CRot[6], CRot[7], CRot[8], CRot[9], CRot[10], CRot[11] };
    dsDrawCylinder(cpos, crot, TURRLENGTH, TURRRADIUS);
}

// simulation loop
static void simLoop(int pause)
{
    int i;
    if (!pause) {
        speedAndSteer(jointChassis_roues1[0], speedBuggy1, steerBuggy1);
        speedAndSteer(jointChassis_roues1[1], speedBuggy1, steerBuggy1);
        speedAndSteer(jointChassis_roues2[0], speedBuggy2, steerBuggy2);
        speedAndSteer(jointChassis_roues2[1], speedBuggy2, steerBuggy2);

        //view 
        //Mettre a jour la vue (pour qu'elle suive le robot)

        /* tests
        dBodyGetPosition(body[0]);
        static float xyz[3] = { 0.8317f,-0.9817f,0.8000f };
        static float hpr[3] = { 121.0000f,-27.5000f,0.0000f };
        */

        dSpaceCollide(space, 0, &nearCallback);
        dWorldStep(world, 0.05);

        // remove all contact joints
        dJointGroupEmpty(contactgroup);

        // fixed or not-fixed camera
        if (lock_cam1) {
            camPos(chassis1[0]);
        }
        if (lock_cam2) {
            camPos(chassis2[0]);
        }
    }

    dsSetColor(0, 1, 1);
    dsSetTexture(DS_WOOD);
    dReal sides[3] = { LENGTH,WIDTH,HEIGHT };

    //Draw buggy 1
    drawBuggy(chassis1, roues1, turrbody1, sides);
    //Draw buggy 2
    drawBuggy(chassis2, roues2, turrbody2, sides);

    // draw the cannon
    /*dMatrix3 R2, R3, R4;
    dRFromAxisAndAngle(R2, 0, 0, 1, cannon_angle);
    dRFromAxisAndAngle(R3, 0, 1, 0, cannon_elevation);
    dMultiply0(R4, R2, R3, 3, 3, 3);
    dReal cpos[3] = { 1,1,1 };
    for (i = 0; i < 3; i++) cpos[i] -=2  * R4[i * 1];
    dsDrawCylinder(cpos, R4, 0.33, 0.5);

    // draw the cannon ball
    dsDrawSphere(dBodyGetPosition(cannon_ball_body), dBodyGetRotation(cannon_ball_body),
        CANNON_BALL_RADIUS);
    */
    dVector3 ss;
    dGeomBoxGetLengths(ground_box, ss);
    dsDrawBox(dGeomGetPosition(ground_box), dGeomGetRotation(ground_box), ss);

}

void createABuggy(dBodyID* chassis, dBodyID* roues,dBodyID* turrbody, dJointID* jointChassis_roues, dJointID* joint_turr, dSpaceID car_space, dGeomID* box, dGeomID* sphere, dGeomID* turrgeom, dMass m, int x, int y, int z) {
    int i;
    // chassis body
    chassis[0] = dBodyCreate(world);
    dBodySetPosition(chassis[0], x, y, z);
    dMassSetBox(&m, 1, LENGTH, WIDTH, HEIGHT);
    dMassAdjust(&m, CMASS);
    dBodySetMass(chassis[0], &m);
    box[0] = dCreateBox(0, LENGTH, WIDTH, HEIGHT);
    dGeomSetBody(box[0], chassis[0]);

    // wheel bodies
    for (i = 0; i <= 3; i++) {
        roues[i] = dBodyCreate(world);
        dQuaternion q;
        dQFromAxisAndAngle(q, 1, 0, 0, M_PI * 0.5);
        dBodySetQuaternion(roues[i], q);
        dMassSetSphere(&m, 1, RADIUS);
        dMassAdjust(&m, WMASS);
        dBodySetMass(roues[i], &m);
        sphere[i] = dCreateSphere(0, RADIUS);
        dGeomSetBody(sphere[i], roues[i]);
    }
    dBodySetPosition(roues[0], 0.5 * LENGTH + x, WIDTH * 0.6 + y, z);
    dBodySetPosition(roues[1], 0.5 * LENGTH + x, -WIDTH * 0.6 + y, z);
    dBodySetPosition(roues[2], -0.5 * LENGTH + x, WIDTH * 0.6 + y, z);
    dBodySetPosition(roues[3], -0.5 * LENGTH + x, -WIDTH * 0.6 + y, z);

    //turet body
    turrbody[0] = dBodyCreate(world);
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 0, 0, M_PI * -0.77);
    dBodySetQuaternion(turrbody[0], q);
    dMassSetCylinder(&m, 1.0, 3, TURRRADIUS, TURRLENGTH);
    dBodySetMass(turrbody[0], &m);
    turrgeom[0] = dCreateCylinder(0, TURRRADIUS, TURRLENGTH);
    dGeomSetBody(turrgeom[0], turrbody[0]);
    dBodySetPosition(turrbody[0], x, y, z + HEIGHT);

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
        // the following alternative method is no good as the wheels may get out
        // of alignment:
        //   dJointSetHinge2Param (joint[i],dParamVel,0);
        //   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
    }

    // joint turret
    joint_turr[0] = dJointCreateHinge2(world, 0);
    dJointAttach(joint_turr[0], chassis[0], turrbody[0]);
    const dReal* a = dBodyGetPosition(turrbody[0]);
    dJointSetHinge2Anchor(joint_turr[0], a[0], a[1], a[2]);
    dJointSetHinge2Axes(joint_turr[0], zunit, yunit);
    dJointSetHinge2Param(joint_turr[0], dParamLoStop, 0);
    dJointSetHinge2Param(joint_turr[0], dParamHiStop, 0);

    // create car space and add it to the top level space
    car_space = dSimpleSpaceCreate(space);
    dSpaceSetCleanup(car_space, 0);
    dSpaceAdd(car_space, box[0]);
    dSpaceAdd(car_space, sphere[0]);
    dSpaceAdd(car_space, sphere[1]);
    dSpaceAdd(car_space, sphere[2]);
    dSpaceAdd(car_space, sphere[3]);
    dSpaceAdd(space, turrgeom[0]);
}

void destroyBuggy(dGeomID* box, dGeomID* sphere, dGeomID* turrgeom) {
    dGeomDestroy(box[0]);
    dGeomDestroy(sphere[0]);
    dGeomDestroy(sphere[1]);
    dGeomDestroy(sphere[2]);
    dGeomDestroy(sphere[3]);
    dGeomDestroy(turrgeom[0]);
}

int main(int argc, char** argv)
{
    int i;
    dMass m;

    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

    // create world
    dInitODE2(0);
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, 0, -0.98);
    ground = dCreatePlane(space, 0, 0, 1, 0);
    //void createABuggy(dBodyID* chassis, dBodyID* roues,dBodyID* turrbody, dJointID* jointChassis_roues, dJointID* joint_turr, dSpaceID car_space, dGeomID* box, dGeomID* sphere, dGeomID* turrgeom, dMass m, int x, int y, int z) {

    createABuggy(chassis1,roues1,turrbody1,jointChassis_roues1, joint_turr1,car_space1,box1,sphere1, turrgeom1,m,-5,0,STARTZ);
    createABuggy(chassis2, roues2, turrbody2, jointChassis_roues2, joint_turr2, car_space2, box2, sphere2, turrgeom2, m, -8, 0, STARTZ);

    // cannon
    /*
    cannon_ball_body = dBodyCreate(world);
    cannon_ball_geom = dCreateSphere(space, CANNON_BALL_RADIUS);
    dMassSetSphereTotal(&m, CANNON_BALL_MASS, CANNON_BALL_RADIUS);
    dBodySetMass(cannon_ball_body, &m);
    dGeomSetBody(cannon_ball_geom, cannon_ball_body);
    dBodySetPosition(cannon_ball_body, 2, 2, CANNON_BALL_RADIUS);
    */

    // environment
    ground_box = dCreateBox(space, 2, 1.5, 1);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 1, 0, -0.15);
    dGeomSetPosition(ground_box, 2, 0, -0.34);
    dGeomSetRotation(ground_box, R);

    // run simulation
    dsSimulationLoop(argc, argv, 1000, 800, &fn);

    destroyBuggy(box1,sphere1,turrgeom1);
    destroyBuggy(box2,sphere2,turrgeom2);

    dGeomDestroy(ground_box);
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
