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

#define LENGTH 0.7	// chassis length
#define WIDTH 0.5	// chassis width
#define HEIGHT 0.2	// chassis height
#define RADIUS 0.3	// wheel radius
#define STARTZ 0.5	// starting height of chassis
#define CMASS 1		// chassis mass
#define WMASS 0.2	// wheel mass

//box
#define BOXLENGTH 0.15	// box length
#define BOXWIDTH 0.15	// box width
#define BOXHEIGHT 0.3	// box height
#define BMASS 0.1		// box mass

#define SPHERERADIUS 0.4 // sphere radius
//turret
#define TURRRADIUS    0.1
#define TURRLENGTH    0.6

static const dVector3 yunit = { 0, 1, 0 }, zunit = { 0, 0, 1 };


// dynamics and collision objects (chassis, 4 wheels, environment)

static dWorldID world;
static dSpaceID space;
static dBodyID chassis[1];
//static dJointID jointChassis[1];	// joint[0] is the front wheel
static dBodyID roues[4];
static dJointID jointChassis_roues[4]; // jointChassis_roues[0] & jointChassis_roues[1] are the front wheels
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space;
static dGeomID box[1];
static dGeomID sphere[4];
static dGeomID ground_box;
static dGeomID ground_box2;

//shere :
static dBodyID sphbody;
static dGeomID sphgeom;
//box 
static dBodyID boxbody;
static dGeomID boxgeom;

static dJointID joint_box;
//turret
static dBodyID turrbody[1];
static dGeomID turrgeom;
static dJointID joint_turr;

// things that the user controls
static dReal speed = 0, steer = 0;	// user commands
static bool lock_cam = true;

static dReal steerCanon = 0;

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback(void*, dGeomID o1, dGeomID o2)
{
    int i, n;
    // Pour prendre en compte toutes les collisisons, commenter les 3 lignes ci dessous
    /* 
    // only collide things with the ground
    int g1 = (o1 == ground || o1 == ground_box || o1 == ground_box2 );
    int g2 = (o2 == ground || o2 == ground_box || o2 == ground_box2 );
    if (!(g1 ^ g2)) return;
    */
    if (o1 == box[0] || o1 == turrgeom && o2 == box[0] || o2 == turrgeom) {
        return;
    }
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


// start simulation - set viewpoint

static float xyz[3] = { 0,0,0 };
static float hpr[3] = { 0,-10,0 };

static void start()
{
    dAllocateODEDataForThread(dAllocateMaskAll);

    dsSetViewpoint(xyz, hpr);
    printf("Press:\t'a' to increase speed.\n"
        "\t's' to decrease speed.\n"
        "\t'q' to steer left.\n"
        "\t'd' to steer right.\n"
        "\t' ' to reset speed and steering.\n"
        "\t'1' to save the current state to 'state.dif'.\n");
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
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 0, 0, 0);
    dBodySetRotation(obj_body, R);
    for (int i = 0; i <= 3; i++) {
        pos = dBodyGetPosition(roues[i]);
        dBodySetPosition(roues[i], pos[0], pos[1], pos[2] + 1);
        dMatrix3 R;
        dRFromAxisAndAngle(R, 0, 0, 0, 0);
        dBodySetRotation(roues[i], R);
    }
    
    
}

// called when a key pressed

static void command(int cmd)
{
    switch (cmd) {
    case 'z': case 'Z':
        if (!(speed >= 3.3)) {
            speed += 0.3;
        }
        break;
    case 's': case 'S':
        if (!(speed <= -3.3)) {
            speed -= 0.3;
        }
        break;
    case 'q': case 'Q':
        steer -= 0.3;
        break;
    case 'd': case 'D':
        steer += 0.3;
        break;
    case 'u': case 'U':
        steerCanon -= 0.3;
        break;
    case 'i': case 'I':
        steerCanon += 0.3;
        break;
    case 'l': case 'L':
        lock_cam = !lock_cam;
        break;
    case '5':
        retourner(chassis[0]);
        break;
    case ' ':
        speed = 0;
        steer = 0;
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


// simulation loop

static void simLoop(int pause)
{
    int i;
    if (!pause) {
        // motor
        dJointSetHinge2Param(jointChassis_roues[0], dParamVel2, -speed*2);
        dJointSetHinge2Param(jointChassis_roues[0], dParamFMax2, 0.2);

        dJointSetHinge2Param(jointChassis_roues[1], dParamVel2, -speed*2);
        dJointSetHinge2Param(jointChassis_roues[1], dParamFMax2, 0.2);

        // steering roue 1
        dReal v1 = steer - dJointGetHinge2Angle1(jointChassis_roues[0]);
        if (v1 > 0.1) v1 = 0.1;
        if (v1 < -0.1) v1 = -0.1;
        v1 *= 10.0;
        dJointSetHinge2Param(jointChassis_roues[0], dParamVel, v1);
        dJointSetHinge2Param(jointChassis_roues[0], dParamFMax, 0.2);
        dJointSetHinge2Param(jointChassis_roues[0], dParamLoStop, -0.75);
        dJointSetHinge2Param(jointChassis_roues[0], dParamHiStop, 0.75);
        dJointSetHinge2Param(jointChassis_roues[0], dParamFudgeFactor, 0.1);

        //steering roue 2
        dReal v2 = steer - dJointGetHinge2Angle1(jointChassis_roues[1]);
        if (v2 > 0.1) v2 = 0.1;
        if (v2 < -0.1) v2 = -0.1;
        v2 *= 10.0;
        dJointSetHinge2Param(jointChassis_roues[1], dParamVel, v2);
        dJointSetHinge2Param(jointChassis_roues[1], dParamFMax, 0.2);
        dJointSetHinge2Param(jointChassis_roues[1], dParamLoStop, -0.75);
        dJointSetHinge2Param(jointChassis_roues[1], dParamHiStop, 0.75);
        dJointSetHinge2Param(jointChassis_roues[1], dParamFudgeFactor, 0.1);

        dReal v3 = steerCanon - dJointGetHinge2Angle1(joint_turr);
        if (v3 > 0.2) v3 = 0.2;
        if (v3 < -0.2) v3 = -0.2;
        v3 *= 10.0;
        dJointSetHinge2Param(joint_turr, dParamVel, v3);
        dJointSetHinge2Param(joint_turr, dParamFMax, 0.2);
        dJointSetHinge2Param(joint_turr, dParamLoStop, -0.75);
        dJointSetHinge2Param(joint_turr, dParamHiStop, 0.75);
        dJointSetHinge2Param(joint_turr, dParamFudgeFactor, 0.1);


        dSpaceCollide(space, 0, &nearCallback);
        dWorldStep(world, 0.05);

        // remove all contact joints
        dJointGroupEmpty(contactgroup);

        // fixed or not-fixed camera
        if (lock_cam) {
            camPos(chassis[0]);
        }
    }

    dsSetColor(0, 1, 1);
    dsSetTexture(DS_WOOD);
    dReal sides[3] = { LENGTH,WIDTH,HEIGHT };
    dsDrawBox(dBodyGetPosition(chassis[0]), dBodyGetRotation(chassis[0]), sides);
    dsSetColor(1, 1, 1);
    for (i = 0; i <= 3; i++) dsDrawCylinder(dBodyGetPosition(roues[i]),
        dBodyGetRotation(roues[i]), 0.08f, RADIUS);

    dReal boxsides[3] = { BOXLENGTH,BOXWIDTH,BOXHEIGHT };
    dsDrawBox(dBodyGetPosition(boxbody), dBodyGetRotation(boxbody), boxsides);


    dVector3 ss;
    dGeomBoxGetLengths(ground_box, ss);
    dsDrawBox(dGeomGetPosition(ground_box), dGeomGetRotation(ground_box), ss);

    dVector3 ss2;
    dGeomBoxGetLengths(ground_box2, ss2);
    dsDrawBox(dGeomGetPosition(ground_box2), dGeomGetRotation(ground_box2), ss2);



    //sphere
    const dReal* SPos = dBodyGetPosition(sphbody);
    const dReal* SRot = dBodyGetRotation(sphbody);
    const double spos[3] = { SPos[0], SPos[1], SPos[2] };
    const double srot[12] = { SRot[0], SRot[1], SRot[2], SRot[3], SRot[4], SRot[5], SRot[6], SRot[7], SRot[8], SRot[9], SRot[10], SRot[11] };
    dsDrawSphere
    (
        spos,
        srot,
        SPHERERADIUS
    ); // single precision
    dsSetColor(0, 1, 1);
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

    // chassis body
    chassis[0] = dBodyCreate(world);
    dBodySetPosition(chassis[0], 0, 0, STARTZ);
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
    dBodySetPosition(roues[0], 0.5 * LENGTH, WIDTH * 0.6, STARTZ - HEIGHT * 0.5);
    dBodySetPosition(roues[1], 0.5 * LENGTH, -WIDTH * 0.6, STARTZ - HEIGHT * 0.5);
    dBodySetPosition(roues[2], -0.5 * LENGTH, WIDTH * 0.6, STARTZ - HEIGHT * 0.5);
    dBodySetPosition(roues[3], -0.5 * LENGTH, -WIDTH * 0.6, STARTZ - HEIGHT * 0.5);


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

    // create car space and add it to the top level space
    car_space = dSimpleSpaceCreate(space);
    dSpaceSetCleanup(car_space, 0);
    dSpaceAdd(car_space, box[0]);
    dSpaceAdd(car_space, sphere[0]);
    dSpaceAdd(car_space, sphere[1]);
    dSpaceAdd(car_space, sphere[2]);
    dSpaceAdd(car_space, sphere[3]);

    // environment
    ground_box = dCreateBox(space, 2, 1.5, 1);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 1, 0, -0.15);
    dGeomSetPosition(ground_box, 2, 0, -0.34);
    dGeomSetRotation(ground_box, R);

    // environment 2nd box
    ground_box2 = dCreateBox(space, 200, 2, 1);
    dMatrix3 R2;
    dRFromAxisAndAngle(R2, 0, 1, 0, 0);
    dGeomSetPosition(ground_box2, 100,0, -0.47);
    dGeomSetRotation(ground_box2, R2);

 

    // sphere
    sphbody = dBodyCreate(world);
    dMassSetSphere(&m, 0.2, SPHERERADIUS);
    dBodySetMass(sphbody, &m);
    sphgeom = dCreateSphere(0, SPHERERADIUS);
    dGeomSetBody(sphgeom, sphbody);
    dBodySetPosition(sphbody, 0, 0, 5.5);
    //dSpaceAdd(space, sphgeom);

    // box body
    boxbody = dBodyCreate(world);
    dBodySetPosition(boxbody, 0, 0, STARTZ + 1);
    dMassSetBox(&m, 1, BOXLENGTH, BOXWIDTH, BOXHEIGHT);
    dMassAdjust(&m, BMASS);
    dBodySetMass(boxbody, &m);
    boxgeom = dCreateBox(0, BOXLENGTH, BOXWIDTH, BOXHEIGHT);
    dGeomSetBody(boxgeom, boxbody);
    dBodySetPosition(boxbody, 0, 0, STARTZ + HEIGHT);
    dSpaceAdd(space, boxgeom);
    




    /*
    // joint box
    joint_box = dJointCreateHinge(world, 0);
    dJointAttach(joint_box, chassis[0], boxbody);
    const dReal* box_pos = dBodyGetPosition(boxbody);
    dJointSetHingeAnchor(joint_box, box_pos[0], box_pos[1], box_pos[2]);
    dJointSetHinge2Axes(joint_box, zunit, yunit);*/
    /*
    joint_box = dJointCreateHinge(world, 0);
    dJointAttach(joint_box, chassis[0], boxbody);
    dJointSetHingeAnchor(joint_box, 0, 0, 0);
    dJointSetHingeAxis(joint_box, 1, -1, 1.41421356);*/

    joint_box = dJointCreateFixed(world, 0);
    dJointAttach(joint_box, chassis[0], boxbody);
    dJointSetFixed(joint_box);
   
    //TURRET
    turrbody[0] = dBodyCreate(world);
    dQuaternion q;
    //dQFromAxisAndAngle(q, 1, 0, 0, M_PI * -0.77);
   // dBodySetQuaternion(turrbody[0], q);
    dMassSetCylinder(&m, 1.0, 3, TURRRADIUS, TURRLENGTH);
    dBodySetMass(turrbody[0], &m);
    turrgeom = dCreateCylinder(0, TURRRADIUS, TURRLENGTH);
    dGeomSetBody(turrgeom, turrbody[0]);
    dBodySetPosition(turrbody[0], 0,0, STARTZ + HEIGHT +BOXHEIGHT-0.1 );
    dSpaceAdd(space, turrgeom);
    dMatrix3 Rbox;
    dRFromAxisAndAngle(Rbox,90,90,90,90);
    dGeomSetRotation(turrgeom, Rbox);
    
    // joint turret
    /*joint_turr = dJointCreateFixed(world, 0);
    dJointAttach(joint_turr, turrbody[0], boxbody);
    dJointSetFixed(joint_turr);*/
    
    joint_turr = dJointCreateHinge2(world, 0);
    dJointAttach(joint_turr, boxbody, turrbody[0]);
    const dReal* a = dBodyGetPosition(turrbody[0]);
    dJointSetHinge2Anchor(joint_turr, a[0], a[1], a[2]);
    dJointSetHinge2Axes(joint_turr, zunit, yunit);


    // run simulation
    dsSimulationLoop(argc, argv, 1000, 800, &fn);

    dGeomDestroy(box[0]);
    dGeomDestroy(sphere[0]);
    dGeomDestroy(sphere[1]);
    dGeomDestroy(sphere[2]);
    dGeomDestroy(sphere[3]);

    dGeomDestroy(sphgeom);
    dGeomDestroy(boxgeom);

    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);

    

    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
