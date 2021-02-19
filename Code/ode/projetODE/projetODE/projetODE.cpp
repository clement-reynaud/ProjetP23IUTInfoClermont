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

#pragma once
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "math.h"
#include <cmath>
#include "projetODE.h"
#include <random>
#include <algorithm>
#include <iterator>
#include "creationBuggy.h"
#include "deplaceBuggy.h"
#include "projetODE.h"
#include "client.h"
#include "terrain.h";

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

 // select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawTriangle dsDrawTriangleD
#endif

// some constants
#define LENGTH 0.7	// chassis length
#define WIDTH 0.5	// chassis width
#define HEIGHT 0.2	// chassis height
#define RADIUS 0.3	// wheel radius
#define CMASS 1		// chassis mass
#define WMASS 0.2	// wheel mass
#define STARTZ 2	// starting height of chassis

//Turret def
//canon
#define TURRRADIUS    0.1
#define TURRLENGTH    0.6

//box
#define BOXLENGTH 0.15    // box length
#define BOXWIDTH 0.15    // box width
#define BOXHEIGHT 0.3    // box height
#define BMASS 0.1        // box mass

//Bullet
#define SPHERERADIUS 0.2

static const dVector3 yunit = { 0, 1, 0 }, zunit = { 0, 0, 1 };

Buggy buggy[6];
Turret turr[6];
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;
static dGeomID ground_box;

//custom
static dGeomID obstacle[4];

static dBodyID sphbody;
static dGeomID sphgeom;

static dGeomID ground;

// things that the user controls

static dReal speed = 0, steer = 0;	// user commands
dReal speedBuggy1 = 0;
dReal speedBuggy2 = 0;
dReal steerBuggy1 = 0;
dReal steerBuggy2 = 0;

static bool lock_cam1 = true;
static bool lock_cam2 = false;
static dReal cannon_angle = 0, cannon_elevation = -1.2;

bool tp;

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback(void*, dGeomID o1, dGeomID o2)
{
    int i, n;
    
    // only collide things with the ground
    int g1 = (o1 == ground || o1 == ground_box);
    int g2 = (o2 == ground || o2 == ground_box);

    if (o1 == ground_box || o2 == ground_box) {
        const int N = 10;
        dContact contact[N];
        n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
        if (n > 0) {
            for (i = 0; i < n; i++) {
                contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                    dContactSoftERP | dContactSoftCFM | dContactApprox1;
                contact[i].surface.mu = dInfinity;
                contact[i].surface.slip1 = 2;
                contact[i].surface.slip2 = 2;
                contact[i].surface.soft_erp = 0.5;
                contact[i].surface.soft_cfm = 0.3;
                dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
                dJointAttach(c,
                    dGeomGetBody(contact[i].geom.g1),
                    dGeomGetBody(contact[i].geom.g2));
            }
        }
    }
    
    if (o1 == sphgeom || o2 == sphgeom) {
        const int N = 10;
        dContact contact[N];
        n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
        if (n > 0) {
            for (i = 0; i < n; i++) {
                contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                    dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
                contact[i].surface.mu = dInfinity;
                contact[i].surface.slip1 = 0.1;
                contact[i].surface.slip2 = 0.1;
                contact[i].surface.soft_erp = 0.5;
                contact[i].surface.soft_cfm = 0.3;
                contact[i].surface.bounce = 1.65;
                contact[i].surface.bounce_vel = 1;
                dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
                dJointAttach(c,
                    dGeomGetBody(contact[i].geom.g1),
                    dGeomGetBody(contact[i].geom.g2));
            }
        }
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

static float xyz[3] = { 0,-90,0 };
static float hpr[3] = { 0.8317f,-0.9817f,0.8000f };

static void start()
{
    buggy[0].moveBuggy.lock_cam = true;
    initTerrain();

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
    );
}

static void command(int cmd)
{

    switch (cmd) {
        case 'z':
            if (!(buggy[0].moveBuggy.speedBuggy >= 10)) {
                buggy[0].moveBuggy.speedBuggy += 0.5;
            }
            break;
        case 'Z':
            if (!(buggy[1].moveBuggy.speedBuggy >= 10)) {
                buggy[1].moveBuggy.speedBuggy += 0.5;
            }
            break;
        case 's':
            if (!(buggy[0].moveBuggy.speedBuggy <= -10)) {
                buggy[0].moveBuggy.speedBuggy -= 0.5;
            }
            break;
        case 'S':
            if (!(buggy[1].moveBuggy.speedBuggy <= -10)) {
                buggy[1].moveBuggy.speedBuggy -= 0.5;
            }
            break;
        case 'q':
            buggy[0].moveBuggy.steerBuggy -= 0.3;
            break;
        case 'Q':
            buggy[1].moveBuggy.steerBuggy -= 0.3;
            break;
        case 'd':
            buggy[0].moveBuggy.steerBuggy += 0.3;
            break;
        case 'D':
            buggy[1].moveBuggy.steerBuggy += 0.3;
            break;
        case 't':
            tirer(&buggy[0], SPHERERADIUS, space, world);
            break;
        case'T':
            tirer(&buggy[1], SPHERERADIUS, space, world);
            break;
        case 'l':
            if (buggy[1].moveBuggy.lock_cam == true) {
                buggy[1].moveBuggy.lock_cam = false;
            }
            buggy[0].moveBuggy.lock_cam = !buggy[0].moveBuggy.lock_cam;
            break;
        case 'L':
            if (buggy[0].moveBuggy.lock_cam == true) {
                buggy[0].moveBuggy.lock_cam = false;
            }
            buggy[1].moveBuggy.lock_cam = !buggy[1].moveBuggy.lock_cam;
            break;
        case '5':
            retournerBuggy(buggy[0]);
            break;
        case '6':
            retournerBuggy(buggy[1]);
            break;
        case ' ':
            arreterBuggy(buggy[0]);
            arreterBuggy(buggy[1]);
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
        deplacementBuggy(&buggy[0]);
        deplacementBuggy(&buggy[1]);

        dSpaceCollide(space, 0, &nearCallback);
        dWorldStep(world, 0.05);

        // remove all contact joints
        dJointGroupEmpty(contactgroup);

        // fixed or not-fixed camera
        if (buggy[0].moveBuggy.lock_cam) {
            camPos(buggy[0], xyz, hpr);
        }

        if (buggy[1].moveBuggy.lock_cam) {
            camPos(buggy[1], xyz, hpr);
        }
    }
    dReal sides[3] = { LENGTH,WIDTH,HEIGHT };

    //Draw buggy 1 & 2
    dsSetTexture(DS_WOOD);
    drawBuggy(buggy[0], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);
    
    dsSetTexture(DS_WOOD);
    drawBuggy(buggy[1], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);
    
    //Draw buggy 3, 4, 5 & 6
    dsSetTexture(DS_WOOD);
    drawBuggy(buggy[2], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);

    dsSetTexture(DS_WOOD);
    drawBuggy(buggy[3], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);
    
    dsSetTexture(DS_WOOD);
    drawBuggy(buggy[4], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);
    
    dsSetTexture(DS_WOOD);
    drawBuggy(buggy[5], sides, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, SPHERERADIUS, RADIUS);
    dsSetColor(1, 1, 1);
    
    dVector3 ss;
    dsSetColor(0.5, 0.5, 1);
    dGeomBoxGetLengths(ground_box, ss);
    dsDrawBox(dGeomGetPosition(ground_box), dGeomGetRotation(ground_box), ss);
    

    dsSetTexture(NULL);
    dsSetColorAlpha(0, 0, 0, 1);
    for (i = 0; i < 4; i++) {
        dGeomBoxGetLengths(obstacle[i], ss);
        dsDrawBox(dGeomGetPosition(obstacle[i]), dGeomGetRotation(obstacle[i]), ss);
    }

    

    dsSetColorAlpha(0, 1, 0, 1);
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

    drawHFieldGeom(getHField(), 0, 0, 0);
}

int main(int argc, char** argv){
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

    buggy[0].colors[0] = 0; buggy[0].colors[1] = 1; buggy[0].colors[2] = 1;
    createABuggy(&buggy[0], m, BMASS, CMASS, WMASS, LENGTH, WIDTH, HEIGHT, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, RADIUS, -1, 0, STARTZ, yunit, zunit, space, world);
    
    buggy[1].colors[0] = 1; buggy[1].colors[1] = 0; buggy[1].colors[2] = 0;
    createABuggy(&buggy[1], m, BMASS, CMASS, WMASS, LENGTH, WIDTH, HEIGHT, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, RADIUS, -5, 0, STARTZ, yunit, zunit, space, world);
    
    buggy[2].colors[0] = 0; buggy[2].colors[1] = 1; buggy[2].colors[2] = 0;
    createABuggy(&buggy[2], m, BMASS, CMASS, WMASS, LENGTH, WIDTH, HEIGHT, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, RADIUS, -5, 3, STARTZ, yunit, zunit, space, world);
    
    buggy[3].colors[0] = 0; buggy[3].colors[1] = 0; buggy[3].colors[2] = 1;
    createABuggy(&buggy[3], m, BMASS, CMASS, WMASS, LENGTH, WIDTH, HEIGHT, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, RADIUS, -5, 6, STARTZ, yunit, zunit, space, world);
    
    buggy[4].colors[0] = 1; buggy[4].colors[1] = 1; buggy[4].colors[2] = 0;
    createABuggy(&buggy[4], m, BMASS, CMASS, WMASS, LENGTH, WIDTH, HEIGHT, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, RADIUS, -5, -3, STARTZ, yunit, zunit, space, world);
    
    buggy[5].colors[0] = 1; buggy[5].colors[1] = 0; buggy[5].colors[2] = 1;
    createABuggy(&buggy[5], m, BMASS, CMASS, WMASS, LENGTH, WIDTH, HEIGHT, BOXLENGTH, BOXWIDTH, BOXHEIGHT, TURRLENGTH, TURRRADIUS, RADIUS, -5, -6, STARTZ, yunit, zunit, space, world);

    // environment
    ground_box = dCreateBox(space, 2, 2, 1);
    //dMatrix3 R;
    //dRFromAxisAndAngle(R, 0, 1, 0, -0.15);
    dGeomSetPosition(ground_box, 18, 15, -0.47);
    //dGeomSetRotation(ground_box, R);

    // sphere
    sphbody = dBodyCreate(world);
    dMassSetSphere(&m, 0.2, SPHERERADIUS);
    dBodySetMass(sphbody, &m);
    sphgeom = dCreateSphere(0, SPHERERADIUS);
    dGeomSetBody(sphgeom, sphbody);
    dBodySetPosition(sphbody, 15, 15, 2);
    dSpaceAdd(space, sphgeom);

    // obstacle
    for (i = 0; i < 4; i++) {
        if (i % 2 == 0) {
            obstacle[i] = dCreateBox(space, 2, 1, 10);
        }
        else {
            obstacle[i] = dCreateBox(space, 2, 10, 1);
        }
        
    }

    dGeomSetPosition(obstacle[1], 5, 0, 1+100);
    dGeomSetPosition(obstacle[3], -5, 0, 1+100);
    dGeomSetPosition(obstacle[0], 0, 5, 1+100);
    dGeomSetPosition(obstacle[2], 0, -5, 1+100);

    dMatrix3 R1;
    dRFromAxisAndAngle(R1, 0, 1, 0, 1.55);

    for (i = 0; i < 4; i++) {
        dGeomSetRotation(obstacle[i], R1);
    }

    createHField(space);
    
    // run simulation
    dsSimulationLoop(argc, argv, 1000, 800, &fn);

    destroyBuggy(buggy[0]);
    destroyBuggy(buggy[1]);
    destroyBuggy(buggy[2]);
    destroyBuggy(buggy[3]);
    destroyBuggy(buggy[4]);
    destroyBuggy(buggy[5]);

    dGeomDestroy(ground_box);
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
