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

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


#define	DEGTORAD			0.01745329251994329577f				//!< PI / 180.0, convert degrees to radians

int g_allow_trimesh;

// Our heightfield geom
dGeomID gheight;



// Heightfield dimensions

#define HFIELD_WSTEP			50		// Vertex count along edge >= 2
#define HFIELD_DSTEP			50

#define HFIELD_WIDTH			REAL( 20.0 )
#define HFIELD_DEPTH			REAL( 20.0 )

#define HFIELD_WSAMP			( HFIELD_WIDTH / ( HFIELD_WSTEP-1 ) )
#define HFIELD_DSAMP			( HFIELD_DEPTH / ( HFIELD_DSTEP-1 ) )


  // select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#define dsDrawTriangle dsDrawTriangleD
#endif


// some constants

#define NUM 100			// max number of objects
#define DENSITY (5.0)		// density of all objects
#define GPB 3			// maximum number of geometries per body
#define MAX_CONTACTS 64		// maximum number of contact points per body


// dynamics and collision objects

struct MyObject {
    dBodyID body;			// the body
    dGeomID geom[GPB];		// geometries representing this body

    // Trimesh only - double buffered matrices for 'last transform' setup
    dReal matrix_dblbuff[16 * 2];
    int last_matrix_index;
};

static int num = 0;		// number of objects in simulation
static int nextobj = 0;		// next object to recycle if num==NUM
static dWorldID world;
static dSpaceID space;
static MyObject obj[NUM];
static dJointGroupID contactgroup;
static int selected = -1;	// selected object
static int show_aabb = 0;	// show geom AABBs?
static int show_contacts = 0;	// show contact points?
static int random_pos = 1;	// drop objects from random position?
static int write_world = 0;


//============================

float RandomFloat(float min, float max, bool recommence, float value) {
    if (recommence == false) {
        return value;
    }
    return  (max - min) * ((((float)rand()) / (float)RAND_MAX)) + min;
}

dReal heightfield_callback(void*, int x, int z)
{
    bool recommence = false;
    int value = 13;
    int r = RandomFloat(-3, 20, recommence, value);
    recommence = false;
    int ra = rand() % 10;
    int ran = rand() % 10;
    int random = rand() % 10;
    dReal fx = (((dReal)x) - (HFIELD_WSTEP + 1) / 2) / (dReal)(HFIELD_WSTEP + 1);
    dReal fz = (((dReal)z) - (HFIELD_DSTEP + 1) / 2) / (dReal)(HFIELD_DSTEP - 1);

    // Create an interesting 'hump' shape
    if (value == 13) {
        value = r;
    }
    dReal h = REAL(1.0) + (REAL(13) * (fx * fx * fz + fz * fz));

    return h;
}




// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback(void*, dGeomID o1, dGeomID o2)
{
    int i;
    // if (o1->body && o2->body) return;

    // exit without doing anything if the two bodies are connected by a joint
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
        return;

    dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
    for (i = 0; i < MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.mu2 = 0;
        contact[i].surface.bounce = 0.1;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.01;
    }
    if (int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
        sizeof(dContact))) {
        dMatrix3 RI;
        dRSetIdentity(RI);
        const dReal ss[3] = { 0.02,0.02,0.02 };
        for (i = 0; i < numc; i++) {
            dJointID c = dJointCreateContact(world, contactgroup, contact + i);
            dJointAttach(c, b1, b2);
            if (show_contacts) {
                dsSetColor(0, 0, 1);
                dsDrawBox(contact[i].geom.pos, RI, ss);
            }
        }
    }
}
// draw a geom

void drawGeom(dGeomID g, const dReal* pos, const dReal* R, int show_aabb)
{
    if (!g)
        return;
    if (!pos)
        pos = dGeomGetPosition(g);
    if (!R)
        R = dGeomGetRotation(g);

    int type = dGeomGetClass(g);

    // Set ox and oz to zero for DHEIGHTFIELD_CORNER_ORIGIN mode.
    int ox = (int)(-HFIELD_WIDTH / 2);
    int oz = (int)(-HFIELD_DEPTH / 2);

    //	for ( int tx = -1; tx < 2; ++tx )
    //	for ( int tz = -1; tz < 2; ++tz )
    dsSetColorAlpha(0.5, 1, 0.5, 0.5);
    dsSetTexture(DS_WOOD);

    for (int i = 0; i < HFIELD_WSTEP - 1; ++i)
        for (int j = 0; j < HFIELD_DSTEP - 1; ++j) {
            dReal a[3], b[3], c[3], d[3];

            a[0] = ox + (i)*HFIELD_WSAMP;
            a[1] = heightfield_callback(NULL, i, j);
            a[2] = oz + (j)*HFIELD_DSAMP;

            b[0] = ox + (i + 1) * HFIELD_WSAMP;
            b[1] = heightfield_callback(NULL, i + 1, j);
            b[2] = oz + (j)*HFIELD_DSAMP;

            c[0] = ox + (i)*HFIELD_WSAMP;
            c[1] = heightfield_callback(NULL, i, j + 1);
            c[2] = oz + (j + 1) * HFIELD_DSAMP;

            d[0] = ox + (i + 1) * HFIELD_WSAMP;
            d[1] = heightfield_callback(NULL, i + 1, j + 1);
            d[2] = oz + (j + 1) * HFIELD_DSAMP;

            dsDrawTriangle(pos, R, a, c, b, 2);
            dsDrawTriangle(pos, R, b, c, d, 1);
        }

    if (show_aabb) {
        // draw the bounding box for this geom
        dReal aabb[6];
        dGeomGetAABB(g, aabb);
        dVector3 bbpos;
        for (int i = 0; i < 3; i++)
            bbpos[i] = 0.5 * (aabb[i * 2] + aabb[i * 2 + 1]);
        dVector3 bbsides;
        for (int i = 0; i < 3; i++)
            bbsides[i] = aabb[i * 2 + 1] - aabb[i * 2];
        dMatrix3 RI;
        dRSetIdentity(RI);
        dsSetColorAlpha(1, 0, 0, 0.5);
        dsDrawBox(bbpos, RI, bbsides);
    }

}

// simulation loop
/*
static void simLoop(int pause)
{
    int i, j;

    dSpaceCollide(space, 0, &nearCallback);

    if (!pause)
        dWorldQuickStep(world, 0.05);


    if (write_world) {
        FILE* f = fopen("state.dif", "wt");
        if (f) {
            dWorldExportDIF(world, f, "X");
            fclose(f);
        }
        write_world = 0;
    }

    // remove all contact joints
    dJointGroupEmpty(contactgroup);



    //
    // Draw Heightfield
    //

    drawGeom(gheight, 0, 0, 0);



    dsSetColor(1, 1, 0);
    dsSetTexture(DS_WOOD);
    for (i = 0; i < num; i++) {
        for (j = 0; j < GPB; j++) {
            if (i == selected) {
                dsSetColor(0, 0.7, 1);
            }
            else if (!dBodyIsEnabled(obj[i].body)) {
                dsSetColor(1, 0.8, 0);
            }
            else {
                dsSetColor(1, 1, 0);
            }

            drawGeom(obj[i].geom[j], 0, 0, show_aabb);
        }
    }

}*/

/*
int main(int argc, char** argv)
{
    printf("ODE configuration: %s\n", dGetConfiguration());

    // Is trimesh support built into this ODE?
    g_allow_trimesh = dCheckConfiguration("ODE_EXT_trimesh");

    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    //fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

    // create world
    dInitODE2(0);
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, 0, -0.05);
    dWorldSetCFM(world, 1e-5);
    dWorldSetAutoDisableFlag(world, 1);
    dWorldSetContactMaxCorrectingVel(world, 0.1);
    dWorldSetContactSurfaceLayer(world, 0.001);
    memset(obj, 0, sizeof(obj));

    dWorldSetAutoDisableAverageSamplesCount(world, 1);

    // base plane to catch overspill
    dCreatePlane(space, 0, 0, 1, 0);


    // our heightfield floor

    dHeightfieldDataID heightid = dGeomHeightfieldDataCreate();

    // Create an finite heightfield.
    dGeomHeightfieldDataBuildCallback(heightid, NULL, heightfield_callback,
        HFIELD_WIDTH, HFIELD_DEPTH, HFIELD_WSTEP, HFIELD_DSTEP,
        REAL(1.0), REAL(0.0), REAL(0.0), 0);

    // Give some very bounds which, while conservative,
    // makes AABB computation more accurate than +/-INF.
    dGeomHeightfieldDataSetBounds(heightid, REAL(-4.0), REAL(+6.0));

    gheight = dCreateHeightfield(space, heightid, 1);

    dVector3 pos;
    pos[0] = 0;
    pos[1] = 0;
    pos[2] = 0;

    // Rotate so Z is up, not Y (which is the default orientation)
    dMatrix3 R;
    dRSetIdentity(R);
    dRFromAxisAndAngle(R, 1, 0, 0, DEGTORAD * 90);

    // Place it.
    dGeomSetRotation(gheight, R);
    dGeomSetPosition(gheight, pos[0], pos[1], pos[2]);

    dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
    dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
    dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
    // dWorldSetStepIslandsProcessingMaxThreadCount(world, 1);
    dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);

    // run simulation
    dsSimulationLoop(argc, argv, DS_SIMULATION_DEFAULT_WIDTH, DS_SIMULATION_DEFAULT_HEIGHT, &fn);

    dThreadingImplementationShutdownProcessing(threading);
    dThreadingFreeThreadPool(pool);
    dWorldSetStepThreadingImplementation(world, NULL, NULL);
    dThreadingFreeImplementation(threading);

    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);

    // destroy heightfield data, because _we_ own it not ODE
    dGeomHeightfieldDataDestroy(heightid);

    dCloseODE();
}*/
