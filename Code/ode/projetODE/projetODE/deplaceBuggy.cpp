#include "deplaceBuggy.h"


void deplacementBuggy(Buggy* buggy) {
    speedAndSteer(buggy->jointChassis_roues[0], buggy->moveBuggy);
    speedAndSteer(buggy->jointChassis_roues[1], buggy->moveBuggy);
}

void speedAndSteer(dJointID jointChassis_roues, MoveBuggy moveBuggy) {
    dJointSetHinge2Param(jointChassis_roues, dParamVel2, -moveBuggy.speedBuggy);
    dJointSetHinge2Param(jointChassis_roues, dParamFMax2, 0.1);
    dReal s = moveBuggy.steerBuggy - dJointGetHinge2Angle1(jointChassis_roues);
    if (s > 0.1) s = 0.1;
    if (s < -0.1) s = -0.1;
    s *= 10.0;
    dJointSetHinge2Param(jointChassis_roues, dParamVel, s);
    dJointSetHinge2Param(jointChassis_roues, dParamFMax, 0.2);
    dJointSetHinge2Param(jointChassis_roues, dParamLoStop, -0.75);
    dJointSetHinge2Param(jointChassis_roues, dParamHiStop, 0.75);
    dJointSetHinge2Param(jointChassis_roues, dParamFudgeFactor, 0.1);
}

void retournerBuggy(Buggy buggy) {
    buggy.moveBuggy.steerBuggy = 0;
    buggy.moveBuggy.speedBuggy = 0;
    const dReal* pos;
    pos = dBodyGetPosition(buggy.chassis[0]);
    dBodySetPosition(buggy.chassis[0], pos[0], pos[1], pos[2] + 1);

    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 0, 0, 0);
    dBodySetRotation(buggy.chassis[0], R);
}

void arreterBuggy(Buggy buggy) {
    buggy.moveBuggy.speedBuggy = 0;
    buggy.moveBuggy.steerBuggy = 0;
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

void camPos(Buggy buggy,static float* xyz, static float* hpr) // print a position
{
    const dReal* pos;
    float fx, fy, fz;
    pos = dBodyGetPosition(buggy.chassis[0]); // on prend les coordonnées du chassis
    fx = (float)pos[0]; // x
    fy = (float)pos[1]; // y
    fz = (float)pos[2]; // z

    const dReal* rota = dBodyGetRotation(buggy.chassis[0]); // on prend l'orientation du chassis
    xyz[0] = fx + (rota[0] * (-2)); // on ajoute l
    xyz[1] = fy + (rota[1] * (2));
    xyz[2] = fz + 1;

    float result_cos = calCos(fx, fy, xyz[0], xyz[1], rota);//float result_cos = calCos(fx, fy, xyz[0], xyz[1], rota);
    hpr[0] = result_cos;
    dsSetViewpoint(xyz, hpr);
}

void tirer(Buggy* buggy, float sphereRadius, dSpaceID space, dWorldID world) {
    int i, j, k;
    dMass m;
    if (buggy->bulletMax.num < 5) {
        // new object to be created
        i = buggy->bulletMax.num;
        //printf("nb balles %d",buggy.bulletMax.num);
        buggy->bulletMax.num++;
    }

    else {
        // recycle existing object
        i = buggy->bulletMax.nextobj++;
        buggy->bulletMax.nextobj %= buggy->bulletMax.num; // wrap-around if needed

        // destroy the body and geoms for slot i
        dBodyDestroy(buggy->bulletMax.obj[i].body);
        buggy->bulletMax.obj[i].body = 0;


        if (buggy->bulletMax.obj[i].geom) {
            dGeomDestroy(buggy->bulletMax.obj[i].geom);
            buggy->bulletMax.obj[i].geom = 0;
        }
    }
    buggy->bulletMax.obj[i].body = dBodyCreate(world);
    dMassSetSphere(&m, 0.2, sphereRadius);
    dBodySetMass(buggy->bulletMax.obj[i].body, &m);
    buggy->bulletMax.obj[i].geom = dCreateSphere(0, sphereRadius);
    dGeomSetBody(buggy->bulletMax.obj[i].geom, buggy->bulletMax.obj[i].body);
    dSpaceAdd(space, buggy->bulletMax.obj[i].geom);

    const dReal* cpos = dBodyGetPosition(buggy->chassis[0]);
    dReal pos[3];
    pos[0] = cpos[0];
    pos[1] = cpos[1];
    pos[2] = cpos[2];
    const dReal* rota = dBodyGetRotation(buggy->chassis[0]);
    dBodySetPosition(buggy->bulletMax.obj[i].body, pos[0], pos[1], pos[2] + 0.55);
    int force = 5;
    dBodySetLinearVel(buggy->bulletMax.obj[i].body, -(rota[0] * (-force)), -(rota[1] * (force)), 1);
    dBodySetAngularVel(buggy->bulletMax.obj[i].body, 0, 0, 0);
}