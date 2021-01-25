#include "client.h"

dReal length;
dReal width;
dReal height;
float radius;
float cylinderLenght;
float cylinderRadius;
float boxLenght; 
float boxWidth;
float boxHeight;
float sphereRadius;

void initilizeBuggyDimentions(Buggy* buggies, dReal Ilength, dReal Iwidth, dReal Iheight, float Iradius, float IcylinderLenght, float IcylinderRadius, float IboxLenght, float IboxWidth, float IboxHeight, float IsphereRadius) {
    length = Ilength;
    width = Iwidth;
    height = Iheight;
    radius = Iradius;
    cylinderLenght = IcylinderLenght;
    cylinderRadius = IcylinderRadius;
    boxLenght = IboxLenght;
    boxWidth = IboxWidth;
    boxHeight = IboxHeight;
    sphereRadius = IsphereRadius;
}

void drawBuggyWithPos(const dReal** positions, int* colors, int nbBulletMax) {
    dsSetColor(colors[0], colors[1], colors[2]);
    dReal sides[3] = { length,width,height };
    dsDrawBox(positions[0], positions[1], sides);
    dsDrawCylinder(positions[2], positions[3], 0.2f, radius);
    dsDrawCylinder(positions[4], positions[5], 0.2f, radius);
    dsDrawCylinder(positions[6], positions[7], 0.2f, radius);
    dsDrawCylinder(positions[8], positions[9], 0.2f, radius);
    dsSetColor(colors[0], colors[1], colors[2]);
    dsDrawCylinder(positions[10], positions[11], cylinderLenght, cylinderRadius);
    dsSetColor(1, 1, 1);
    dReal boxsides[3] = { boxLenght,boxWidth,boxHeight };
    dsDrawBox(positions[12], positions[13], boxsides);
    int i = 14;
    for (int cpt = 0; cpt < nbBulletMax; cpt++) {
        dsDrawSphere(positions[i], positions[i + 1], sphereRadius);
        i = i + 2;
    }
}