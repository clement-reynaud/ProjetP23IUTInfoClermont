#pragma once
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "math.h"
#include "creationBuggy.h"

void initilizeBuggyDimentions(Buggy* buggies, dReal Ilength, dReal Iwidth, dReal Iheight, float Iradius, float IcylinderLenght, float IcylinderRadius, float IboxLenght, float IboxWidth, float IboxHeight, float IsphereRadius);
void drawBuggyWithPos(const dReal** positions, Buggy buggies);