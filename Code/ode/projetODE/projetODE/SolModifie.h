#pragma once
#include <math.h>
#include <ode/common.h>

const int Nombre_de_sommets = 11;
const float R = 1.0f;

float x1 = cos(54) * R;
float y = sin(54) * R;
float x2 = cos(18) * R;
float y2 = sin(18) * R;
float x3 = cos(18) * R;
float y3 = -sin(18) * R;
float x4 = cos(54) * R;
float y4 = -sin(54) * R;
float x5 = -cos(54) * R;
float y5 = -sin(54) * R;
float x6 = -cos(18) * R;
float y6 = -sin(18) * R;
float x7 = -cos(18) * R;
float y7 = sin(18) * R;
float x8 = -cos(54) * R;
float y8 = sin(54) * R;


float Arêtes[Nombre_de_sommets * 3] = {
	0.0f, 0.0f, 0.0f,
	R, 0.0f, 0.0f,
	x1, y, 0.0f,
	x2, y2, 0.0f,
	x3, y3, 0.0f,
	x4, y4, 0.0f,
	0.0f, R, 0.0f,
	x5, y5, 0.0f,
	x6, y6, 0.0f,
	x7, y7, 0.0f,
	x8, y8, 0.0f
};

dTriIndex Triangles[Nombre_de_sommets - 1][3] = {
	{0,1,2},
	{0,2,3},
	{0,3,4},
	{0,4,5},
	{0,5,6},
	{0,6,7},
	{0,7,8},
	{0,8,9},
	{0,9,10},
	{0,10,1}
};