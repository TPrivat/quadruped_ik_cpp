// The inverse kinematics for a quadruped robot
// This is a translation of the python functions originally obtained from
// https://spotmicroai.readthedocs.io/en/latest/kinematic/
// Translation by Taylor Privat

#ifndef QUAD_IK
#define QUAD_IK

#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Dense>

#define PI 3.14159

using namespace std;
using namespace Eigen;

// ----- Data types needed ------

// Stores joint angles of one leg
typedef struct {
	float ang1;
	float ang2;
	float ang3;
} JointAngles ;

// Stores the link lengths of one leg
typedef struct {
	float l1;
	float l2;
	float l3;
	float l4;
} LinkLens ;

// Transform matrices from bodyIK
typedef struct {
	Matrix4f rb;	// Right Back
	Matrix4f rf;	// Right Front
	Matrix4f lb;	// Left Back
	Matrix4f lf;	// Left Front
	Matrix4f tm;	// Transform Matrix
} TransformMatrices ;

// ------ Functions ----------

JointAngles legIK(float x, float y, float z, LinkLens links);

Matrix<float, 5, 4> calcLegPoints(JointAngles angles, LinkLens links);

TransformMatrices bodyIK(float omega, float phi, float psi,
                         float x, float y, float z,
                         float L, float W);

#endif
