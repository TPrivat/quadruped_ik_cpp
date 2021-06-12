// The inverse kinematics for a quadruped robot
// This is a translation of the python functions originally obtained from
// https://spotmicroai.readthedocs.io/en/latest/kinematic/
// Translation by Taylor Privat

#ifndef QUAD_IK
#define QUAD_IK

#include <iostream>
#include <math.h>
#include <vector>

#define pi 3.14159

using namespace std;

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
	vector<vector<float>> rb;	// Right Back
	vector<vector<float>> rf;	// Right Front
	vector<vector<float>> lb;	// Left Back
	vector<vector<float>> lf;	// Left Front
	vector<vector<float>> tm;	// Transform Matrix
} TransformMatrices ;

// ------ Functions ----------

vector<vector<float>> matmul(vector<vector<float>> a, vector<vector<float>> b);

vector<float> mvmul(vector<vector<float>> matrix, vector<float> vector);

JointAngles legIK(float x, float y, float z, LinkLens links);

vector<vector<float>> calcLegPoints(JointAngles angles, LinkLens links);

TransformMatrices bodyIK(float omega, float phi, float psi,
                         float x, float y, float z,
                         float L, float W);

#endif
