// The inverse kinematics for a quadruped robot
// This is a translation of the python functions originally obtained from
// https://spotmicroai.readthedocs.io/en/latest/kinematic/

#include "quad_ik.h"

vector<vector<float>> matmul(vector<vector<float>> a, vector<vector<float>> b) {
	vector<vector<float>> result(a.size(), vector<float>(a[0].size()));
	for (int i=0; i<a.size(); i++) {
		for (int j=0; j<b.size(); j++) {
			for (int k=0; k<a.size(); k++) {
				result[i][j] += a[i][k] * b[k][j];
			}
		}
	}
	return result;
}


vector<float> mvmul(vector<vector<float>> matrix, vector<float> vec) {
	vector<float> result(vec.size(), 0);
	for (int i=0; i<vec.size(); i++) {
		for (int j=0; j<matrix.size(); j++) {
			result[i] += matrix[i][j] * vec[j];
		}
	}
	return result;
}			
				

JointAngles legIK(float x, float y, float z, LinkLens links) {
	// (x, y, z) -> coordinates of target position for EE
	// f -> len of shoulder-point to target point on x/y
	// g -> len we need to read to point on x/y
	// h -> 3D len we need to reach

	JointAngles ja;

	float f = sqrt(x*x + y*y - links.l1*links.l1);
	float g = f - links.l2;
	float h = sqrt(g*g + z*z);

	ja.ang1 = atan2(y, x) - atan2(f, -1*links.l1);

	float d = (h*h - links.l3*links.l3 - links.l4*links.l4);
	d /= (2 * links.l3 * links.l4);

	ja.ang3 = acos(d);

	ja.ang2 = atan2(z, g) - atan2(links.l4*sin(ja.ang3), links.l3+links.l4*cos(ja.ang3));

	return ja;
}


vector<vector<float>> calcLegPoints(JointAngles angles, LinkLens links) {
	float ang23 = angles.ang2 + angles.ang3;

	vector<float> t0 = {0, 0, 0, 1};
	
	vector<float> t1(t0);
	t1[0] += -1*links.l1*cos(angles.ang1);
	t1[1] += links.l1*sin(angles.ang1);

	vector<float> t2(t1);
	t2[0] += -1*links.l2*sin(angles.ang1);
	t2[1] += -1*links.l2*cos(angles.ang1);

	vector<float> t3(t2);
	t3[0] += -1*links.l3*sin(angles.ang1)*cos(angles.ang2);
	t3[1] += -1*links.l3*cos(angles.ang1)*cos(angles.ang2);
	t3[2] += links.l3*sin(angles.ang2);

	vector<float> t4(t3);
	t4[0] += -1*links.l4*sin(angles.ang1)*cos(ang23);
	t4[1] += -1*links.l4*cos(angles.ang1)*cos(ang23);
	t4[2] += links.l4*sin(ang23);

	vector<vector<float>> LegPoints = {t0, t1, t2, t3, t4};

	return LegPoints;
}


TransformMatrices bodyIK(float omega, float phi, float psi,
                         float x, float y, float z,
                         float L, float W) {
	/*
	Calculate the four Transformation-Matrices for our Legs
    rx=X-Axis Rotation Matrix
    ry=Y-Axis Rotation Matrix
    rz=Z-Axis Rotation Matrix
    rxyz=All Axis Rotation Matrix
    t=Translation Matrix
    tm=Transformation Matrix
    rb,rf,lb,lf=final Matrix for RightBack,RightFront,LeftBack and LeftFront

	L -> body len??
	W -> body width??
	*/

	vector<float> r1 = {1, 0, 0, 0};
	vector<float> r2 = {0, cos(omega), -1*sin(omega), 0};
	vector<float> r3 = {0, sin(omega), cos(omega), 0};
	vector<float> r4 = {0, 0, 0, 1};
	vector<vector<float>> rx = {r1, r2, r3, r4};

	r1 = {cos(phi), 0, sin(phi), 0};
	r2 = {0, 1, 0, 0};
	r3 = {-1*sin(phi), 0, cos(phi), 0};
	r4 = {0, 0, 0, 1};
	vector<vector<float>> ry = {r1, r2, r3, r4};

	r1 = {cos(psi), -1*sin(psi), 0, 0};
	r2 = {sin(psi), cos(psi), 0, 0};
	r3 = {0, 0, 1, 0};
	r4 = {0, 0, 0, 1};
	vector<vector<float>> rz = {r1, r2, r3, r4};

	vector<vector<float>> rxyz = matmul(rx, ry);
	rxyz = matmul(rxyz, rz);
	
	vector<vector<float>> t = {
		{0, 0, 0, x},
		{0, 0, 0, y},
		{0, 0, 0, z},
		{0, 0, 0, 0}};
	vector<vector<float>> tm(t.size(), vector<float>(rxyz.size()));
	for (int i=0; i<t.size(); i++) {
		for (int j=0; j<t.size(); j++) {
			tm[i][j] = t[i][j] + rxyz[i][j];
		}
	}

	vector<vector<float>> temp = {
		{cos(pi/2), 0, sin(pi/2), -1*L/2},
		{0, 1, 0, 0},
		{-1*sin(pi/2), 0, cos(pi/2), -1*W/2},
		{0, 0, 0, 1}};
	vector<vector<float>> rb = matmul(tm, temp);

	temp = {
		{cos(pi/2), 0, sin(pi/2), L/2},
		{0, 1, 0, 0},
		{-1*sin(pi/2), 0, cos(pi/2), -1*W/2},
		{0, 0, 0, 1}};
	vector<vector<float>> rf = matmul(tm, temp);

	temp = {
		{cos(pi/2), 0, sin(pi/2), L/2},
		{0, 1, 0, 0},
		{-1*sin(pi/2), 0, cos(pi/2), W/2},
		{0, 0, 0, 1}};
	vector<vector<float>> lf = matmul(tm, temp);

	temp = {
		{cos(pi/2), 0, sin(pi/2), -1*L/2},
		{0, 1, 0, 0},
		{-1*sin(pi/2), 0, cos(pi/2), W/2},
		{0, 0, 0, 1}};
	vector<vector<float>> lb = matmul(tm, temp);

	TransformMatrices tfm = {rb, rf, lb, lf, tm};
	return tfm;
}