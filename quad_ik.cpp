// The inverse kinematics for a quadruped robot
// This is a translation of the python functions originally obtained from
// https://spotmicroai.readthedocs.io/en/latest/kinematic/
// Translation by Taylor Privat

#include "quad_ik.h"


JointAngles legIK(float x, float y, float z, const LinkLens links) {
	// Calculates Joint Angles given a target foot position {IN LEG SPACE}
	// (x, y, z) -> coordinates of target position for EE
	// f -> len of shoulder-point to target point on x/y
	// g -> len we need to read to point on x/y
	// h -> 3D len we need to reach

	JointAngles ja;

	float f = sqrt(x*x + y*y - links.l1*links.l1);
	float g = f - links.l2;
	float h = sqrt(g*g + z*z);

	ja.ang1 = -1*atan2(y, x) - atan2(f, -1*links.l1);

	float d = (h*h - links.l3*links.l3 - links.l4*links.l4);
	d /= (2 * links.l3 * links.l4);

	ja.ang3 = acos(d);

	ja.ang2 = atan2(z, g) - atan2(links.l4*sin(ja.ang3), links.l3+links.l4*cos(ja.ang3));

	return ja;
}

Matrix<float, 5, 4> calcLegPoints(JointAngles angles, LinkLens links) {
	// Calculates Leg points of each joint {IN LEG SPACE}
	float ang23 = angles.ang2 + angles.ang3;
	Eigen::Matrix<float, 5, 4> result;

	Eigen::Vector4f t0;
	t0 << 0, 0, 0, 1;
	result.row(0) = t0;

	Eigen::Vector4f temp;
	temp << -1*links.l1*cos(angles.ang1), links.l1*sin(angles.ang1), 0, 0;
	result.row(1) = t0 + temp;

	temp[0] = -1*links.l2*sin(angles.ang1);
	temp[1] = -1*links.l2*cos(angles.ang1);
	result.row(2) = result.row(1) + temp.transpose();

	temp[0] = -1*links.l3*sin(angles.ang1)*cos(angles.ang2);
	temp[1] = -1*links.l3*cos(angles.ang1)*cos(angles.ang2);
	temp[2] = links.l3*sin(angles.ang2);
	result.row(3) = result.row(2) + temp.transpose();

	temp[0] = -1*links.l4*sin(angles.ang1)*cos(ang23);
	temp[1] = -1*links.l4*cos(angles.ang1)*cos(ang23);
	temp[2] = links.l4*sin(ang23);
	result.row(4) = result.row(3) + temp.transpose();

	return result;
}

TransformMatrices bodyIK(float omega, float phi, float psi,
                         float x, float y, float z,
                         float L, float W) {
	// Calculates position of body (4 shoulders) {IN BODY SPACE}
	Matrix4f rx;
	rx << 1, 0, 0, 0,
		0, cos(omega), -1*sin(omega), 0,
		0, sin(omega), cos(omega), 0,
		0, 0, 0, 1;
	Matrix4f ry;
	ry << cos(phi), 0, sin(phi), 0,
		0, 1, 0, 0,
		-1*sin(phi), 0, cos(phi), 0,
		0, 0, 0, 1;
	Matrix4f rz;
	rz << cos(psi), -1*sin(psi), 0, 0,
		sin(psi), cos(psi), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	Matrix4f rxyz = rx * ry * rz;

	Matrix4f t;
	t << 0, 0, 0, x,
		0, 0, 0, y,
		0, 0, 0, z,
		0, 0, 0, 0;
	Matrix4f tm = t + rxyz;
	
	Matrix4f temp;
	temp << cos(PI/2), 0, sin(PI/2), -1*L/2,
		0, 1, 0, 0,
		-1*sin(PI/2), 0, cos(PI/2), -1*W/2,
		0, 0, 0, 1;
	Matrix4f rb = tm * temp;

	temp << cos(PI/2), 0, sin(PI/2), L/2,
		0, 1, 0, 0,
		-1*sin(PI/2), 0, cos(PI/2), -1*W/2,
		0, 0, 0, 1;
	Matrix4f rf = tm * temp;

	temp << cos(PI/2), 0, sin(PI/2), L/2,
		0, 1, 0, 0,
		-1*sin(PI/2), 0, cos(PI/2), W/2,
		0, 0, 0, 1;
	Matrix4f lf = tm * temp;

	temp << cos(PI/2), 0, sin(PI/2), -1*L/2,
		0, 1, 0, 0,
		-1*sin(PI/2), 0, cos(PI/2), W/2,
		0, 0, 0, 1;
	Matrix4f lb = tm * temp;
	
	TransformMatrices tfm = {rb, rf, lb, lf, tm};
	return tfm;
}
