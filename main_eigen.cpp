#include "quad_ik.h"
#include <fstream>
#include <Eigen/LU>

int main() {
	LinkLens links = {25, 20, 80, 80};
	JointAngles ja = legIK(-55.0, -100.0, 20.0, links);
	Eigen::Matrix<float, 5, 4> lp1 = calcLegPoints(ja, links);
	cout << lp1 << '\n';

	// The main program for this library starts here.
	// Given some body position 	Om	Ph	  Ps   X  Y  Z  Len  Width
	TransformMatrices tfm = bodyIK(0.3, 0.1, -0.4, 0, 0, 0, 120, 90);
	cout << "tfm worked\n";

	// Now we get the 4 shoulder positions
	Vector4f fp;
	fp << 0,0,0,1;

	MatrixXf cp0 = tfm.lf * fp;
	MatrixXf cp1 = tfm.rf * fp;
	MatrixXf cp2 = tfm.lb * fp;
	MatrixXf cp3 = tfm.rb * fp;

	fstream fin;
	fin.open("robot.dat", ios::out);
	for (int i=0; i<3; i++) {
		fin << cp0(i) << '\t';
	}
	fin << '\n';
	for (int i=0; i<3; i++) {
		fin << cp1(i) << '\t';
	}
	fin << '\n';
	for (int i=0; i<3; i++) {
		fin << cp3(i) << '\t';
	}
	fin << '\n';
	for (int i=0; i<3; i++) {
		fin << cp2(i) << '\t';
	}
	fin << "\n\n";
	
	// Now we get the leg joint angles
	// and leg positions
	Matrix4f lp; // Leg positions in world space
	lp << 100, -100, 100, 1,
		100, -100, -100, 1,
		-100, -100, 100, 1,
		-100, -100, -100, 1;

	// Invert local x
	Matrix4f Ix;
	Ix << -1, 0, 0, 0,
		   0, 1, 0, 0,
		   0, 0, 1, 0,
		   0, 0, 0, 1;

	fin << '\n';
	MatrixXf q = tfm.lf.inverse() * lp.row(0).transpose();
	Matrix<float, 5, 4> lfja = calcLegPoints(legIK(q(0), q(1), q(2), links), links);
	Matrix<float, 5, 4> points;
	for (int i=0; i<5; i++) {
		points.row(i) = tfm.lf * lfja.row(i).transpose();
	}
	for (int i=0; i<5; i++) {
		for (int j=0; j<3; j++) {
			fin << points(i, j) << '\t';
		}
		fin << '\n';
	}
	fin << "\n\n";

	q = tfm.lb.inverse() * lp.row(2).transpose();
	Matrix<float, 5, 4> lbja = calcLegPoints(legIK(q(0), q(1), q(2), links), links);
	for (int i=0; i<5; i++) {
		points.row(i) = tfm.lb * lbja.row(i).transpose();
	}
	for (int i=0; i<5; i++) {
		for (int j=0; j<3; j++) {
			fin << points(i, j) << '\t';
		}
		fin << '\n';
	}
	fin << "\n\n";

	q = Ix * tfm.rf.inverse() * lp.row(1).transpose();
	Matrix<float, 5, 4> rfja = calcLegPoints(legIK(q(0), q(1), q(2), links), links);
	for (int i=0; i<5; i++) {
		points.row(i) = tfm.rf * Ix * rfja.row(i).transpose();
	}
	for (int i=0; i<5; i++) {
		for (int j=0; j<3; j++) {
			fin << points(i, j) << '\t';
		}
		fin << '\n';
	}
	fin << "\n\n";

	q = Ix * tfm.rb.inverse() * lp.row(3).transpose();
	Matrix<float, 5, 4> rbja = calcLegPoints(legIK(q(0), q(1), q(2), links), links);
	for (int i=0; i<5; i++) {
		points.row(i) = tfm.rb * Ix * rbja.row(i).transpose();
	}
	for (int i=0; i<5; i++) {
		for (int j=0; j<3; j++) {
			fin << points(i, j) << '\t';
		}
		fin << '\n';
	}
	fin << "\n\n";
	return 0;
}
