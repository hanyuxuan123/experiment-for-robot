/*!
 * \file handeye.h
 * Hand-Eye Calibration by Different Methods.
 *
 * \author NUDT_WG
 * \date March 2016
 *
 * 
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "quaternion.h"

using namespace cv;
using namespace std;

/**
 * Compute the skew symmetric matrix.
 * as defined  [e]x  =    [ 0,-e3,e2;
 *                          e3,0,-e1;
 *                          -e2,e1,0 ]
 *
 * @Returns  cv::Mat 3x3 matrix
 * @param A [in] 3x1 vector
 */
Mat skew(Mat A)
{
	CV_Assert(A.cols == 1 && A.rows == 3);
	Mat B(3, 3, CV_64FC1);

	B.at<double>(0, 0) = 0.0;
	B.at<double>(0, 1) = -A.at<double>(2, 0);
	B.at<double>(0, 2) = A.at<double>(1, 0);

	B.at<double>(1, 0) = A.at<double>(2, 0);
	B.at<double>(1, 1) = 0.0;
	B.at<double>(1, 2) = -A.at<double>(0, 0);

	B.at<double>(2, 0) = -A.at<double>(1, 0);
	B.at<double>(2, 1) = A.at<double>(0, 0);
	B.at<double>(2, 2) = 0.0;

	return B;
}


/**
 * Creates a dual quaternion from a rotation matrix and a translation vector.
 *
 * @Returns  void
 * @param q [out] q
 * @param qprime [out] q'
 * @param R [in] Rotation
 * @param t [in] Translation
 */
void getDualQ(Mat q, Mat qprime, Mat R, Mat t)
{
	Mat r(3, 1, CV_64FC1);
	Mat l(3, 1, CV_64FC1);
	double theta;
	Mat tempd(1, 1, CV_64FC1);
	double d;
	Mat c(3, 1, CV_64FC1);
	Mat m(3, 1, CV_64FC1);
	Mat templ(3, 1, CV_64FC1);
	Mat tempqt(1, 1, CV_64FC1);
	double qt;
	Mat tempml(3, 1, CV_64FC1);

	Rodrigues(R, r);
	theta = norm(r);
	l = r / theta;
	tempd = l.t()*t;
	d = tempd.at<double>(0, 0);

	c = 0.5*(t - d*l) + cos(theta / 2) / sin(theta / 2)*l.cross(t);
	m = c.cross(l);

	q.at<double>(0, 0) = cos(theta / 2);
	templ = sin(theta / 2)*l;
	templ.copyTo(q(Rect(0, 1, 1, 3)));

	tempqt = -0.5*templ.t()*t;
	qt = tempqt.at<double>(0, 0);
	tempml = 0.5*(q.at<double>(0, 0)*t + t.cross(templ));

	qprime.at<double>(0, 0) = qt;
	tempml.copyTo(qprime(Rect(0, 1, 1, 3)));

}

/**
 * Compute the Kronecker tensor product of matrix A and B.
 *
 * @Returns  cv::Mat (MP)x(NQ) matrix
 * @param A [in] MxN matrix
 * @param B [in] PxQ matrix
 */
Mat kron(Mat A, Mat B)
{
	Mat C(A.rows*B.rows, A.cols*B.cols, CV_64FC1, Scalar(0));

	for (int i = 0; i < A.rows; i++)
		for (int j = 0; j < A.cols; j++)
			C(Rect(B.cols * j, B.rows * i, B.cols, B.rows)) = A.at<double>(i, j)*B;

	return C;
}

/**
 * Signum function.
 * For each element of X, SIGN(X) returns 1 if the element is greater than zero, 
 * return 0 if it equals zero and -1 if it is less than zero.
 *
 * @Returns  double
 * @param a [in] 
 */
double sign(double a)
{
	if (a > 0)
		return 1;
	else if (a < 0)
		return -1;
	else
		return 0;
}

/**
 * Hand/Eye calibration using Tsai' method.
 * Read the paper "A New Technique for Fully Autonomous and Efficient 3D Robotics 
 * Hand-Eye Calibration, Tsai, 1989" for further details.
 *
 * Solving AX=XB
 * @Returns  void
 * @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
 * @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
 * @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
 */
void Tsai_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	Mat Rgij(3, 3, CV_64FC1);
	Mat Rcij(3, 3, CV_64FC1);

	Mat rgij(3, 1, CV_64FC1);
	Mat rcij(3, 1, CV_64FC1);

	double theta_gij;
	double theta_cij;

	Mat rngij(3, 1, CV_64FC1);
	Mat rncij(3, 1, CV_64FC1);

	Mat Pgij(3, 1, CV_64FC1);
	Mat Pcij(3, 1, CV_64FC1);

	Mat tempA(3, 3, CV_64FC1);
	Mat tempb(3, 1, CV_64FC1);

	Mat A;
	Mat b;
	Mat pinA;

	Mat Pcg_prime(3, 1, CV_64FC1);
	Mat Pcg(3, 1, CV_64FC1);
	Mat PcgTrs(1, 3, CV_64FC1);

	Mat Rcg(3, 3, CV_64FC1);
	Mat eyeM = Mat::eye(3, 3, CV_64FC1);
		
	Mat Tgij(3, 1, CV_64FC1);
	Mat Tcij(3, 1, CV_64FC1);

	Mat tempAA(3, 3, CV_64FC1);
	Mat tempbb(3, 1, CV_64FC1);

	Mat AA;
	Mat bb;
	Mat pinAA;

	Mat Tcg(3, 1, CV_64FC1);

	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);

		Rodrigues(Rgij, rgij);
		Rodrigues(Rcij, rcij);

		theta_gij = norm(rgij);
		theta_cij = norm(rcij);

		rngij = rgij / theta_gij;
		rncij = rcij / theta_cij;

		Pgij = 2 * sin(theta_gij / 2)*rngij;
		Pcij = 2 * sin(theta_cij / 2)*rncij;

		tempA = skew(Pgij + Pcij);
		tempb = Pcij - Pgij;

		A.push_back(tempA);
		b.push_back(tempb);
	}

	//Compute rotation
	invert(A, pinA, DECOMP_SVD);

	Pcg_prime = pinA * b;
	Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
	PcgTrs = Pcg.t();	
	Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg)*norm(Pcg))*skew(Pcg));

	//Computer Translation 
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		tempAA = Rgij - eyeM;
		tempbb = Rcg * Tcij - Tgij;

		AA.push_back(tempAA);
		bb.push_back(tempbb);
	}

	invert(AA, pinAA, DECOMP_SVD);
	Tcg = pinAA * bb;

	Rcg.copyTo(Hcg(Rect(0, 0, 3, 3)));
	Tcg.copyTo(Hcg(Rect(3, 0, 1, 3)));
	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}
/**
 * Hand/Eye calibration using Park' method(NAVY).
 * Read the paper "Robot Sensor Calibration: Solving AX = XB on the Euclidean Group, 1994" for further details.
 *
 * @Returns  void
 * @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
 * @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
 * @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
 */
void Navy_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	Mat Rgij(3, 3, CV_64FC1);
	Mat Rcij(3, 3, CV_64FC1);

	Mat alpha1(3, 1, CV_64FC1);
	Mat beta1(3, 1, CV_64FC1);
	Mat alpha2(3, 1, CV_64FC1);
	Mat beta2(3, 1, CV_64FC1);
	Mat A(3, 3, CV_64FC1);
	Mat B(3, 3, CV_64FC1);

	Mat alpha(3, 1, CV_64FC1);
	Mat beta(3, 1, CV_64FC1);
	Mat M(3, 3, CV_64FC1, Scalar(0));

	Mat MtM(3, 3, CV_64FC1);
	Mat veMtM(3, 3, CV_64FC1);
	Mat vaMtM(3, 1, CV_64FC1);
	Mat pvaM(3, 3, CV_64FC1, Scalar(0));

	Mat Rx(3, 3, CV_64FC1);

	Mat Tgij(3, 1, CV_64FC1);
	Mat Tcij(3, 1, CV_64FC1);

	Mat eyeM = Mat::eye(3, 3, CV_64FC1);

	Mat tempCC(3, 3, CV_64FC1);
	Mat tempdd(3, 1, CV_64FC1);

	Mat C;
	Mat d;
	Mat Tx(3, 1, CV_64FC1);

	//Compute rotation
	if (Hgij.size() == 2) // Two (Ai,Bi) pairs
	{
		Rodrigues(Hgij[0](Rect(0, 0, 3, 3)), alpha1);
		Rodrigues(Hgij[1](Rect(0, 0, 3, 3)), alpha2);
		Rodrigues(Hcij[0](Rect(0, 0, 3, 3)), beta1);
		Rodrigues(Hcij[1](Rect(0, 0, 3, 3)), beta2);
		
		alpha1.copyTo(A.col(0));
		alpha2.copyTo(A.col(1));
		(alpha1.cross(alpha2)).copyTo(A.col(2));

		beta1.copyTo(B.col(0));
		beta2.copyTo(B.col(1));
		(beta1.cross(beta2)).copyTo(B.col(2));

		Rx = A*B.inv();

	}
	else // More than two (Ai,Bi) pairs
	{
		for (int i = 0; i < nStatus; i++)
		{
			Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
			Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);

			Rodrigues(Rgij, alpha);
			Rodrigues(Rcij, beta);
			
			M = M + beta*alpha.t();
		}

		MtM = M.t()*M;
		eigen(MtM, vaMtM, veMtM);

		pvaM.at<double>(0, 0) = 1 / sqrt(vaMtM.at<double>(0, 0));
		pvaM.at<double>(1, 1) = 1 / sqrt(vaMtM.at<double>(1, 0));
		pvaM.at<double>(2, 2) = 1 / sqrt(vaMtM.at<double>(2, 0));

		Rx = veMtM*pvaM*veMtM.inv()*M.t();
	}

	//Computer Translation 
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		tempCC = eyeM - Rgij;
		tempdd = Tgij - Rx * Tcij;

		C.push_back(tempCC);
		d.push_back(tempdd);
	}

	Tx = (C.t()*C).inv()*(C.t()*d);

	Rx.copyTo(Hcg(Rect(0, 0, 3, 3)));
	Tx.copyTo(Hcg(Rect(3, 0, 1, 3)));
	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}
/**
* Hand/Eye calibration using Horaud' method(INRIA).
* Read the paper "Hand-Eye Calibration, Radu Horaud and Fadi Dornaika, 1995" for further details.
*
* @Returns  void
* @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
* @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
* @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
*/
void Inria_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	Mat Rgij(3, 3, CV_64FC1);
	Mat Rcij(3, 3, CV_64FC1);
	Mat Tgij(3, 1, CV_64FC1);
	Mat Tcij(3, 1, CV_64FC1);

	Mat Qgij(4, 1, CV_64FC1);
	Mat Qcij(4, 1, CV_64FC1);
	Mat Lqij(4, 4, CV_64FC1);
	Mat Rqij(4, 4, CV_64FC1);

	Mat tempA(4, 4, CV_64FC1);
	Mat A;
	Mat w, u, vt, v;
	Mat qoff(4, 1, CV_64FC1);
	Mat Roff(3, 3, CV_64FC1);

	Mat eyeM = Mat::eye(3, 3, CV_64FC1);
	Mat tempAA(3, 3, CV_64FC1);
	Mat tempbb(3, 1, CV_64FC1);
	Mat AA;
	Mat bb;
	Mat pinAA;

	Mat Toff(3, 1, CV_64FC1);

	// Compute rotation
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		Qgij = dcm2q(Rgij);
		Qcij = dcm2q(Rcij);
		
		// qA*qX=qX*qB<=>(RqA-LqB)*qx=0
		Lqij = qskewL(Qgij);
		Rqij = qskewR(Qcij);

		tempA = Lqij - Rqij;
		A.push_back(tempA);
	}

	SVD::compute(A, w, u, vt, SVD::FULL_UV);
	v = vt.t();
	v(Rect(3, 0, 1, 4)).copyTo(qoff);
	Roff = q2dcm(qoff);

	// Computer Translation 
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		tempAA = Rgij - eyeM;
		tempbb = Roff * Tcij - Tgij;

		AA.push_back(tempAA);
		bb.push_back(tempbb);
	}

	invert(AA, pinAA, DECOMP_SVD);
	Toff = pinAA * bb;

	Roff.copyTo(Hcg(Rect(0, 0, 3, 3)));
	Toff.copyTo(Hcg(Rect(3, 0, 1, 3)));

	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}

/**
 * Hand/Eye calibration using Daniilidis' method.
 * Read the paper "Hand-Eye Calibration Using Dual Quaternions, Konstantinos Daniilidis, 1999" for further details.
 *
 * @Returns  void
 * @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
 * @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
 * @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
 */
void DualQ_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	Mat Rgij(3, 3, CV_64FC1);
	Mat Rcij(3, 3, CV_64FC1);
	Mat Tgij(3, 1, CV_64FC1);
	Mat Tcij(3, 1, CV_64FC1);

	Mat Qa(4, 1, CV_64FC1);
	Mat Qaprime(4, 1, CV_64FC1);
	Mat Qb(4, 1, CV_64FC1);
	Mat Qbprime(4, 1, CV_64FC1);

	Mat a(3, 1, CV_64FC1);
	Mat aprime(3, 1, CV_64FC1);
	Mat b(3, 1, CV_64FC1);
	Mat bprime(3, 1, CV_64FC1);

	Mat a_b(3, 1, CV_64FC1);
	Mat ap_bp(3, 1, CV_64FC1);
	Mat axb(3, 3, CV_64FC1);
	Mat apxbp(3, 3, CV_64FC1);

	Mat zero1 = Mat::zeros(3, 1, CV_64FC1);
	Mat zero3 = Mat::zeros(3, 3, CV_64FC1);

	Mat S(6, 8, CV_64FC1);
	Mat T;

	Mat w, u, vt, v;

	Mat v7, v8;
	Mat u1, v1, u2, v2;

	Mat coeffs(3, 1, CV_64FC1);
	Mat s;

	double val1, val2, fs, val;
	double lamda1, lamda2;

	Mat Qresult, Q, Qprime;
	Mat Rresult(3, 3, CV_64FC1);
	Mat Tresult(3, 1, CV_64FC1);

	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		getDualQ(Qa, Qaprime, Rgij, Tgij);
		getDualQ(Qb, Qbprime, Rcij, Tcij);

		Qa(Rect(0, 1, 1, 3)).copyTo(a);
		Qaprime(Rect(0, 1, 1, 3)).copyTo(aprime);
		Qb(Rect(0, 1, 1, 3)).copyTo(b);
		Qbprime(Rect(0, 1, 1, 3)).copyTo(bprime);

		a_b = a - b;
		ap_bp = aprime - bprime;

		axb = skew(a + b);
		apxbp = skew(aprime + bprime);

		a_b.copyTo(S(Rect(0, 0, 1, 3)));
		axb.copyTo(S(Rect(1, 0, 3, 3)));
		zero1.copyTo(S(Rect(4, 0, 1, 3)));
		zero3.copyTo(S(Rect(5, 0, 3, 3)));

		ap_bp.copyTo(S(Rect(0, 3, 1, 3)));
		apxbp.copyTo(S(Rect(1, 3, 3, 3)));
		a_b.copyTo(S(Rect(4, 3, 1, 3)));
		axb.copyTo(S(Rect(5, 3, 3, 3)));

		T.push_back(S);
	}

	SVD::compute(T, w, u, vt);
	v = vt.t();

	v(Rect(6, 0, 1, 8)).copyTo(v7);
	v(Rect(7, 0, 1, 8)).copyTo(v8);

	v7(Rect(0, 0, 1, 4)).copyTo(u1);
	v7(Rect(0, 4, 1, 4)).copyTo(v1);
	v8(Rect(0, 0, 1, 4)).copyTo(u2);
	v8(Rect(0, 4, 1, 4)).copyTo(v2);

	coeffs.at<double>(0, 0) = u1.dot(v1);
	coeffs.at<double>(1, 0) = u1.dot(v2) + u2.dot(v1);
	coeffs.at<double>(2, 0) = u2.dot(v2);

	solvePoly(coeffs, s);

	val1 = s.at<double>(0, 0)*s.at<double>(0, 0)*u1.dot(u1) + 2 * s.at<double>(0, 0)*u1.dot(u2) + u2.dot(u2);
	val2 = s.at<double>(1, 0)*s.at<double>(1, 0)*u1.dot(u1) + 2 * s.at<double>(1, 0)*u1.dot(u2) + u2.dot(u2);

	if (val1 > val2)
	{
		fs = s.at<double>(0, 0);
		val = val1;

	}
	else
	{
		fs = s.at<double>(1, 0);
		val = val2;
	}

	lamda2 = sqrt(1 / val);
	lamda1 = fs*lamda2;

	Qresult = lamda1*v7 + lamda2*v8;
	Qresult(Rect(0, 0, 1, 4)).copyTo(Q);
	Qresult(Rect(0, 4, 1, 4)).copyTo(Qprime);

	Rresult = q2dcm(Q);
	Tresult = 2 * qmult(Qprime, qconj(Q))(Rect(0, 1, 1, 3));
	
	Rresult.copyTo(Hcg(Rect(0, 0, 3, 3)));
	Tresult.copyTo(Hcg(Rect(3, 0, 1, 3)));
	
	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}

/**
 * Hand/Eye calibration using Andreff' method.
 * Read the paper "Robot Hand-Eye Calibration Using Structure-from-Motion, Andreff N, Horaud R, 2001" for further details.
 *
 * @Returns  void
 * @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
 * @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
 * @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
 */
void Kron_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	Mat Rgij(3, 3, CV_64FC1);
	Mat Rcij(3, 3, CV_64FC1);
	Mat eyeM9 = Mat::eye(9, 9, CV_64FC1);

	Mat tempLi(9, 9, CV_64FC1);
	Mat Li;

	Mat w, u, vt, v;
	Mat vi(9, 1, CV_64FC1);
	Mat Vi(3, 3, CV_64FC1);
	Mat Rcg(3, 3, CV_64FC1);

	Mat tempCC(3, 3, CV_64FC1);
	Mat tempdd(3, 1, CV_64FC1);

	Mat Tgij(3, 1, CV_64FC1);
	Mat Tcij(3, 1, CV_64FC1);

	Mat eyeM3 = Mat::eye(3, 3, CV_64FC1);
	Mat C;
	Mat d;
	Mat Tcg(3, 1, CV_64FC1);

	// Compute rotation
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);

		tempLi = eyeM9 - kron(Rgij, Rcij);
		Li.push_back(tempLi);
	}

	SVD::compute(Li, w, u, vt, SVD::FULL_UV);
	v = vt.t();
	v(Rect(8, 0, 1, 9)).copyTo(vi);
	Vi = vi.reshape(0, 3);
	Rcg = sign(determinant(Vi))*pow(abs(determinant(Vi)), -1.0 / 3)*Vi; // -1.0/3 NOT -1/3

	//Computer Translation 
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		tempCC = eyeM3 - Rgij;
		tempdd = Tgij - Rcg * Tcij;

		C.push_back(tempCC);
		d.push_back(tempdd);
	}

	Tcg = (C.t()*C).inv()*(C.t()*d);

	Rcg.copyTo(Hcg(Rect(0, 0, 3, 3)));
	Tcg.copyTo(Hcg(Rect(3, 0, 1, 3)));
	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}