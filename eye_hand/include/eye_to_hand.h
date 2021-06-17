
#pragma once

#include <opencv2\opencv.hpp>
#include <vector>
#include "quaternion.h"

using namespace cv;





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