/*!
 * \file quaternion.h
 * Operations of Quaternion.
 *
 * \author NUDT_WG
 * \date March 2016
 *
 * 
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;

/**
 * Normalizes quaternions.
 *
 * @Returns  cv::Mat 4x1 vector:Output quaternion.
 * @param qin [in] 4x1 vector:Input quaternion.
 */
Mat qnorm(Mat qin)
{
	CV_Assert(qin.rows == 4 && qin.cols == 1);
	Mat qout(4, 1, CV_64FC1);
	double normq;
	normq = norm(qin);
	qout = qin / normq;
	return qout;
}

/**
* Compute the left skew symmetric matrix of the quaternion.
* q1*q2 = [q1]x*q2.
*
* @Returns  cv::Mat 4x4 matrix:left skew symmetric matrix.
* @param q [in] 4x1 vector:quaternion.
*/
Mat qskewL(Mat q)
{
	CV_Assert(q.rows == 4 && q.cols == 1);
	Mat qL(4, 4, CV_64FC1);

	qL.at<double>(0, 0) = q.at<double>(0, 0);
	qL.at<double>(0, 1) = -q.at<double>(1, 0);
	qL.at<double>(0, 2) = -q.at<double>(2, 0);
	qL.at<double>(0, 3) = -q.at<double>(3, 0);

	qL.at<double>(1, 0) = q.at<double>(1, 0);
	qL.at<double>(1, 1) = q.at<double>(0, 0);
	qL.at<double>(1, 2) = -q.at<double>(3, 0);
	qL.at<double>(1, 3) = q.at<double>(2, 0);

	qL.at<double>(2, 0) = q.at<double>(2, 0);
	qL.at<double>(2, 1) = q.at<double>(3, 0);
	qL.at<double>(2, 2) = q.at<double>(0, 0);
	qL.at<double>(2, 3) = -q.at<double>(1, 0);

	qL.at<double>(3, 0) = q.at<double>(3, 0);
	qL.at<double>(3, 1) = -q.at<double>(2, 0);
	qL.at<double>(3, 2) = q.at<double>(1, 0);
	qL.at<double>(3, 3) = q.at<double>(0, 0);

	return qL;
}

/**
* Compute the right skew symmetric matrix of the quaternion.
* q1*q2 = [q2]x*q1.
*
* @Returns  cv::Mat 4x4 matrix:right skew symmetric matrix.
* @param q [in] 4x1 vector:quaternion.
*/
Mat qskewR(Mat q)
{
	CV_Assert(q.rows == 4 && q.cols == 1);
	Mat qR(4, 4, CV_64FC1);

	qR.at<double>(0, 0) = q.at<double>(0, 0);
	qR.at<double>(0, 1) = -q.at<double>(1, 0);
	qR.at<double>(0, 2) = -q.at<double>(2, 0);
	qR.at<double>(0, 3) = -q.at<double>(3, 0);

	qR.at<double>(1, 0) = q.at<double>(1, 0);
	qR.at<double>(1, 1) = q.at<double>(0, 0);
	qR.at<double>(1, 2) = q.at<double>(3, 0);
	qR.at<double>(1, 3) = -q.at<double>(2, 0);

	qR.at<double>(2, 0) = q.at<double>(2, 0);
	qR.at<double>(2, 1) = -q.at<double>(3, 0);
	qR.at<double>(2, 2) = q.at<double>(0, 0);
	qR.at<double>(2, 3) = q.at<double>(1, 0);

	qR.at<double>(3, 0) = q.at<double>(3, 0);
	qR.at<double>(3, 1) = q.at<double>(2, 0);
	qR.at<double>(3, 2) = -q.at<double>(1, 0);
	qR.at<double>(3, 3) = q.at<double>(0, 0);

	return qR;
}


/**
 * Calculates the product of two quaternions q1 and q2.
 *
 * @Returns  cv::Mat 4x1 vector:Output quaternion.
 * @param q1 [in] 4x1 vector:First input quaternion.
 * @param q2 [in] 4x1 vector:Second input quaternion.
 */
Mat qmult(Mat q1, Mat q2)
{
	CV_Assert(q1.rows == 4 && q1.cols == 1 && q2.rows == 4 && q2.cols == 1);
	Mat qout(4, 1, CV_64FC1);
	double q1w, q2w;
	Mat q1xyz(3, 1, CV_64FC1), q2xyz(3, 1, CV_64FC1);

	q1w = q1.at<double>(0, 0);
	q2w = q2.at<double>(0, 0);

	q1(Rect(0, 1, 1, 3)).copyTo(q1xyz);
	q2(Rect(0, 1, 1, 3)).copyTo(q2xyz);

	qout.at<double>(0, 0) = q1w*q2w - q1xyz.dot(q2xyz);
	qout(Rect(0, 1, 1, 3)) = q1w*q2xyz + q2w*q1xyz + q1xyz.cross(q2xyz);

	//qout = qskewL(q1)*q2;
	//qout = qskewR(q2)*q1;

	return qout;

}


/**
 * Calculates the conjugate of the quaternion q.
 *
 * @Returns  cv::Mat 4x1 vector:Output quaternion.
 * @param qin [in] 4x1 vector:Input quaternion.
 */
Mat qconj(Mat qin)
{
	CV_Assert(qin.rows == 4 && qin.cols == 1);
	Mat qout(4, 1, CV_64FC1);
	qout.at<double>(0, 0) = qin.at<double>(0, 0);
	qout.at<double>(1, 0) = - qin.at<double>(1, 0);
	qout.at<double>(2, 0) = - qin.at<double>(2, 0);
	qout.at<double>(3, 0) = - qin.at<double>(3, 0);

	return qout;
}


/**
 * Converts quaternions into direction cosine matrices.
 *
 * @Returns  cv::Mat 3x3 matrix:rotation.
 * @param q [in] 4x1 vector:quaternion.
 */
Mat q2dcm(Mat q)
{
	CV_Assert(q.rows == 4 && q.cols == 1);
	Mat R(3, 3, CV_64FC1);
	Mat normq(4, 1, CV_64FC1);
	normq = qnorm(q);

	double w, x, y, z;
	w = normq.at<double>(0, 0);
	x = normq.at<double>(1, 0);
	y = normq.at<double>(2, 0);
	z = normq.at<double>(3, 0);

	R.at<double>(0, 0) = 1 - 2 * y*y - 2 * z*z;
	R.at<double>(0, 1) = 2 * x*y - 2 * w*z;
	R.at<double>(0, 2) = 2 * x*z + 2 * w*y;

	R.at<double>(1, 0) = 2 * x*y + 2 * w*z;
	R.at<double>(1, 1) = 1 - 2 * x*x - 2 * z*z;
	R.at<double>(1, 2) = 2 * y*z - 2 * w*x;

	R.at<double>(2, 0) = 2 * x*z - 2 * w*y;
	R.at<double>(2, 1) = 2 * y*z + 2 * w*x;
	R.at<double>(2, 2) = 1 - 2 * x*x - 2 * y*y;

	return R;
}


/**
 * Converts direction cosine matrices into quaternions.
 *
 * @Returns  cv::Mat 4x1 vector:quaternion.
 * @param R [in] 3x3 matrix:rotation.
 */
Mat dcm2q(Mat R)
{
	CV_Assert(R.rows == 3 && R.cols == 3);
	Mat q(4, 1, CV_64FC1);
	q.at<double>(0, 0) = 0.5*sqrt(1 + R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2));
	q.at<double>(1, 0) = 0.25*(R.at<double>(1, 2) - R.at<double>(2, 1)) / q.at<double>(0, 0);
	q.at<double>(2, 0) = 0.25*(R.at<double>(2, 0) - R.at<double>(0, 2)) / q.at<double>(0, 0);
	q.at<double>(3, 0) = 0.25*(R.at<double>(0, 1) - R.at<double>(1, 0)) / q.at<double>(0, 0);

	return q;

}


