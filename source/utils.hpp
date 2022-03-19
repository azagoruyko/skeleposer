#pragma once

#include <chrono>
#include <iostream>

#include <maya/MVector.h> 
#include <maya/MMatrix.h> 
#include <maya/MGlobal.h> 

#define MSTR(v) MString(to_string(v).c_str())
#define EPSILON 1e-6
#define RAD2DEG 57.2958
#define DEG2RAD 0.0174532862

inline double rbf_gaussian(double r, double c = 1) { return exp(-pow(r * c, 2)); }
inline double rbf_multiquadric(double r, double c = 1) { return sqrt(1 + pow(r*c, 2)); }
inline double rbf_inverse_quadric(double r, double c = 1) { return 1.0 / (1 + pow(r*c, 2)); }
inline double rbf_inverse_multiquadric(double r, double c = 1) { return 1.0 / sqrt(1 + pow(r*c, 2)); }
inline double rbf_polyharmonic_spline(double r, double c = 1) { return pow(r, 2 * c - 1); }
inline double rbf_sigmoid(double r, double c = 1) { return 1.0 / (1 + exp(-r*c)); }

inline chrono::steady_clock::time_point getMeasureTime() { return chrono::steady_clock::now(); }

template <typename... Args>
inline string formatString(const char* format, Args... args)
{
	int nsize = snprintf(NULL, 0, format, args...) + 1;
	char* buffer = new char[nsize];
	snprintf(buffer, nsize, format, args...);
	string s(buffer);
	delete[] buffer;
	return s;
}

inline void measureTime(const string& name, const chrono::steady_clock::time_point& startTime)
{
	const auto count = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - startTime).count();
	MGlobal::displayInfo(MString(formatString("%s: %.2fms", name.c_str(), count / 1000.0).c_str()));
}

inline MVector maxis(const MMatrix& mat, unsigned int index) { return MVector(mat[index][0], mat[index][1], mat[index][2]); }
inline MVector xaxis(const MMatrix& mat) { return maxis(mat, 0); }
inline MVector yaxis(const MMatrix& mat) { return maxis(mat, 1); }
inline MVector zaxis(const MMatrix& mat) { return maxis(mat, 2); }
inline MVector taxis(const MMatrix& mat) { return maxis(mat, 3); }

inline MMatrix& set_maxis(MMatrix& mat, unsigned int a, const MVector& v)
{
    mat[a][0] = v.x;
    mat[a][1] = v.y;
    mat[a][2] = v.z;
    return mat;
}

inline MVector mscale(const MMatrix& mat)
{
	return MVector(xaxis(mat).length(), yaxis(mat).length(), zaxis(mat).length());
}

inline MMatrix& set_mscale(MMatrix& mat, const MVector& scale)
{
    set_maxis(mat, 0, maxis(mat, 0).normal() * scale.x);
    set_maxis(mat, 1, maxis(mat, 1).normal() * scale.y);
    set_maxis(mat, 2, maxis(mat, 2).normal() * scale.z);
    return mat;
}

inline MQuaternion mat2quat(const MMatrix& inputMat)
{
	MMatrix mat(inputMat);
	set_mscale(mat, MVector(1,1,1)); // reset scale

	float tr, s;
	float q[4];
	int i, j, k;
	MQuaternion quat;

	int nxt[3] = { 1, 2, 0 };
	tr = mat[0][0] + mat[1][1] + mat[2][2];

	// check the diagonal
	if (tr > 0.0)
	{
		s = sqrt(tr + float(1.0));
		quat.w = s / float(2.0);
		s = float(0.5) / s;

		quat.x = (mat[1][2] - mat[2][1]) * s;
		quat.y = (mat[2][0] - mat[0][2]) * s;
		quat.z = (mat[0][1] - mat[1][0]) * s;
	}
	else
	{
		// diagonal is negative
		i = 0;
		if (mat[1][1] > mat[0][0])
			i = 1;
		if (mat[2][2] > mat[i][i])
			i = 2;

		j = nxt[i];
		k = nxt[j];
		s = sqrt((mat[i][i] - (mat[j][j] + mat[k][k])) + float(1.0));

		q[i] = s * float(0.5);
		if (s != float(0.0))
			s = float(0.5) / s;

		q[3] = (mat[j][k] - mat[k][j]) * s;
		q[j] = (mat[i][j] + mat[j][i]) * s;
		q[k] = (mat[i][k] + mat[k][i]) * s;

		quat.x = q[0];
		quat.y = q[1];
		quat.z = q[2];
		quat.w = q[3];
	}

	return quat;
}

inline MVector vectorMult(const MVector& v1, const MVector &v2)
{
	return MVector(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
}

inline MVector vectorLerp(const MVector& v1, const MVector& v2, double w)
{
	return MVector(
		v1.x * (1 - w) + v2.x * w,
		v1.y * (1 - w) + v2.y * w,
		v1.z * (1 - w) + v2.z * w);
}

inline MMatrix blendMatrices(const MMatrix& m1, const MMatrix& m2, double w)
{
	const MQuaternion q = slerp(mat2quat(m1), mat2quat(m2), w);
	MMatrix m = q.asMatrix();
	set_maxis(m, 3, vectorLerp(taxis(m1), taxis(m2), w));
	set_mscale(m, vectorLerp(mscale(m1), mscale(m2), w));
	return m;
}