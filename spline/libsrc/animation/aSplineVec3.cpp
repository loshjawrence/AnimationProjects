#include "aSplineVec3.h"
#include <algorithm>
#include <Eigen/Dense>

#pragma warning(disable:4018)
#pragma warning(disable:4244)

ASplineVec3::ASplineVec3() : mInterpolator(new ALinearInterpolatorVec3())
{
}

ASplineVec3::~ASplineVec3()
{
    delete mInterpolator;
}

void ASplineVec3::setFramerate(double fps)
{
    mInterpolator->setFramerate(fps);
}

double ASplineVec3::getFramerate() const
{
    return mInterpolator->getFramerate();
}

void ASplineVec3::setLooping(bool loop)
{
    mLooping = loop;
}

bool ASplineVec3::getLooping() const
{
    return mLooping;
}

void ASplineVec3::setInterpolationType(ASplineVec3::InterpolationType type)
{
    double fps = getFramerate();

    delete mInterpolator;
    switch (type)
    {
    case LINEAR: mInterpolator = new ALinearInterpolatorVec3(); break;
    case CUBIC_BERNSTEIN: mInterpolator = new ABernsteinInterpolatorVec3(); break;
    case CUBIC_CASTELJAU: mInterpolator = new ACasteljauInterpolatorVec3(); break;
	case CUBIC_MATRIX: mInterpolator = new AMatrixInterpolatorVec3(); break; 
	case CUBIC_HERMITE: mInterpolator = new AHermiteInterpolatorVec3(); break;
	case CUBIC_BSPLINE: mInterpolator = new ABSplineInterpolatorVec3(); break;
    };
    
    mInterpolator->setFramerate(fps);
    computeControlPoints();
    cacheCurve();
}

ASplineVec3::InterpolationType ASplineVec3::getInterpolationType() const
{
    return mInterpolator->getType();
}

void ASplineVec3::editKey(int keyID, const vec3& value)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys[keyID].second = value;
    computeControlPoints();
    cacheCurve();
}

void ASplineVec3::editControlPoint(int ID, const vec3& value)
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0)
    {
        mStartPoint = value;
        computeControlPoints();
    }
    else if (ID == mCtrlPoints.size() + 1)
    {
        mEndPoint = value;
        computeControlPoints();
    }
    else mCtrlPoints[ID-1] = value;
    cacheCurve();
}

void ASplineVec3::appendKey(double time, const vec3& value, bool updateCurve)
{
    mKeys.push_back(Key(time, value));

    if (mKeys.size() >= 2)
    {
        int totalPoints = mKeys.size();

        //If there are more than 1 interpolation point, set up the 2 end points to help determine the curve.
        //They lie on the tangent of the first and last interpolation points.
        vec3 tmp = mKeys[0].second - mKeys[1].second;
        double n = tmp.Length();
        mStartPoint = mKeys[0].second + (tmp / n) * n * 0.25; // distance to endpoint is 25% of distance between first 2 points

        tmp = mKeys[totalPoints - 1].second - mKeys[totalPoints - 2].second;
        n = tmp.Length();
        mEndPoint = mKeys[totalPoints - 1].second + (tmp / n) * n * 0.25;
    }

    if (updateCurve)
    {
        computeControlPoints();
        cacheCurve();
    }
}

void ASplineVec3::appendKey(const vec3& value, bool updateCurve)
{
    if (mKeys.size() == 0)
    {
        appendKey(0, value, updateCurve);
    }
    else
    {
        double lastT = mKeys[mKeys.size() - 1].first;
        appendKey(lastT + 1, value, updateCurve);
    }
}

void ASplineVec3::deleteKey(int keyID)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys.erase(mKeys.begin() + keyID);
    computeControlPoints();
    cacheCurve();
}

vec3 ASplineVec3::getKey(int keyID)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    return mKeys[keyID].second;
}

int ASplineVec3::getNumKeys() const
{
    return mKeys.size();
}

vec3 ASplineVec3::getControlPoint(int ID)
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0) return mStartPoint;
    else if (ID == mCtrlPoints.size() + 1) return mEndPoint;
    else return mCtrlPoints[ID-1];
}

int ASplineVec3::getNumControlPoints() const
{
    return mCtrlPoints.size()+2; // include endpoints
}

void ASplineVec3::clear()
{
    mKeys.clear();
}

double ASplineVec3::getDuration() const 
{
    return mKeys[mKeys.size()-1].first;
}

double ASplineVec3::getNormalizedTime(double t) const 
{
    return (t / getDuration());
}

vec3 ASplineVec3::getValue(double t)
{
    if (mCachedCurve.size() == 0) return vec3();

    double dt = mInterpolator->getDeltaTime();
    int rawi = (int)(t / dt); // assumes uniform spacing
    int i = rawi % mCachedCurve.size();
    double frac = t - rawi*dt;
    int inext = i + 1;
    if (!mLooping) inext = std::min<int>(inext, mCachedCurve.size() - 1);
    else inext = inext % mCachedCurve.size();

    vec3 v1 = mCachedCurve[i];
    vec3 v2 = mCachedCurve[inext];
    vec3 v = v1*(1 - frac) + v2 * frac;
    return v;
}

void ASplineVec3::cacheCurve()
{
    mInterpolator->interpolate(mKeys, mCtrlPoints, mCachedCurve);
}

void ASplineVec3::computeControlPoints()
{
    mInterpolator->computeControlPoints(mKeys, mCtrlPoints, mStartPoint, mEndPoint);
}

int ASplineVec3::getNumCurveSegments() const
{
    return mCachedCurve.size();
}

vec3 ASplineVec3::getCurvePoint(int i) const
{
    return mCachedCurve[i];
}

//---------------------------------------------------------------------
AInterpolatorVec3::AInterpolatorVec3(ASplineVec3::InterpolationType t) : mDt(1.0 / 120.0), mType(t)
{
}

void AInterpolatorVec3::setFramerate(double fps)
{
    mDt = 1.0 / fps;
}

double AInterpolatorVec3::getFramerate() const
{
    return 1.0 / mDt;
}

double AInterpolatorVec3::getDeltaTime() const
{
    return mDt;
}

void AInterpolatorVec3::interpolate(const std::vector<ASplineVec3::Key>& keys, 
    const std::vector<vec3>& ctrlPoints, std::vector<vec3>& curve)
{
	vec3 val = 0.0;
	double u = 0.0; 

	curve.clear();
	
	int numSegments = keys.size() - 1;
    for (int segment = 0; segment < numSegments; segment++)
    {
        for (double t = keys[segment].first; t < keys[segment+1].first - FLT_EPSILON; t += mDt)
        {
            
			// TODO: Compute u, fraction of duration between segment and segmentnext, for example,
            // u = 0.0 when t = keys[segment-1].first  
            // u = 1.0 when t = keys[segment].first
			double segment_duration = keys[segment+1].first - keys[segment].first;
			double elapsed_time_in_segment = t - keys[segment].first;
			u = elapsed_time_in_segment / segment_duration;
            val = interpolateSegment(keys, ctrlPoints, segment, u);
            curve.push_back(val);
        }
    }
	// add last point
	if (keys.size() > 1)
	{
		u = 1.0;
		val = interpolateSegment(keys, ctrlPoints, numSegments-1, u);
		curve.push_back(val);
	}
	
    
}


vec3 ALinearInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
   
	vec3 curveValue(0, 0, 0);
	vec3 key0 = keys[segment].second;
    vec3 key1 = keys[segment+1].second;

    // TODO: 
	//Step 1: Create a Lerp helper function
	curveValue = key0*(1 - u) + key1*u;
	//Step 2: Linear interpolate between key0 and key1 so that u = 0 returns key0 and u = 1 returns key1
    
	return curveValue;
}

vec3 ABernsteinInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	vec3 b0; 
	vec3 b1;
	vec3 b2; 
	vec3 b3;
    vec3 curveValue(0,0,0);
	//t is actually u
	double u = t;
    // TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];
	
	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
	vec3 b0influence = b0 * (1 - u) * (1 - u) * (1 - u);
	vec3 b1influence = b1 * 3 * u * (1 - u) * (1 - u);
	vec3 b2influence = b2 * 3 * u * u * (1 - u);
	vec3 b3influence = b3 * u * u * u;
	curveValue = b0influence + b1influence + b2influence + b3influence;
	return curveValue;
}


vec3 ACasteljauInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);
	double u = t;
	
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	// Step2: Compute the interpolated value f(u) point using  deCsteljau alogithm
	vec3 b01 = b0*(1 - u) + b1*u;
	vec3 b11 = b1*(1 - u) + b2*u;
	vec3 b21 = b2*(1 - u) + b3*u;

	vec3 b02 = b01*(1 - u) + b11*u;
	vec3 b12 = b11*(1 - u) + b21*u;

	vec3 b03 = b02*(1 - u) + b12*u;

	curveValue = b03;
	return curveValue;
}

vec3 AMatrixInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);
	double u = t;
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	// Step2: Compute the interpolated value f(u) point using  matrix method f(u) = GMU
	// Hint: Using Eigen::MatrixXd data representations for a matrix operations
    //U Vector
	Eigen::Vector4d  U(1, u, u*u, u*u*u); //creates a 4 entry vector of doubles

    //M Bezier Matrix
	Eigen::MatrixXd M_Bezier(4,4);
    M_Bezier(0,0) = 1; M_Bezier(0,1) = -3; M_Bezier(0,2) =  3; M_Bezier(0,3) = -1;
    M_Bezier(1,0) = 0; M_Bezier(1,1) =  3; M_Bezier(1,2) = -6; M_Bezier(1,3) =  3;
    M_Bezier(2,0) = 0; M_Bezier(2,1) =  0; M_Bezier(2,2) =  3; M_Bezier(2,3) = -3;
    M_Bezier(3,0) = 0; M_Bezier(3,1) =  0; M_Bezier(3,2) =  0; M_Bezier(3,3) =  1;

    //B Bezier Matrix
	Eigen::MatrixXd G_Bezier(4,4);
    G_Bezier(0,0) = b0[0], G_Bezier(0, 1) = b1[0], G_Bezier(0, 2) = b2[0], G_Bezier(0,3) = b3[0];
	G_Bezier(1,0) = b0[1], G_Bezier(1, 1) = b1[1], G_Bezier(1, 2) = b2[1], G_Bezier(1,3) = b3[1];
	G_Bezier(2,0) = b0[2], G_Bezier(2, 1) = b1[2], G_Bezier(2, 2) = b2[2], G_Bezier(2,3) = b3[2];
	G_Bezier(3,0) = 1    , G_Bezier(3, 1) = 1    , G_Bezier(3, 2) = 1    , G_Bezier(3,3) = 1    ;

    Eigen::Vector4d f_u = G_Bezier * (M_Bezier * U);
    
	double w = f_u[3];
    curveValue[0] = f_u[0] / w;
    curveValue[1] = f_u[1] / w;
    curveValue[2] = f_u[2] / w;
	return curveValue;
}

vec3 AHermiteInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{

    vec3 p0 = keys[segment].second;
    vec3 p1 = keys[segment + 1].second;
    vec3 q0 = ctrlPoints[segment]; // slope at p0
    vec3 q1 = ctrlPoints[segment + 1]; // slope at p1
	vec3 curveValue(0, 0, 0);
	double u = t;
    // TODO: Compute the interpolated value h(u) using a cubic Hermite polynomial  
    double H03 =  2*u*u*u - 3*u*u + 1;
    double H13 =    u*u*u - 2*u*u + u;
    double H23 =    u*u*u -   u*u;
    double H33 = -2*u*u*u + 3*u*u;

    curveValue = p0*H03 + q0*H13 + q1*H23 + p1*H33;
    
    return curveValue;
}


vec3 ABSplineInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double t)
{
	vec3 curveValue(0, 0, 0);
	
	// Hint: Create a recursive helper function N(knots,n,j,t) to calculate BSpline basis function values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = curve interval on knot vector in which to interpolate
	//     t = time value	

	// Step 1: determine the index j
	float _t =  segment + t;
	double u = t;
	unsigned int n = 3;
	unsigned int j = n;

	unsigned int m = keys.size() - 1;//m is an index value
	std::vector<float> knots(m + 1 + (2 * n) + 1);
	//assumes even intervals of 1 sec
	int stop = knots.size() - n;
	for (int i = -1 * n; i < stop; i++) {
		knots[i + n] = i;
	}

	// Step 2: compute the n nonzero Bspline Basis functions N given j
	std::vector<float> basisfuncsN(n + 1);
	//for (int j = segment; j <= segment + n; j++) {
	//	basisfuncsN[j-segment] = N(knots, n, _t, j);
	//}

	basisfuncsN[0] = ( (1-u) * (1-u) * (1-u) ) / 6;
	basisfuncsN[1] = ( 4 - 6*u*u + 3*u*u*u ) / 6;
	basisfuncsN[2] = ( 1 + 3*u + 3*u*u - 3*u*u*u ) / 6;
	basisfuncsN[3] = ( u*u*u ) / 6;


	// Step 3: get the corresponding control points from the ctrlPoints vector
	// Step 4: compute the Bspline curveValue at time t
	for (int j = segment; j <= segment + n; j++) {
		curveValue = curveValue + (ctrlPoints[j] * basisfuncsN[j-segment]);
	}
	
	return curveValue;
}

void ACubicInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys, 
    std::vector<vec3>& ctrlPoints, 
    vec3& startPoint, vec3& endPoint)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;

    for (int i = 1; i < keys.size(); i++)
    {
        vec3 b0, b1, b2, b3;
		vec3 slope_u_0, slope_u_1;
        // TODO: compute b0, b1, b2, b3

		//b0 and b3 are simply the keys for the curve segment
		b0 = keys[i - 1].second;
		b3 = keys[i].second;

		//middle control points b1 and b2 are derived from slopes at end points b0 and b3
		//slope at b0 (u = 0)
		if (i == 1) { //slope at left side of spline
			slope_u_0 = 3* (keys[i - 1].second - startPoint); //keys[i].second - keys[i - 1].second;
		}
		else {	// slope for middle segment of spline 
			slope_u_0 = (keys[i].second - keys[i - 2].second) / 2;
		}
		
		//Slope at b3 (u = 1)
		if (i == keys.size() - 1) {	// right side of spline			
			slope_u_1 = 3 * (endPoint - keys[i].second); //keys[i].second - keys[i - 1].second;
		} else { //slope for middle segments of spline
			slope_u_1 = (keys[i + 1].second - keys[i - 1].second) / 2;
		}

		//for cubic curves b1 and b2 are defined as:
		b1 = b0 + 0.33 * slope_u_0;
		b2 = b3 - 0.33 * slope_u_1;

		ctrlPoints.push_back(b0);
        ctrlPoints.push_back(b1);
        ctrlPoints.push_back(b2);
        ctrlPoints.push_back(b3);
    }
}

void AHermiteInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints,
    vec3& startPoint, vec3& endPoint)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;

    int numKeys = keys.size();


    // TODO: 
	// For each key point pi, compute the corresonding value of the slope pi_prime.
	// Hints: Using Eigen::MatrixXd for a matrix data structures, 
	// this can be accomplished by solving the system of equations AC=D for C.
	// Don't forget to save the values computed for C in ctrlPoints
	// For clamped endpoint conditions, set 1st derivative at first and last points (p0 and pm) to s0 and s1, respectively
	// For natural endpoints, set 2nd derivative at first and last points (p0 and pm) equal to 0

	// Step 1: Initialize A
    Eigen::MatrixXd A(numKeys, numKeys);
    for (int row = 0; row < numKeys; row++) {
        for (int col = 0; col < numKeys; col++) {
            double val = 0;
            if ( std::abs(row - col) == 1 ) {
				val = 1;
            } else if ( row == col ) {
                val = 4;
            }
            A(row,col) = val;
        }
    }
    //Natural Hermite Endpoint conditions (second derivative at endpoints = 0)
    A(0,0) = A(numKeys-1, numKeys-1) = 2;

	//// Step 2: Initialize D
    Eigen::MatrixXd D(numKeys,3);
    for(int row = 1; row < numKeys-1; row++) { //don't do endpoints yet(first and last row)
		D(row, 0) = keys[row + 1].second[0] - keys[row - 1].second[0];//x
		D(row, 1) = keys[row + 1].second[1] - keys[row - 1].second[1];//y
		D(row, 2) = keys[row + 1].second[2] - keys[row - 1].second[2];//z
    }

    //first entry in D  
	D(0, 0) = keys[1].second[0] - keys[0].second[0];//x
    D(0, 1) = keys[1].second[1] - keys[0].second[1];//y
    D(0, 2) = keys[1].second[2] - keys[0].second[2];//z

    //last entry in D 
    D(numKeys-1, 0) = keys[numKeys-1].second[0] - keys[numKeys-2].second[0];//x
    D(numKeys-1, 1) = keys[numKeys-1].second[1] - keys[numKeys-2].second[1];//y
    D(numKeys-1, 2) = keys[numKeys-1].second[2] - keys[numKeys-2].second[2];//z
    
    //scale by 3
    D *= 3;

	//// Step 3: Solve AC=D for C
    Eigen::MatrixXd C(numKeys, 3);
    C = A.inverse() * D;

	// Step 4: Save control points in ctrlPoints
	for (int row = 0; row < numKeys; row++) {
		ctrlPoints.push_back( vec3( C(row, 0), C(row, 1), C(row, 2) ) );
	}

}



void ABSplineInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints, 
    vec3& startPt, vec3& endPt)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;

    // TODO: c
    // Hints: 
	// 1. use Eigen::MatrixXd to calculate the control points by solving the system of equations AC=D for C
	
    // 2. Create a recursive helper function dN(knots,n,t,l) to calculate derivative BSpline values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = interval on knot vector in which to interpolate
	//     t = time value
	//     l = derivative (l = 1 => 1st derivative)

	// Step 1: Calculate knot vector using a uniform BSpline
	//         (assume knots are evenly spaced 1 apart and the start knot is at time = 0.0)
	unsigned int m = keys.size() - 1;//m is an index value
	unsigned int n = 3;
	std::vector<float> knots(m+1 + (2*n) + 1);

	//assumes even intervals of 1 sec
	int stop = knots.size() - n;
	for (int i = -1 * n; i < stop; i++) {
		knots[i + n] = i;
	}

	// Step 2: Calculate A matrix  for a natural BSpline
	//         (Set 2nd derivative at t0 and tm to zero, where tm is the last point knot; m = #segments)
	Eigen::MatrixXd A(m + n, m + n);

	for (int row = 1; row < A.rows() - 1; row++) {
		for (int col = 0; col < A.cols(); col++) {
			unsigned int t = knots[row - 1 + n];
			A(row,col) = N(knots, n, t, col);
		}
	}

	//fill in second to last row
	for (int i = 0; i <= n; i++) {
		A(m + 1, m - 1 + i) = N(knots, n, m, m - 1 + i);
	}

	//fill in first row (second derivative values)
	for(int col = 0; col < A.cols(); col++) {
		float val = 0;
		if (col <= n) {
			val = dN(knots, n, 0, col, 2);
		}
		A(0, col) = val;
	}

	//fill in last row
	for (int col = 0; col < A.cols(); col++) {
		float val = 0;
		if (col > m - 1) {
			int i = col - m;
			val = dN(knots, n, m, m + i, 2);
		}
		A(m + 2, col) = val;
	}

	// Step 3: Calculate  D matrix composed of our target points to interpolate
	Eigen::MatrixXd D(m + n , 3);
	for (int i = 0; i < keys.size(); i++) {
		D(i + 1, 0) = keys[i].second[0];//x
		D(i + 1, 1) = keys[i].second[1];//y
		D(i + 1, 2) = keys[i].second[2];//z
	}
	//first entry 
	D(0, 0) = 0;//x
	D(0, 1) = 0;//y
	D(0, 2) = 0;//z
	//last entry 
	D(m + n - 1, 0) = 0;//x
	D(m + n - 1, 1) = 0;//y
	D(m + n - 1, 2) = 0;//z

	// Step 4: Solve AC=D for C 
	Eigen::MatrixXd C(m + n, 3);
	C = A.inverse() * D;
	
	// Step 5: save control points in ctrlPoints
	for (int row = 0; row < C.rows(); row++) {
		ctrlPoints.push_back(vec3(C(row, 0), C(row, 1), C(row, 2)));
	}
	std::cout << "\nA : \n" << A << "\n";
	std::cout << "\nD : \n" << D << "\n";
	std::cout << "\nC : \n" << C << "\n";
}

float N(const std::vector<float>& knots, const unsigned int& n, const unsigned int& t, const unsigned int& j) {
	if (n == 0) {
		if (knots[j] <= t && t < knots[j + 1]) {
			return 1;
		}
		else {
			return 0;
		}
	}

	if (t < knots[j] || t >= knots[j+n+1]) {
		return 0;
	}

	float Nj_coeff   = (t - knots[j]) / (knots[j + n] - knots[j]);
	float Njp1_coeff = (knots[j + n + 1] - t) / (knots[j + n + 1] - knots[j + 1]);
	float first_term = Nj_coeff * N(knots, n - 1, t, j);
	float second_term = Njp1_coeff * N(knots, n - 1, t, j + 1);
	return  first_term + second_term;
}

float dN(const std::vector<float>& knots, const unsigned int& n, const unsigned int& t, const unsigned int& j, const unsigned int& l) {
	float Nj_coeff   = 1 / (knots[j + n] - knots[j]);
	float Njp1_coeff = 1 / (knots[j + n + 1] - knots[j + 1]);
	if (l == 0) {
		return N(knots, n, t, j);
	} else {
	    return n * ( Nj_coeff   * dN(knots, n - 1, t, j, l-1)
		           - Njp1_coeff * dN(knots, n - 1, t, j + 1, l-1) );
    }
}
