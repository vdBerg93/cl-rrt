#ifndef functions_h
#define functions_h

#include <numeric>

const double inf = std::numeric_limits<double>::infinity();
const double pi = M_PI;

using namespace std;
// Linear interpolation following MATLAB linspace
vector<double> LinearSpacedVector(double a, double b, std::size_t N)
{
    double h = (b - a) / static_cast<double>(N-1);
    std::vector<double> xs(N);
    std::vector<double>::iterator x;
    double val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
        *x = val;
    }
    return xs;
}

vector<double> bezierCurveInterpolation(vector<double> cp, vector<double> t){
		vector<double> vel_vector;
	for(int i =0; i<=t.size(); i++){
		vel_vector.push_back( pow(1-t[i],3)*cp[0] + 3*pow(1-t[i],2)*t[i]*cp[1] + 3*pow(1-t[i],2)*cp[2] + pow(t[i],3)*cp[3] );
	}
	return vel_vector;
}

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

double wrapTo2Pi(double x){
    // Wrap angle to [0,2pi]
    x = fmod(x,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x;
}

double wrapToPi(double x){
    x = fmod(x + pi,2*pi);
    if (x < 0)
        x += 2*pi;
    return x - pi;
}

double angleDiff(double a,double b){
    // Angle difference on interval [0,2pi]
    double dif = fmod(b - a + M_PI,2*M_PI);
    if (dif < 0){
        dif += 2*M_PI;
    }
    return dif - M_PI;
}



double checkSaturation(double min, double max, double val){
    return std::max(std::min(val,max),min);
}


#endif
