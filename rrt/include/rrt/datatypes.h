#ifndef DATATYPES_H
#define DATATYPES_H

struct point2D{
    double x,y;
    point2D(double _x, double _y){
        x = _x; y = _y;
    }
};

typedef vector<double> state_type;
typedef vector<vector<double>> StateArray;

struct R2S{
    float x,y,theta;
    R2S(float _x, float _y, float _theta):x(_x), y(_y), theta(_theta){};
};

// struct CarFrame{
//     R2S origin;
// };

struct poly{
    float c2,c1,c0;
};
// struct RoadFrame{
//     R2S origin;
//     poly xy;
//     poly xs;
//     float c2,c1,c0;
//     RoadFrame(vector<double> Xcar, vector<double> Cxy, vector<double> Cxs);
//     void setOrigin(){
//     }
// };


#endif
