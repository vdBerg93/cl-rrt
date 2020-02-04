#ifndef COLLISION_H
#define COLLISION_H

// Define 2D vector class
struct Vector2D{
    double x;
    double y;
    Vector2D(double _x, double _y): x(_x), y(_y){};
};

// Define Vector2D dot operation
template <class T>
double myDot(T a, T b){
    return (a.x*b.x + a.y*b.y);
}

class OBB{
    public:
        Vector2D pos;    // Center
        float w;           // Width
        float h;           // Height
        float o;           // Orientation
        
        OBB(Vector2D _pos, float _w, float _h, float _o): pos(_pos), w(_w), h(_h), o(_o){
            setVertices();
            setNorms();
        }
        void findMaxMin(float x,float y);
        float normsX[4], normsY[4];         // Normal axis
        float verticesX[4], verticesY[4];   // Vertices
        float maxMin[2];
    private:
        void setVertices();
        void setNorms();
};

// Primitives
double getOBBdist(OBB a, OBB b);
bool checkCollisionTra(const StateArray& T, const vector<car_msgs::Obstacle2D>& det, const vector<double>& carState);
double checkObsDistance(const vector<double>& states, const vector<car_msgs::Obstacle2D>& det, const vector<double>& carState);
vector<OBB> getOBBvector(const vector<car_msgs::Obstacle2D>& det, const double& t, const vector<double>& carState);

#endif