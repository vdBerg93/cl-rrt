const double pi = M_PI;
const double inf = std::numeric_limits<double>::infinity();

struct vector2D{
    double dx, dy;
};

struct OBB{
    double theta, size_x, size_y, area;
};

double myDot(vector2D a, vector2D b){
    return a.dx*b.dx + a.dy*b.dy;
};


struct Vertices{
	double x[4];
	double y[4];
    Vertices(const car_msgs::Obstacle2D& det){
        double Cx = det.obb.center.x;
        double Cy = det.obb.center.y;
        double o  = det.obb.center.theta;
        double h = det.obb.size_x;
        double w = det.obb.size_y;
        // Pushback vertices
        x[0] = Cx+cos(o)*(h/2)-sin(o)*(w/2);             // FL
        y[0] = Cy+sin(o)*(h/2)+cos(o)*(w/2);           // FL
        x[1] = Cx+cos(o)*(h/2)-sin(o)*(-w/2);            // FR
        y[1] = Cy+sin(o)*(h/2)+cos(o)*(-w/2);          // FR
        x[2] = Cx+cos(o)*(-h/2)-sin(o)*(-w/2);           // RR
        y[2] = Cy+sin(o)*(-h/2)+cos(o)*(-w/2);         // RR
        x[3] = Cx+cos(o)*(-h/2)-sin(o)*(w/2);            // RL
        y[3] = Cy+sin(o)*(-h/2)+cos(o)*(w/2);          // RL
    }
};

double wrapTo2Pi(double x){
    // Wrap angle to [0,2pi]
    x = fmod(x,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x;
}