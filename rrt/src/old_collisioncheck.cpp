/* -------------------------------------------
    COLLISION DETECTION FUNCTIONS
--------------------------------------------*/
#include "rrt/collision.h"

vector<OBB> getOBBvector(const vector<car_msgs::Obstacle2D>& det, const double& t, const vector<double>& carState){
    vector<OBB> obstacleVector;
    for(int i = 0; i!=det.size(); i++){
        // Get OBB for future state
        double h = det[i].obb.center.theta;
        // OBB obs(Vector2D(det[i].obb.center.x + (det[i].vel.linear.x+cos(h)*carState[4])*t,
        //                 det[i].obb.center.y + det[i].vel.linear.y*t),
        //                 det[i].obb.size_x/2,det[i].obb.size_y/2,det[i].obb.center.theta);
        OBB obs(Vector2D(det[i].obb.center.x + det[i].vel.linear.x*t,
                        det[i].obb.center.y + det[i].vel.linear.y*t),
                        det[i].obb.size_x/2,det[i].obb.size_y/2,det[i].obb.center.theta);
        obstacleVector.push_back(obs);
    }

    // if(draw_obs){    drawObstacles(ptrPub,obstacleVector);}
    return obstacleVector;
}

double checkObsDistance(const vector<double>& states, const vector<car_msgs::Obstacle2D>& det, const vector<double>& carState){
    double t;
    if (obs_use_pred){
        t = states[6];
    }else{
        t = 0;
    }
    // t = 0;
    
    vector<OBB> obstacleVector = getOBBvector(det,t,carState);
    Vector2D vPos(states[0]+1.424*cos(states[2]),states[1]+1.424*sin(states[2]));
    ROS_WARN_STREAM_THROTTLE(5,"In CD: Check vehicle dimensions");
    OBB vOBB(vPos,2,4.848,states[2]); // Create vehicle OBB

    double dist2closest = 10000;
    for(int j = 0; j !=obstacleVector.size(); j++){
        double D = getOBBdist(vOBB,obstacleVector[j]);
        assert( (D>=0)&&"Obstacle distance is negative!");
        // ROS_INFO_STREAM("Obs dist="<<D);
        if (D==0){
            return 0;
        }else if (D < dist2closest){
            dist2closest = D;
        }
    }
    // No collisions
    return dist2closest;
}




void OBB::setVertices(){
    verticesX[0] = pos.x+cos(o)*(h/2)-sin(o)*(w/2);             // FL
    verticesY[0] = pos.y+sin(o)*(h/2)+cos(o)*(w/2);           // FL
    verticesX[1] = pos.x+cos(o)*(h/2)-sin(o)*(-w/2);            // FR
    verticesY[1] = pos.y+sin(o)*(h/2)+cos(o)*(-w/2);          // FR
    verticesX[2] = pos.x+cos(o)*(-h/2)-sin(o)*(-w/2);           // RR
    verticesY[2] = pos.y+sin(o)*(-h/2)+cos(o)*(-w/2);         // RR
    verticesX[3] = pos.x+cos(o)*(-h/2)-sin(o)*(w/2);            // RL
    verticesY[3] = pos.y+sin(o)*(-h/2)+cos(o)*(w/2);          // RL
}

void OBB::setNorms(){
    int i = 0;
    while (i<3){
        normsX[i] = verticesY[i+1]-verticesY[i];
        normsY[i] = -(verticesX[i+1]-verticesX[i]);
        i++;
    }
    normsX[3] = verticesY[0]-verticesY[3];
    normsX[3] = -(verticesX[0]-verticesX[3]);
}

void OBB::findMaxMin(float x, float y){
    // First vertice as baseline
    maxMin[0] = verticesX[0]*x + verticesY[0]*y;    // Initial maximum
    maxMin[1] = maxMin[0];                                      // Initial minimum
    // Iterate through the remaining
    for(int i = 1; i <= 3; i++){
        float proj = verticesX[i]*x + verticesY[i]*y;
        // if new maximum
        if (proj > maxMin[0]){
            maxMin[0] = proj;
        }
        // if new minimum
        else if (proj < maxMin[1]){
            maxMin[1] = proj;
        }
    }
    return;
}

// Collison checking two rectangles OBB using the Separating Axis Theorem
double getOBBdist(OBB a, OBB b){
    // http://blog.marcher.co/sat1/
    // https://gamedevelopment.tutsplus.com/tutorials/collision-detection-using-the-separating-axis-theorem--gamedev-169
    //SWRI_PROFILE("intersects");

    // circle collision optimization
    // Vector2D d(a.pos.x-b.pos.x, a.pos.y-b.pos.y); // change this operation
    // float distSq = myDot(d,d);
    // float r = max(a.w, a.h) + max(b.w, b.h);
    // if (distSq > r*r){
    //     //cout<<"passed circle test! \n \n"<<endl;
    //     return false;
    // }
    // SAT
    // Check for first OBB
    float axis[2];
    for(int i =0; i <= 3; i++){
        a.findMaxMin(a.normsX[i],a.normsY[i]);
        float aProj[2] {a.maxMin[0],a.maxMin[1]};
        b.findMaxMin(a.normsX[i],a.normsY[i]);
        float bProj[2] {b.maxMin[0],b.maxMin[1]};
        // Check if this axis gives a separating axis
        // float D1 = bProj[0]-aProj[1];
        // float D2 = aProj[0]-bProj[1];
        float D1 = bProj[1]-aProj[0];
        float D2 = aProj[1]-bProj[0];
        if (D1>0){
            return D1;
        }else if (D2>0){
            return D2;
        }
    }
   for(int i =0; i <= 3; i++){
        a.findMaxMin(b.normsX[i],b.normsY[i]);
        float aProj[2] {a.maxMin[0],a.maxMin[1]};
        b.findMaxMin(b.normsX[i],b.normsY[i]);
        float bProj[2] {b.maxMin[0],b.maxMin[1]};
        // Check if this axis gives a separating axis
        // float D1 = bProj[0]-aProj[1];
        // float D2 = aProj[0]-bProj[1];
        float D1 = bProj[1]-aProj[0];
        float D2 = aProj[1]-bProj[0];
        if (D1>0){
            return D1;
        }else if (D2>0){
            return D2;
        }
    }
    // No separating axis exists -> intersection
    return 0;
}
