//Imports
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sstream>
#include "basic_teleop/Move.h"
#include "limits.h"
#include <typeinfo>
#include "sensor_msgs/CameraInfo.h"
#include <cmath>
#include <boost/date_time.hpp>
#include <boost/lexical_cast.hpp>
#include <array>
#include <vector>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdlib.h>

//Constants
#define WIDTH 320
#define HEIGHT 240
#define PRINT_NAME(x) std::cout << #x << " - " << typeid(x).name() << '\n' // For Debugging
#define DEGREES_PER_SECOND 90
#define X_SHIFT .16
#define BLOB_TOLERANCE 15
#define BLOB_MIN_SIZE 25
#define HORIZON 138

//Namespaces
namespace btt = boost::posix_time;


//Types and Classes
union depth_data { //For processing data as mm values
    unsigned char raw[WIDTH * HEIGHT * 2]; //uint8 counterpart
    unsigned short int mm[WIDTH * HEIGHT]; //unit16 counterpart
};

class point2d {
    public:
        point2d(int x, int y) : x(x), y(y) {};
        int x;
        int y;
};

class point2dDepth {
    public:
        point2dDepth() : x(0), y(0), depth(0.0) {};
        point2dDepth(int x, int y, double d) : x(x), y(y), depth(d) {};
        int x;
        int y;
        double depth;
        inline bool operator == (const point2dDepth &other) {
            //ROS_INFO("Point Comparison: %d %d %d %d", x, y, other.x, other.y);
            //ROS_INFO("Val: %d", (x == other.x) && (y == other.y));
            return (x == other.x) && (y == other.y);
        }
};

class point3d {
    public:
        point3d(double x, double y, double z) : x(x), y(y), z(z) {};
        double x;
        double y;
        double z;
};

template<class T>
class Blob {
    public:
        Blob() : including() {};
        Blob(T point) {
            including = { point };
        } 
        Blob(std::vector<T> i) : including(i) {};
        int getArea() {return 0;};
        std::array<int, 2> getCenter() {return std::array<int, 2> { {0, 0} }; };
        bool inBlob(T point) {
            for(int i = 0; i < including.size(); i++) { //Switch to more efficient search later
                if (including[i] == point) {
                    return true;
                }
            }
            return false;
        }
        bool isEmpty() {
            return including.size() == 0;        
        }
        int numMembers() {
            return including.size();
        }
        void addPoint(T point) {
            including.push_back(point);
        }
    private:
        std::vector<T> including;
};

//Globals
sensor_msgs::CameraInfo camera;
basic_teleop::Move last_dir;
Blob<point2dDepth> last_blob;
std::array<float, 3> last_untransformed;
std::array<float, 3> velocity; // m/s
btt::ptime last_time = btt::microsec_clock::universal_time();
bool received_first = false;
bool received_2 = false;
bool calibrate_lag = false;
depth_data d;

//Protofunctions
depth_data smoothFrame(const depth_data&);
short int average_pixel(const depth_data&, int, int);
void sort_into_by_depth(std::vector<point2dDepth>*, point2dDepth);
std::vector<point2dDepth> get_higher_neighbors(const point2dDepth&, const depth_data&);
int decide_lindeberg_case(point2dDepth, const std::vector<point2dDepth>&, std::vector<point2dDepth>*, std::vector<Blob<point2dDepth>>);
int getXY(const depth_data&, int, int);
point2dDepth getXY(const depth_data&, point2d);
void print_closest_point_pixel_coords(const depth_data&);
std::array<float, 3> conv_to_3d(const depth_data&, int, int, const double*);
std::array<int, 2> conv_to_2d(const std::array<float, 3>, const double*);
void change_velocity(const std::array<float, 3>);

//Class Member Definitions


//Helper Definitions

depth_data smoothFrame(const depth_data& frame) {
    depth_data smoothed;
    for(int x = 0; x < WIDTH; x++) {
        for(int y = 0; y < HEIGHT; y++) {
            smoothed.mm[x + (y * WIDTH)] = average_pixel(frame, x, y);
        }
    }
    return smoothed;
}

short int average_pixel(const depth_data& frame, int x, int y) {
    short int averaged = (( getXY(frame, x + 1, y)
                  + getXY(frame, x + 1, y + 1)
                  + getXY(frame, x + 1, y - 1)
                  + getXY(frame, x, y)
                  + getXY(frame, x, y + 1)
                  + getXY(frame, x, y - 1)
                  + getXY(frame, x - 1, y)
                  + getXY(frame, x - 1, y + 1)
                  + getXY(frame, x - 1, y - 1)) / 9);
    if (averaged > 0) {
        ROS_INFO("Found nonzero average %d, %d: %d", x, y, averaged);
    }
    return averaged;
}

std::vector<Blob<point2dDepth>> blobFrame(const depth_data& frame) {
    std::vector<Blob<point2dDepth>> output;

    depth_data sFrame = smoothFrame(frame);    
    
    ROS_INFO("Checking depth on smoothed frame: %d", getXY(sFrame, 280, 1));

    
    //transform grid to pixel array - ignore things below the horizon
    std::vector<point2dDepth> sorted;
    for (int x = 0; x < WIDTH; x++) {
        for (int y = 0; y < HORIZON; y++) {
            sort_into_by_depth(&sorted, getXY(sFrame, point2d(x, y)));
        }
    }

    /*for (int k = 0; k < sorted.size(); k++) {
        if (sorted[k].depth < 0.1) {
            ROS_INFO("Pixel #%d: %d, %d depth: %f", k, sorted[k].x, sorted[k].y, sorted[k].depth);
            while(true) {};
        }        
    }*/

    //while(true) {}; // halt program

    //sort pixels into blobs + background

    int counter = 0;
    std::vector<point2dDepth> background;
    for (int i = 0; i < sorted.size(); i++) {
        point2dDepth p = sorted[i];
        std::vector<point2dDepth> neighbors = get_higher_neighbors(p, sFrame);
        //ROS_INFO("[%d, %d]", p.x, p.y);        
        if(neighbors.size() == 0) {
            point2dDepth plateau_check = sorted[i-1];
            if (p.depth != 0 && p.depth >= plateau_check.depth - .000001 && p.depth <= plateau_check.depth + .000001 && // actually I hate doubles
                plateau_check.x <= p.x + 1 &&
                plateau_check.x >= p.x - 1 &&
                plateau_check.y <= p.y + 1 &&
                plateau_check.y >= p.y - 1) { //deal with edge case of equal height neighbors
                ROS_INFO("Pleateau case found (%d, %d, %f), blobs found %d", plateau_check.x, plateau_check.y, plateau_check.depth, output.size());
                //ROS_INFO("Test: %d", plateau_check.x, plateau_check.y, plateau_check.depth, output.size());
                for (Blob<point2dDepth> b : output) {
                    if (b.inBlob(plateau_check)) {
                        b.addPoint(p);
                        ROS_INFO("Found Blob For Point with %d members", b.numMembers());
                        //while (true) {} ;
                        break;
                    }                
                }
                continue;
            } 
            
            if(output.size() >= 9) {
                background.push_back(p);
                continue;            
            }
            output.push_back(Blob<point2dDepth>(p));
        }
        //else if(p.depth > 1.0) {
        //    background.push_back
        //}
        else {
            decide_lindeberg_case(p, neighbors, &background, output);
        }
        counter++;
        //ROS_INFO("[Classified: %d]", counter);        
    }
    
    //sort blobs by closeness/size?

    //return blobs sorted by closeness/size?
    ROS_INFO("Pixel 0, 0 depth: %d", getXY(sFrame, 0, 0));
    ROS_INFO("Pixel 1, 0 depth: %d", getXY(sFrame, 1, 0));
    ROS_INFO("Pixel 2, 0 depth: %d", getXY(sFrame, 2, 0));
    return output;
}

void sort_into_by_depth(std::vector<point2dDepth>* v, point2dDepth p) { //given that V is sorted
    for(int i = 0; i < v->size(); i++) {
        //ROS_INFO("[Depth: %f]", p.depth);
        if (p.depth > 0.2 && (p.depth <= v->at(i).depth || v->at(i).depth == 0) ) {//20cm is below the tolerance of the sensor, and if you've hit the zeros you should stop
            v->insert(v->begin() + i, p);
            return;
        }
    }
    if(p.depth > 0){
    ROS_INFO("[Mid Sorted Number: %d]", v->size());
    }
    v->push_back(p);
}

std::vector<point2dDepth> get_higher_neighbors(const point2dDepth& p, const depth_data& frame) {
    std::vector<point2dDepth> output;
    point2dDepth temp;
    
    temp = getXY(frame, point2d(p.x + 1, p.y));
    if (temp.depth != 0 && temp.depth < p.depth) {
        output.push_back(temp);
    }

    temp = getXY(frame, point2d(p.x + 1, p.y + 1));
    if (temp.depth != 0 && temp.depth < p.depth) {
        output.push_back(temp);
    }

    temp = getXY(frame, point2d(p.x + 1, p.y - 1));
    if (temp.depth != 0 && temp.depth < p.depth) {
        output.push_back(temp);
    }   
 
    temp = getXY(frame, point2d(p.x, p.y + 1));
    if (temp.depth != 0 && temp.depth < p.depth) {
        output.push_back(temp);
    }

    temp = getXY(frame, point2d(p.x, p.y - 1));
    if (temp.depth != 0 && temp.depth < p.depth) {
        output.push_back(temp);
    }

    temp = getXY(frame, point2d(p.x - 1, p.y));
    if (temp.depth != 0 && temp.depth < p.depth) {
        output.push_back(temp);
    }

    temp = getXY(frame, point2d(p.x - 1, p.y + 1));
    if (temp.depth != 0 && temp.depth < p.depth) {
        output.push_back(temp);
    }

    temp = getXY(frame, point2d(p.x - 1, p.y - 1));
    if (temp.depth != 0 && temp.depth < p.depth) {
        output.push_back(temp);
    }
    
    if (p.depth > 0) {
        ROS_INFO("Found nonzero point %d, %d, %f", p.x, p.y, p.depth);
        ROS_INFO("Neighbornum: %d", output.size());
        //while(true){}; //Next step is how to solve edge case of equal neighbors
    }
    return output;
}

int decide_lindeberg_case(point2dDepth point, const std::vector<point2dDepth>& neighbors, std::vector<point2dDepth>* background, std::vector<Blob<point2dDepth>> blobs) {
    Blob<point2dDepth>* potentialBlob;
    //ROS_INFO("[Neighbor Number: %d]", neighbors.size());
    //ROS_INFO("[Background Number: %d]", background->size());
    //ROS_INFO("[Blob Number: %d]", blobs.size());    
    for(point2dDepth neighbor : neighbors) {
        
        if(std::find(background->begin(), background->end(), neighbor) != background->end()) {
            background->push_back(point); // is pushing back better o(n) than putting to the front for searching in the background??            
            return 0;
        }
        else if (potentialBlob == NULL) {
            for (Blob<point2dDepth> b : blobs) {
                if (b.inBlob(neighbor)){
                    potentialBlob = &b;
                    break;
                }
            }
        }
        else {
            if (!potentialBlob->inBlob(neighbor)) {
                background->push_back(point);
                return 1;
            }
        }
    }
    potentialBlob->addPoint(point);
    return 2;
} 

void printFrameWithBlobs(std::vector<Blob<point2dDepth>> blobs) {
    std::ofstream outFile;
    outFile.open("output.txt");
    ROS_INFO("[Blob number: %d]", blobs.size());
    for(int row = 0; row < HEIGHT; row++) {
        std::string oneRow;
        for(int col = 0; col < WIDTH; col++) {
            bool added = false;
            if(row >= HORIZON) {
                oneRow += "0";
                continue;
            }
            for(int k = 0; k < blobs.size(); k++) {
                if(blobs[k].inBlob(point2dDepth(col, row, 0))) {
                    oneRow += boost::lexical_cast<std::string>(k + 1);
                    added = true;
                    break;
                }
            }
            if(!added) {
                oneRow += "0";
            }
        }
        //ROS_INFO("[%s]", oneRow.c_str());
        outFile<<oneRow.c_str();
        outFile<<"\n";
    }
    outFile.close();
}

inline int getXY(const depth_data& grid, int x, int y) {
    if( x > 0 && x < WIDTH && y > 0 && y < HEIGHT) {
        return grid.mm[x + (y * WIDTH)];
    }
    return 0;
}


inline point2dDepth getXY(const depth_data& grid, point2d p) {
    if( p.x > 0 && p.x < WIDTH && p.y > 0 && p.y < HEIGHT && getXY(grid, p.x, p.y) > 0) {
        return point2dDepth(p.x, p.y, getXY(grid, p.x, p.y)/1000.0); // + (rand() % 100)/ 100000.0); // add random distribution to points to try to avoid strict equality during smoothing
    }
    return point2dDepth(p.x, p.y, 0.0);
}

void set_camera_info(sensor_msgs::CameraInfo c){
    camera = c;
}

std::array<float, 3> conv_to_3d(const depth_data& grid, int x, int y, const double* intr){
    //using default values found online    
    float cx = 319.5;// intr[2];
    float cy = 239.5;// intr[5];
    float fx_inv = 1.0 / 525.0;// intr[0];
    float fy_inv = 1.0 / 525.0;// intr[4];

    float zO = getXY(grid, x, y) * .001; // z
    float xO = (zO * ((x - cx) * fx_inv)) + X_SHIFT; //x
    float yO = zO * ((y - cy) * fy_inv); //y

    return std::array<float, 3> { {xO, yO, zO} };
}

std::array<int, 2> conv_to_2d(const std::array<float, 3> in, const double* intr){
    //using default values found online    
    float cx = 319.5;// intr[2];
    float cy = 239.5;// intr[5];
    float fx = 525.0;// intr[0];
    float fy = 525.0;// intr[4];    

    return std::array<int, 2> { 
                              {(((in[0] - X_SHIFT) * fx) / in[2]) + cx,
                               ((in[1] * fy) / in[2]) + cy}
                              };
}

std::array<float, 3> rotate_point_around_robot(const std::array<float, 3> in, float angle) {
    float x = in[0];// height doesn't change with turning
    float z = in[2];

    angle = angle*M_PI/180; // Convert to radians
    
    float pivX = 0; //point around center of robot
    float pivZ = -.06;

    x -= pivX;
    z -= pivZ;

    x = x*cos(angle) - z*sin(angle);
    z = z*cos(angle) + x*sin(angle);

    x += pivX;
    z += pivZ;

    return std::array<float, 3> { {x, in[1], z} }; //care for pointer problems
}

void change_velocity(const std::array<float, 3> new_transformed) {
    double time_passed = (btt::microsec_clock::universal_time() - last_time).total_milliseconds() / 1000.0; //In Seconds 
    float angle_turned = DEGREES_PER_SECOND * time_passed;
    if (last_dir.direction == "ccw") {
        angle_turned *= -1.0; // is this right for undoing the turning? - since cw is in the negative direction, flip 
    }
    else if (last_dir.direction == "fwd") {
        angle_turned = 0;
    }

    std::array<float, 3> temp = rotate_point_around_robot(new_transformed, angle_turned); // undo expected rotation

    if (last_blob.inBlob(point2dDepth(conv_to_2d(temp, camera.K.data())[0], conv_to_2d(temp, camera.K.data())[1], 0))) {
        velocity[0] = 0;
        velocity[1] = 0;
        velocity[2] = 0;
    }
    else {
        velocity[0] = (temp[0] - last_untransformed[0]) / time_passed;
        velocity[1] = (temp[1] - last_untransformed[1]) / time_passed;
        velocity[2] = (temp[2] - last_untransformed[2]) / time_passed;
    }
    ROS_INFO("Temp[0]: %f", temp[0]);
    ROS_INFO("last_untransf[0]: %f", last_untransformed[0]);
    ROS_INFO("Time passed: %f", time_passed);
    ROS_INFO("New Vel (x,y,z): [%f, %f, %f]", velocity[0], velocity[1], velocity[2]);
}

std::array<float, 3> predict_new_center() {
    double time_passed = (btt::microsec_clock::universal_time() - last_time).total_milliseconds() / 1000.0; //In Seconds   
    std::array<float, 3> new_center { {0.0, 0.0, 0.0} };
    new_center[0] = last_untransformed[0] + (velocity[0] * time_passed); //apply predicted movement 
                                                                         //of target
    new_center[1] = last_untransformed[1] + (velocity[1] * time_passed);
    new_center[2] = last_untransformed[2] + (velocity[2] * time_passed); 

    float angle_turned = DEGREES_PER_SECOND * time_passed;
    
    if (last_dir.direction.data() == "cw") {
        angle_turned *= -1.0;
    }

    return rotate_point_around_robot(new_center, angle_turned); // apply predicted movement of robot
}

void print_closest_point_pixel_coords(const depth_data& grid) {
    int out_x, out_y;
    int farthest = INT_MAX; 
    for(int x = 0; x < WIDTH; x++){
        for(int y = 0; y < HEIGHT; y++){
            if ((getXY(grid, x, y) < farthest) && (getXY(grid, x, y) != 0)){
                out_x = x;
                out_y = y;
                farthest = getXY(grid, x, y);
            }
        }
    }
    ROS_INFO("I heard: [%d, %d]", out_x, out_y);
}

void print_type_shortcuts() {
    PRINT_NAME(char);
    PRINT_NAME(signed char);
    PRINT_NAME(unsigned char);
    PRINT_NAME(short);
    PRINT_NAME(unsigned short);
    PRINT_NAME(int);
    PRINT_NAME(unsigned int);
    PRINT_NAME(long);
    PRINT_NAME(unsigned long);
    PRINT_NAME(float);
    PRINT_NAME(double);
    PRINT_NAME(long double);
    PRINT_NAME(char*);
    PRINT_NAME(const char*);
}

//Ros Functions

void callback(sensor_msgs::Image frame) {  
    depth_data grid;
    memcpy(&grid, frame.data.data(), sizeof(depth_data));
    d = grid;
    Blob<point2dDepth> b;// = Blob(grid);    
    
    std::array<int, 2> center = b.getCenter();
    std::array<float, 3> real_coords = conv_to_3d(grid, center[0], center[1], camera.K.data()); //may need to convert camera data
    
    /*
    ROS_INFO("Center At: [%d, %d]", center[0], center[1]); //For Debugging
    ROS_INFO("Area: [%d]", b.getArea());
    ROS_INFO("Real World Last (x,y,z): [%f, %f, %f]", last_untransformed[0], last_untransformed[1], last_untransformed[2]);
    ROS_INFO("Real World Real (x,y,z): [%f, %f, %f]", real_coords[0], real_coords[1], real_coords[2]);*/

    if (received_first) { // make sure we have real starting point to compare against    
        change_velocity(real_coords);
    }
    
    last_untransformed[0] = real_coords[0];
    last_untransformed[1] = real_coords[1];
    last_untransformed[2] = real_coords[2];    
    last_time = btt::microsec_clock::universal_time();
    //last_blob = b;

    if (!received_first) {
        received_first = true;   
    }
    else if (!received_2) {
        received_2 = true;
    }
}

/*
int main(int argc, char **argv)
{

  ros::init(argc, argv, "predictive_director");

  ros::NodeHandle n;

  ros::Subscriber temp = n.subscribe("/openni2_camera/depth/camera_info", 1000, set_camera_info);

  while(&camera == NULL) {//wait until we have camera information
    continue;  
  }

  temp.~Subscriber();

  ros::Subscriber sub = n.subscribe("/openni2_camera/depth/image_raw", 1000, callback);
  ros::Publisher dir_pub = n.advertise<basic_teleop::Move>("movement", 1000);

  ros::Rate loop_rate(10); 
    
  last_dir.direction = "fwd";

  while (ros::ok())
  {
    
    while(!received_2) { // wait until we have valid velocity data
        ros::spinOnce();

        loop_rate.sleep();
    } 

    ROS_INFO("Last pub-ed: [%s]", last_dir.direction.data());  

    std::array<float, 3> new_center = predict_new_center();    

    basic_teleop::Move next;

    std::array<int, 2> temp_2d = conv_to_2d(new_center, camera.K.data());

    if(temp_2d[0] >= 130 && temp_2d[0] <= 190) {
        next.direction = "fwd";
    }
    else if(temp_2d[0] < 130){ // to the right of the robot is lower values
        next.direction = "cw";
    }
    else {
        next.direction = "ccw";
    }
    
    dir_pub.publish(next);
    last_dir = next;  
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
    
  return 0;
}*/

int main(int argc, char **argv)
{ 
  //  ros::init(argc, argv, "predictive_director");
    ros::Time::init();
  //ros::NodeHandle n;

  //ros::Subscriber temp = n.subscribe("/openni2_camera/depth/camera_info", 1000, set_camera_info);

  //while(&camera == NULL) {//wait until we have camera information
  //  continue;  
  //}

  //temp.~Subscriber();

  //ros::Subscriber sub = n.subscribe("/openni2_camera/depth/image_raw", 1000, callback);
  //ros::Publisher dir_pub = n.advertise<basic_teleop::Move>("movement", 1000);

  ros::Rate loop_rate(.1); 
    
  last_dir.direction = "fwd";
    
    /*while(!received_2) { // wait until we have valid velocity data
        ros::spinOnce();

        loop_rate.sleep();
    }*/
    
    // Fake Image for debugging purposes
    d.mm[100] = 4500;
    d.mm[200] = 5400;
    d.mm[300] = 6300;
    d.mm[400] = 7200;
    d.mm[500] = 8100;
    d.mm[600] = 9000;
    d.mm[700] = 9900;
    d.mm[800] = 10800;
    d.mm[900] = 11700;
    loop_rate.sleep();
    printFrameWithBlobs(blobFrame(d));    

};
