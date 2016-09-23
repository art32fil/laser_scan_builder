#include <iostream>
#include <fstream>
#include <vector>
#include "core/sensor_data.h"
#include "core/geometry_utils.h"
#include "core/state_data.h"
#include <math.h>
#include <limits>

using namespace std;

struct PolarPoint {
  double ro;
  double fi;
};

struct DoublePoint {
  double x;
  double y;
};

using Point = DoublePoint;


class Line {
  Point p1;
  Point p2;
public:
  Line(const Point& p1, const Point& p2) : p1(p1), p2(p2) {}

  Point& get_p1() { return p1; }
  const Point& get_p1() const { return p1; }
  Point& get_p2() { return p2; }
  const Point& get_p2() const { return p2; }
};

class Ray : public Line {
public:
  Ray(const Point& p1, const Point& p2) : Line(p1, p2) {}
};
class Intersect : public Ray {
public:
  Intersect(const Point& p1, const Point& p2) : Ray(p1, p2) {}
};

template <typename T>
T det(T a, T b, T c, T d) {
  return a*d - b*c;
}

bool cross(const Line& l1, const Line& l2, Point& out) {
  double a1,b1,c1, a2,b2,c2; //a1*x+b1*y+c1=0 & a2*x+b2*x+c2=0
  a1 = l1.get_p1().y - l1.get_p2().y;
  b1 = l1.get_p2().x - l1.get_p1().x;
  c1 = det(l1.get_p1().x, l1.get_p1().y, l1.get_p2().x, l1.get_p2().y);

  a2 = l2.get_p1().y - l2.get_p2().y;
  b2 = l2.get_p2().x - l2.get_p1().x;
  c2 = det(l2.get_p1().x, l2.get_p1().y, l2.get_p2().x, l2.get_p2().y);

  if (EQ_DOUBLE(a1*b2 - a2*b1, 0.0)) {
    out.x = 0;
    out.y = 0;
    return false;
  }

  out.x = -det(c1,b1,c2,b2) / det(a1,b1,a2,b2);
  out.y = -det(a1,c1,a2,c2) / det(a1,b1,a2,b2);
  return true;
}

bool cross(const Intersect& i1, const Line& l2, Point& out) {
  if (!cross(Line{i1.get_p1(), i1.get_p2()}, l2, out)) {
    return false;
  }
  if (EQ_DOUBLE(abs(i1.get_p1().x - out.x) + abs(i1.get_p2().x - out.x),
           abs(i1.get_p1().x - i1.get_p2().x)) &&
      EQ_DOUBLE(abs(i1.get_p1().y - out.y) + abs(i1.get_p2().y - out.y),
           abs(i1.get_p1().y - i1.get_p2().y)))
    return true;
  out.x = 0;
  out.y = 0;
  return false;
}

bool cross(const Line& l1, const Intersect& i2, Point& out) {
  return cross(i2, l1, out);
}

bool cross(const Intersect& i1, const Ray& r2, Point& out) {
  if (!cross(i1, Line{r2.get_p1(), r2.get_p2()}, out))
    return false;
  if (0 < (out.x - r2.get_p1().x)*(r2.get_p2().x - r2.get_p1().x))
    return true;
  out.x = 0;
  out.y = 0;
  return false;
}
bool cross(const Ray& r1, const Intersect& i2, Point& out){
  return cross(i2, r1, out);
}

double dist_sq(const Point& p1, const Point& p2) {
  return pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0);
}

class Object {
  vector<Point> ptrs;
public:

  Object(vector<Point> ptrs) : ptrs(ptrs) {}

  vector<Point>& get_points() { return ptrs; }
  const vector<Point>& get_points() const { return ptrs; }

  bool cross(const Ray& view, Point& out) const{
    bool found_cross = false;
    double min_range = numeric_limits<double>::max();
    for (size_t i = 0; i < ptrs.size()-1; i++) {
      Intersect current_wall{ptrs[i], ptrs[i+1]};
      Point current_intersect;
      if (::cross(current_wall, view, current_intersect)) {
        found_cross = true;
        double current_range = dist_sq(current_intersect, view.get_p1());
        if (current_range < min_range) {
          out = current_intersect;
          min_range = current_range;
        }
      }
    }
    return found_cross;
  }
};

static RobotState current_pose{0,0,0};
static RobotState prev_pose{0,0,0};
static vector<Object> world = { // to create a vector of objects
                                { // to create an object
                                  { // to create a vector of points
                                    {10, 10}, // to create point
                                    {20, 10},
                                    {20, 20},
                                    {10, 20}
                                  }
                                }
                              };
double angle_min = -M_PI;
double angle_max = M_PI;
double angle_inc = M_PI/180;

void generate_laser_scan(TransformedLaserScan& out_scan) {
  out_scan.points.clear();
  out_scan.d_x = current_pose.x - prev_pose.x;
  out_scan.d_y = current_pose.y - prev_pose.y;
  out_scan.d_yaw = current_pose.theta - prev_pose.theta;

  for (double angle = angle_min + current_pose.theta; angle < angle_max + current_pose.theta; angle += angle_inc) {
    Ray current_view{{current_pose.x, current_pose.y},
                     {current_pose.x+100.0*cos(angle),
                      current_pose.y+100.0*sin(angle)}};
    double min_dist = numeric_limits<double>::max();
    bool find_cross = false;
    for (size_t i = 0; i < world.size(); i++) {
      Point cross_point;
      if (world[i].cross(current_view, cross_point)) {
        find_cross = true;
        double dist = dist_sq({current_pose.x,current_pose.y},cross_point);
        if (dist < min_dist) {
          min_dist = dist;
        }
      }
    }
    if (find_cross)
      out_scan.points.push_back({sqrt(min_dist), angle, true});
    else
      out_scan.points.push_back({1000, angle, false});
  }
}

void step (double dx, double dy, double d_yaw) {
  prev_pose = current_pose;
  current_pose.x += dx;
  current_pose.y += dy;
  current_pose.theta += d_yaw;
  /*for (auto& obj : world) {
    for (auto& pt : obj.get_points()) {
      pt.x -= dx;
      pt.y -= dy;
      double x = pt.x;
      double y = pt.y;
      pt.x = cos(-d_yaw)*x - sin(-d_yaw)*y;
      pt.y = sin(-d_yaw)*x + cos(-d_yaw)*y;
    }
  }*/
}

void put_scan (ostream& stream, TransformedLaserScan& scan) {
  stream << angle_min << " " << angle_max << " " << angle_inc << " "
         << scan.d_x << " " << scan.d_y << " " << scan.d_yaw;
  for (size_t i = 0; i < scan.points.size(); i++) {
    stream  << " " << scan.points[i].range << " " << scan.points[i].is_occupied;
  }
  stream << endl;
  return;
}

int main(int argc, char** argv){

  TransformedLaserScan scan;
  ofstream out ("base_scan.txt");


  generate_laser_scan(scan);
  put_scan(out, scan);

  step (0.,5,60.0/180.0*M_PI);

  generate_laser_scan(scan);
  put_scan(out, scan);

  out.close();
  return 0;
}
