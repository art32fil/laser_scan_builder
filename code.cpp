#include <iostream>
#include <vector>
#include "core/sensor_data.h"
#include "core/geometry_utils.h"
#include <math.h>
#include <limits>

using namespace std;

#define EQ_D(x,y) \
  (std::abs((x)-(y)) <= std::numeric_limits<double>::epsilon()* \
                        std::max(std::abs(x), std::abs(y)))

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

  if (EQ_D(a1*b2 - a2*b1, 0.0)) {
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
  if (EQ_D(abs(i1.get_p1().x - out.x) + abs(i1.get_p2().x - out.x),
           abs(i1.get_p1().x - i1.get_p2().x)) &&
      EQ_D(abs(i1.get_p1().y - out.y) + abs(i1.get_p2().y - out.y),
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

  bool cross(const Ray& view, Point& out) {
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

int main(int argc, char** argv){

  vector<Object> world = {{{{100, 100}, {200, 100}, {200, 200}, {100, 200}}}};
  // the world was generated (strip line through these points (isn't looped))
  vector<Point> laser_scan;
  // there will be points in Euclidean coordinates
  vector<PolarPoint> polar_laser_scan;
  // there will be points in polar coordinates
  for (double angle = -M_PI; angle < M_PI; angle += M_PI/180) {
    Intersect current_view{{0, 0}, {1000*cos(angle), 1000*sin(angle)}};
    for (auto& obj : world) {
      // go on every object in the world
      Point cross_point;
      if (obj.cross(current_view, cross_point)) {
        // and try to find intersection with every object
        laser_scan.push_back(cross_point);
        polar_laser_scan.push_back({sqrt(pow(cross_point.x, 2.0)+
                                         pow(cross_point.y, 2.0)), angle});
      }
    }
  }
  // after that display all intersect point coordinates (in Euclidean and polar)
  int i = 0;
  for (auto& pt : laser_scan) {
    double x = polar_laser_scan[i].ro*cos(polar_laser_scan[i].fi);
    double y = polar_laser_scan[i].ro*sin(polar_laser_scan[i].fi);
    cout << pt.x << " " << pt.y << "\t| "
         << x << " " << y << "\t| "
         << pt.x - x << " " << pt.y - y << endl;
    i++;
  }
  return 0;
}
