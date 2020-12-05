#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include <sensor.h>
#include <string>
#include <envir.h>
#include <robot.h>
#include <algorithm>
#include <geom.h>

namespace arpro
{
class RangeSensor : public Sensor
{
public:
    RangeSensor(Robot &_robot, double _x, double _y, double _theta):
        Sensor(_robot, _x, _y, _theta)
        {}

void correctTwist(Twist &_v)
{
    if (_v.vx > 0.1 * (s_ - 0.1))
        _v.vx = 0.1 * (s_ - 0.1);
}
void update(const Pose &_p)
{
    Pose p1, p2;
    std::vector<double> d;
    for (int i = 0; i < envir_-> walls.size(); i++)
    {
        p1 = envir_ -> walls[i];
        p2 = envir_ -> walls[(i + 1) % envir_ -> walls.size()];
        d.push_back(std::abs((p1.x * p2.y - p1.x * _p.y - p2.x * p1.y +
                     p2.x * _p.y + _p.x * p1.y - _p.x *p2.y)/(
                     p1.x * sin(_p.theta) - p2.x * sin(_p.theta) -
                     p1.y * cos(_p.theta) +  p2.y * cos(_p.theta))));
    }
    s_ = *min_element(d.begin(),d.end());
    cout << "closest distance to wall: " << s_ << endl;
}

};
}
#endif
