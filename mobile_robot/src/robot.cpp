

#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;

//Question 2
//In Robot::Robot the first Robot is a class and the second one is a constructor. Receives Arguments _name, _x, _y and _theta to define a new robot
// yes we cand define a new robot. For this I have define a robot2 "bb8"" in main.cpp
Robot::Robot(string _name, double _x, double _y, double _theta)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);
}


void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}

void Robot::initWheels(double r,double b,double Omegamax)
{
    radius=r;
    distance=b;
    velocitymax=Omegamax;
    init_wheels = true;
}


void Robot::rotateWheels(double _left, double _right)
{
    // to fill up after defining an initWheel method

    if(init_wheels==true)
        {
           double _v=(radius*(_left+_right)/2);
           double _w=(radius*(_left)-_right)/(distance*2);

            double x_,y_,t_;
            x_=_v*cos(pose_.theta);
            y_=_v*sin(pose_.theta);
            t_=_w;
            moveXYT(x_,y_,t_);

           double a;
           a = max((abs(_left))/velocitymax,(abs(_right))/velocitymax);
           if(a<1)
           {
               a=1;
           }
           else
           {
           std::cout<<"velocity setpoint is high";
           _left=_left/a;
           _right=_right/a;
            }
        }
}


// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    double _left,_right;
    _left=(_v+distance*_omega)/radius;
    _right=(_v-distance*_omega)/radius;
    rotateWheels(_left,_right);
}




// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}


void Robot::moveWithSensor(Twist _twist)
{
    for ( auto & sensor : sensors_){
           sensor->updateFromRobotPose(this->pose_);
           sensor->correctRobotTwist(_twist);
       }
    int alpha=20;
    moveVW(_twist.vx,alpha*_twist.vy + _twist.w);

    // uses X-Y motion (perfect but impossible in practice)
    // moveXYT(_twist.vx, _twist.vy,_twist.w);  
}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

