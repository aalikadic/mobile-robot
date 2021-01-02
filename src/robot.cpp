#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;

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
void Robot::initWheel(double _radius, double _base, double _omega_limit)
{
    radius_ = _radius;
    base_ = _base;
    omega_limit_ = _omega_limit;

    wheels_init_ = true;
};

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


void Robot::rotateWheels(double _left, double _right)
{
    // check if wheels have been initialized
    if(wheels_init_)
    {
        auto w_max = omega_limit_;
        //Calculate the scaling factor. If the scaling factor is below 1, it means
        // that the wr and wl are within the bouds
        auto a = max((abs(_left)/w_max),abs((_right)/w_max));

        // Check if a is smaller than 1
        if (a < 1)
        {
            a = 1;
        }

        // Scale both of the velocities using that factor
        auto left = _left/a;
        auto right = _right/a;
        // Print out that the initial velocities had to be scaled
        std::cout<<"The initial wheel velocities were not in the boundaries and therefore had to be scaled"<<endl;

        auto v = (radius_ * (left + right)/2);
        auto omega = (radius_ * (right-left)/(2*base_));

        auto vx = v*cos(pose_.theta);
        auto vy = v*sin(pose_.theta);

        moveXYT(vx, vy, omega);
    }
}


// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    // This was the initial moveVM
    // For this part of the code to work, we need to calculate the values of theta, x and y.
    // Theta can we directly calculated as the multiplication of time and omega (I must check the unit).
    // We calculate the x,y using v and cos/sin of theta.
    // double _theta;
    // double x;
    // double y;

    // _theta = _omega*dt_;
    // x = _v*cos(_theta);
    // y = _v*sin(_theta);

    // Robot::moveXYT(x, y, _omega);
    auto _left = (_v + ( base_ *_omega ))/radius_;
    auto _right = (_v - (base_ * _omega ))/radius_;

    rotateWheels(_left, _right);

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
    // to fill up, sensor measurement and twist checking

    for ( auto & sensor : sensors_ )
    {
        sensor ->updateFromRobotPose( pose_ );
        sensor -> correctRobotTwist((_twist));
    }
    // uses X-Y motion (perfect but impossible in practice)
    //  moveXYT(_twist.vx, _twist.vy,_twist.w);


    double alpha = 20;
    auto v  = _twist.vx;
    auto omega = _twist.vy*alpha + _twist.w;

    moveVW(v, omega);
}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

