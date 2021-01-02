#ifndef SENSOR_BEARING_H
#define SENSOR_BEARING_H

#include <iostream>
#include <vector>
#include <geom.h>
#include <sensor.h>
#include <robot.h>
#include <math.h>
#include <limits>

namespace arpro
{
class BearingSensor : public Sensor
{
public : BearingSensor ( Robot & _robot , double _x , double _y , double _theta ) :
            Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
{}


    void update(const Pose &_p)
    {
        auto p = _p.transformInverse(pose_);
        auto angle_helper = 0;
        // look for first other robot
        for ( auto other : envir_ -> robots_ )
            if ( other != robot_ )
            {
                std::cout << _p.x << std::endl;
                std::cout << _p.y << std::endl;
                std::cout << _p.theta << std::endl;

                // Angle between the bearing sensor and the detected robot
                angle_helper = atan2 ((other->pose().y - _p.y), (other->pose().x - _p.x)) - _p.theta;
                break ;
            }
        if (angle_helper < -M_PI)
            angle_helper += 2*M_PI;
        else if (angle_helper > M_PI)
            angle_helper -= 2*M_PI;

        s_ = angle_helper;
        s_history_.push_back((angle_helper));

        std::cout << "s_ = " << s_ << std::endl;

    }

    void correctTwist(Twist &_v)
    {
        auto g = 0.5;
        _v.w = _v.w - g*s_;
        std::cout << "_v.w = " << _v.w << std::endl;
    }

};

}
#endif // SENSOR_BEARING_H
