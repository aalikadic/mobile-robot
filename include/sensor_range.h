#ifndef SENSOR_RANGE_H
#define SENSOR_RANGE_H

#include <iostream>
#include <vector>
#include <geom.h>
#include <sensor.h>
#include <robot.h>
#include <limits>


namespace arpro
{
class RangeSensor : public Sensor
{
public :
    RangeSensor ( Robot & _robot , double _x , double _y , double _theta ) :
         Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
{}


    void update(const Pose &_p)
    {
        double nearest_wall = std::numeric_limits<double>::max();
        double distance_helper;
        double distance_helper_denominator;

        Pose p1, p2;

        for ( int i=0; i< envir_->walls.size(); ++i)
        {
            p1 = envir_ -> walls[i];
            p2 = envir_ -> walls[(i+1)%envir_ -> walls.size()];
            //calculate the distance
            distance_helper_denominator = (p1.x * sin(_p.theta)) - (p2.x * sin(_p.theta)) - (p1.y * cos(_p.theta)) + (p2.y * cos(_p.theta));
            if(distance_helper_denominator != 0)
            {
                distance_helper =  ((p1.x * p2.y) - (p1.x * _p.y) - (p2.x * p1.y) + (p2.x * _p.y) + (_p.x * p1.y) - (_p.x * p2.y)) / (distance_helper_denominator);
                cout << "Distance to wall " << i << ": " << distance_helper << endl;

                // check if closest
                if (distance_helper < nearest_wall)
                    nearest_wall = distance_helper;
            }

        }
        cout<<"Closest distance is: "<<nearest_wall<<endl;

        s_ = nearest_wall;
        s_history_.push_back((nearest_wall));

    }

    void correctTwist(Twist &_v)
    {
        auto s_min = 0.1;
        auto g = 0.1;

        if (_v.vx > g*(s_ - s_min))
        {
            _v.vx = g*(s_ - s_min);
        }
    }

};
}
#endif // SENSOR_RANGE_H
