#ifndef QBHAND_FORCE_CONTROL_H
#define QBHAND_FORCE_CONTROL_H

#include <qbInterface.h>
#include <ros/ros.h>
#include <qb_interface/handRef.h>
#include <qb_interface/handPos.h>
#include <vector>
#include <qbHand.h>

#define ERR_TIMEOUT		5

class qb_force{

public:
    qb_force();
    void spinOnce();
    void spin();
    ~qb_force();

protected:
    ros::NodeHandle* node_;
    comm_settings* qb_comm_;
    double step_time_,stiffness_;
    ros::Subscriber hand_sub;
    ros::Publisher hand_pub,debug_pub;
    void handRefCallback(const qb_interface::handRef::ConstPtr&);
    std::vector<qbHand*> hand_chain_;
    std::vector<float> pos_,cur_,pos_ref_;

    bool open(const char*, const int);
    bool close();
    bool activate();
    bool deactivate();
    bool readMeasCurrent();
    void move(float &debug_cur);
    float speed_,damping_;
    float calc_current(float set_pos,float cur_pos,float stiffness, float speed, float damping,float deadband);
    };
#endif
