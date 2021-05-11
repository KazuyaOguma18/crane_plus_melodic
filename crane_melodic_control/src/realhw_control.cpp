#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <crane_melodic_control/realhw.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hw_crane_plus");

    CranePlus craneplus;
    controller_manager::ControllerManager cm(&craneplus, craneplus.nh);

    ros::Rate rate(1 / craneplus.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);

    spinner.start();

    while (ros::ok())
    {
        ros::Time now = craneplus.getTime();
        ros::Duration dt = craneplus.getPeriod();

        craneplus.read(now, dt);
        cm.update(now, dt);

        craneplus.write(now, dt);
        rate.sleep();
    }
    spinner.stop();

    return 0;
}