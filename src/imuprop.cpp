#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

#include "utility.h"

#include <tf/transform_broadcaster.h>

// Mutual exclusive, some kind of a software flag for ownership
#include <mutex>

// Constant gravity vector
Vector3d GRAV(0, 0, 9.82);

ros::Subscriber odom_sub;
ros::Subscriber imu_sub;

// Save the odometry sample
nav_msgs::Odometry odom;

std::mutex imu_prop_mtx;
std::shared_ptr<ImuProp> imu_prop = nullptr;

ros::NodeHandlePtr nh_ptr;

void odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Do something here
    printf(KYEL "Receiving a msg at time: %f\n" RESET, msg->header.stamp.toSec());
    odom = *msg;

    // Extract the time
    double t_odom = msg->header.stamp.toSec();

    // Extract robot states
    
    // Orientation states
    Quaternd orientation(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    // Position
    Vector3d position(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    // Velocity  
    Vector3d velocity(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);

    // Angular velocity
    Vector3d angular_velocity(msg->twist.twist.angular.x,
                              msg->twist.twist.angular.y,
                              msg->twist.twist.angular.z);

    // Acceleration
    Vector3d linear_acceleration(msg->twist.covariance[0],
                                 msg->twist.covariance[1],
                                 msg->twist.covariance[2]);
    // Imu bias
    Vector3d bias_g(msg->twist.covariance[3],
                    msg->twist.covariance[4],
                    msg->twist.covariance[5]);

    Vector3d bias_a(msg->twist.covariance[6],
                    msg->twist.covariance[7],
                    msg->twist.covariance[8]);

    std::shared_ptr<ImuProp> imu_prop_new(new ImuProp(orientation, position, velocity, bias_g, bias_a,
                                                      angular_velocity, linear_acceleration, GRAV, t_odom)); // gets u a *ImuProp

    imu_prop_mtx.lock();

    if (imu_prop != nullptr)
    {
        for(int idx = 0; idx < imu_prop->size(); idx++)
        {
            if(imu_prop->t[idx] < imu_prop_new->t.front())
                continue;
            else
            {
                imu_prop_new->forwardPropagate(imu_prop->gyr[idx],
                                               imu_prop->acc[idx],
                                               imu_prop->t[idx]);
            }
        }
    }

    imu_prop = imu_prop_new;
            
    imu_prop_mtx.unlock();

    static tf::TransformBroadcaster tfbr;
    
    tf::Transform tf_now;
    tf_now.setOrigin(tf::Vector3(position(0), position(1), position(2)));
    tf_now.setRotation(tf::Quaternion(orientation.w(), orientation.x(), orientation.y(), orientation.z()));
    tfbr.sendTransform(tf::StampedTransform(tf_now, ros::Time::now(),
                       odom.header.frame_id, "body_10Hz"));
}

void imuCB(const sensor_msgs::Imu::ConstPtr &msg)
{
    printf(KRED "Receving an IMU msg at time %f\n" RESET, msg->header.stamp.toSec());

    if(imu_prop == nullptr)
        return;

    if (msg->header.stamp.toSec() <= imu_prop->t.back())
            return;

    // Extract imu data
    imu_prop_mtx.lock();
    
    imu_prop->forwardPropagate(msg);

    mytf tf_W_B = imu_prop->getBackTf();
    double t_prop = imu_prop->t.back();

    imu_prop_mtx.unlock();

    static ros::Publisher imu_prop_pub = nh_ptr->advertise<nav_msgs::Odometry>("/prop_odom", 10);

    nav_msgs::Odometry prop_odom;
    
    prop_odom.header.stamp = ros::Time(t_prop);
    prop_odom.header.frame_id = odom.header.frame_id;
    prop_odom.child_frame_id = "body_400Hz";

    prop_odom.pose.pose.position.x = tf_W_B.pos(0);
    prop_odom.pose.pose.position.y = tf_W_B.pos(1);
    prop_odom.pose.pose.position.z = tf_W_B.pos(2);

    prop_odom.pose.pose.orientation.x = tf_W_B.rot.x();
    prop_odom.pose.pose.orientation.y = tf_W_B.rot.y();
    prop_odom.pose.pose.orientation.z = tf_W_B.rot.z();
    prop_odom.pose.pose.orientation.w = tf_W_B.rot.w();

    imu_prop_pub.publish(prop_odom);

    static tf::TransformBroadcaster tfbr;
    
    tf::Transform tf_now;
    tf_now.setOrigin(tf::Vector3(tf_W_B.pos(0), tf_W_B.pos(1), tf_W_B.pos(2)));
    tf_now.setRotation(tf::Quaternion(tf_W_B.rot.w(), tf_W_B.rot.x(), tf_W_B.rot.y(), tf_W_B.rot.z()));
    tfbr.sendTransform(tf::StampedTransform(tf_now, ros::Time::now(),
                       odom.header.frame_id, "body_400Hz"));

}

int main(int argc, char **argv)
{
    // Initializing the node with roscore
    ros::init(argc, argv, "ImuOdom");

    // Create a handle, ~ means using local namespaces for sub and pub
    ros::NodeHandle nh("~");

    nh_ptr = boost::make_shared<ros::NodeHandle>(nh);
    
    // Some trivial notification
    printf("Hello world. Coding starts.\n");

    // Subcribe to the opt_odom
    odom_sub = nh.subscribe("/opt_odom", 100, odomCB);

    // Subcribe to the opt_odom
    imu_sub = nh.subscribe("/vn100/imu", 100, imuCB);

    // Spin, leave the callbacks to run on their own
    ros::spin();
}
