
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <random>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


// Color definitions
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

using namespace std;
using namespace Eigen;

typedef sensor_msgs::Imu::ConstPtr RosImuPtr;
typedef Eigen::Quaterniond Quaternd;
// typedef Eigen::Quaternionf Quaternf;

template <typename T=double>
struct myTf
{
    Eigen::Quaternion<T>   rot;
    Eigen::Matrix<T, 3, 1> pos;

    myTf Identity()
    {
        return myTf();
    }
    
    myTf(const myTf<T> &other)
    {
        rot = other.rot;
        pos = other.pos;
    }

    myTf()
    {
        rot = Quaternd(1, 0, 0, 0);
        pos = Vector3d(0, 0, 0);
    }

    myTf(Eigen::Quaternion<T> rot_in, Eigen::Matrix<T, 3, 1> pos_in)
    {
        this->rot = rot_in;
        this->pos = pos_in;
    }

    myTf(Eigen::Matrix<T, 3, 3> rot_in, Eigen::Matrix<T, 3, 1> pos_in)
    {
        this->rot = Quaternion<T>(rot_in);
        this->pos = pos_in;
    }

    myTf(Eigen::Matrix<T, 3, 1> axisangle_in, Eigen::Matrix<T, 3, 1> pos_in)
    {
        this->rot = Quaternd(Eigen::AngleAxis<T>(axisangle_in.norm(),
                                                 axisangle_in/axisangle_in.norm()));
        this->pos = pos_in;
    }

    template <typename Tin>
    myTf(Eigen::Matrix<Tin, 4, 4> tfMat)
    {
        Eigen::Matrix<T, 3, 3> M = tfMat.block(0, 0, 3, 3).template cast<T>();
        this->rot = Quaternion<T>(M);
        this->pos = tfMat.block(0, 3, 3, 1).template cast<T>();
    }


    myTf(const nav_msgs::Odometry &odom)
    {
        this->rot = Quaternion<T>(odom.pose.pose.orientation.w,
                                    odom.pose.pose.orientation.x,
                                    odom.pose.pose.orientation.y,
                                    odom.pose.pose.orientation.z);
                                    
        this->pos << odom.pose.pose.position.x,
                     odom.pose.pose.position.y,
                     odom.pose.pose.position.z;
    }

    myTf(const geometry_msgs::PoseStamped &pose)
    {
        this->rot = Quaternion<T>(pose.pose.orientation.w,
                                  pose.pose.orientation.x,
                                  pose.pose.orientation.y,
                                  pose.pose.orientation.z);
                                    
        this->pos << pose.pose.position.x,
                     pose.pose.position.y,
                     pose.pose.position.z;
    }

    myTf(Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform)
    {
        this->rot = Eigen::Quaternion<T>{transform.linear()}.normalized();
        this->pos = transform.translation();
    }

    Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform() const
    {
        Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform;
        transform.linear() = rot.normalized().toRotationMatrix();
        transform.translation() = pos;
        return transform;
    }

    Eigen::Matrix<T, 4, 4> tfMat() const
    {
        Eigen::Matrix<T, 4, 4> M = Matrix<T, 4, 4>::Identity();
        M.block(0, 0, 3, 3) = rot.normalized().toRotationMatrix();
        M.block(0, 3, 3, 1) = pos;
        return M;
    }

    double roll() const
    {
        return atan2(rot.x()*rot.w() + rot.y()*rot.z(), 0.5 - (rot.x()*rot.x() + rot.y()*rot.y()))/M_PI*180.0;
    }

    double pitch() const
    {
        return asin(-2*(rot.x()*rot.z() - rot.w()*rot.y()))/M_PI*180.0;
    }

    double yaw() const
    {
        return atan2(rot.x()*rot.y() + rot.w()*rot.z(), 0.5 - (rot.y()*rot.y() + rot.z()*rot.z()))/M_PI*180.0;
    }

    Matrix<T, 3, 1> SO3Log()
    {
        Eigen::AngleAxis<T> phi(rot);
        return phi.angle()*phi.axis();
    }

    myTf inverse() const
    {
        Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform_inv = this->transform().inverse();
        myTf tf_inv;
        tf_inv.rot = transform_inv.linear();
        tf_inv.pos = transform_inv.translation();
        return tf_inv;
    }

    myTf operator*(const myTf &other) const
    {
        Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform_out = this->transform() * other.transform();
        return myTf(transform_out);
    }

    Vector3d operator*(const Vector3d &v) const
    {
        return (rot*v + pos);
    }

    template <typename NewType>
    myTf<NewType> cast() const
    {
        myTf<NewType> tf_new{this->rot.template cast<NewType>(), this->pos.template cast<NewType>()};
        return tf_new;
    }

    friend std::ostream &operator<<(std::ostream &os, const myTf &tf)
    {
        os << tf.pos.x() << " " << tf.pos.y() << " " << tf.pos.z() << " " << tf.rot.w() << " "
           << tf.rot.x() << " " << tf.rot.y() << " " << tf.rot.z();
        return os;
    }
}; // class myTf

typedef myTf<> mytf;

class ImuProp
{
public:

    deque<double> t;

    deque<Quaternd> Q;
    deque<Vector3d> P;
    deque<Vector3d> V;

    Vector3d bg; Vector3d ba; Vector3d grav;
    
    deque<Vector3d> gyr;
    deque<Vector3d> acc;

    ~ImuProp() {}

    ImuProp()
    {
        t = { 0 };

        Q = { Quaternd(1, 0, 0, 0) };
        P = { Vector3d(0, 0, 0)    } ;
        V = { Vector3d(0, 0, 0)    };

        bg = Vector3d(0, 0, 0);
        ba = Vector3d(0, 0, 0);

        gyr = { Vector3d(0, 0, 0) };
        acc = { Vector3d(0, 0, 0) };

        grav = Vector3d(0, 0, 0);
    }

    ImuProp(Quaternd Q0,  Vector3d P0,  Vector3d V0,
            Vector3d bg_, Vector3d ba_, Vector3d gyr0, Vector3d acc0, Vector3d grav_, double t0)
    {
        reset(Q0, P0, V0, bg_, ba_, gyr0, acc0, grav_, t0);
    }

    void reset(Quaternd &Q0, Vector3d &P0, Vector3d &V0, Vector3d &bg_, Vector3d &ba_, Vector3d &gyr0, Vector3d &acc0, Vector3d &grav_, double t0)
    {
        t = { t0 };
        
        Q = { Q0 };
        P = { P0 };
        V = { V0 };

        gyr = { gyr0 };
        acc = { acc0 };

        bg = bg_; ba = ba_; grav = grav_;
    }

    void forwardPropagate(const RosImuPtr &msg)
    {
        Vector3d gyrn(msg->angular_velocity.x,
                      msg->angular_velocity.y,
                      msg->angular_velocity.z);

        Vector3d accn(msg->linear_acceleration.x,
                      msg->linear_acceleration.y,
                      msg->linear_acceleration.z);

        double tn = msg->header.stamp.toSec();

        forwardPropagate(gyrn, accn, tn);
    }

    void forwardPropagate(Vector3d &gyrn, Vector3d &accn, double tn)
    {
        double to = t.back();

        Quaternd Qo = Q.back();
        Vector3d Po = P.back();
        Vector3d Vo = V.back();

        Vector3d gyro = gyr.back();
        Vector3d acco = acc.back();

        // Time step
        double dt = tn - to;

        // Orientation
        Vector3d un_gyr = 0.5 * (gyro + gyrn) - bg;
        Quaternd Qn = Qo * QExp(un_gyr * dt);

        // Position
        Vector3d un_acco = Qo * (acco - ba) - grav;
        Vector3d un_accn = Qn * (accn - ba) - grav;
        Vector3d un_acc  = 0.5 * (un_acco + un_accn);

        Vector3d Pn = Po + dt * Vo + 0.5 * dt * dt * un_acc;
        Vector3d Vn = Vo + dt * un_acc;

        // Store the data
        t.push_back(tn);

        Q.push_back(Qn);
        P.push_back(Pn);
        V.push_back(Vn);

        gyr.push_back(gyrn);
        acc.push_back(accn);
    }

    inline Quaternd QExp(const Vector3d &theta)
    {
        Quaternd dq;

        double theta_nrm = theta.norm();

        if (theta_nrm < 1e-9)
        {
            Vector3d half_theta = theta;
            half_theta /= 2.0;
            dq.w() = 1.0;
            dq.x() = half_theta.x();
            dq.y() = half_theta.y();
            dq.z() = half_theta.z();
        }
        else
        {
            double costheta = cos(theta_nrm / 2);
            double sintheta = sin(theta_nrm / 2);
            Vector3d quat_vec = theta / theta_nrm * sintheta;

            dq.w() = costheta;
            dq.vec() = quat_vec;
        }

        // printf("dq: %f, %f, %f, %f. norm: %f\n", dq.x(), dq.y(), dq.z(), dq.w(), dq.norm());

        return dq;
    }

    void backwardPropagate(Vector3d &gyro, Vector3d &acco, double to)
    {
        double tn = t.front();

        Quaternd Qn = Q.front();
        Vector3d Pn = P.front();
        Vector3d Vn = V.front();

        Vector3d gyrn = gyr.front();
        Vector3d accn = acc.front();

        // Time step
        double dt = tn - to;

        // Orientation
        Vector3d un_gyr = 0.5 * (gyro + gyrn) - bg;
        Quaternd Qo = Qn * QExp(-un_gyr * dt);

        // Position
        Vector3d un_acco = Qo * (acco - ba) - grav;
        Vector3d un_accn = Qn * (accn - ba) - grav;
        Vector3d un_acc  = 0.5 * (un_acco + un_accn);

        Vector3d Vo = Vn - dt * un_acc;
        Vector3d Po = Pn - dt * Vo - 0.5 * dt * dt * un_acc;

        // Store the data
        t.push_front(to);

        Q.push_front(Qo);
        P.push_front(Po);
        V.push_front(Vo);

        gyr.push_front(gyro);
        acc.push_front(acco);
    }

    mytf getFrontTf()
    {
        return (mytf(Q.front(), P.front()));
    }

    mytf getBackTf() const
    {
        return (mytf(Q.back(), P.back()));
    }

    mytf getTf(double ts) const
    {
        ROS_ASSERT(t.size() >= 1);

        if (ts < t.front())
        {
            // if ( fabs(ts - t.front()) > 5e-3 )
                printf(KYEL "Point time %.6f is earlier than [%.6f, %.6f]. "
                            "Returning pose at start time.\n" RESET, ts, t.front(), t.back());

            return (mytf(Q.front(), P.front()));
        }
        else if (ts > t.back())
        {
            // if ( fabs(ts - time.back()) > 5e-3 )
                printf(KYEL "Point time %.6f is later than [%.6f, %.6f]. "
                            "Returning pose at end time.\n" RESET, ts, t.front(), t.back());
            return (mytf(Q.back(), P.back()));
        }
        else
        {
            // Find the pose that fit the time
            for(int i = 0; i < t.size() - 1; i++)
            {
                if( t[i] <= ts && ts <= t[i+1] )
                {
                    double s = (ts - t[i])/(t[i+1] - t[i]);
                    Quaternd Qs = Q[i]*Quaternd::Identity().slerp(s, Q[i].inverse()*Q[i+1]);
                    Vector3d Ps = (1 - s)*P[i] + s*P[i+1];

                    return (mytf(Qs, Ps));   
                }
            }
        }
    }

    double getStartTime()
    {
        return t.front();
    }

    double getEndTime()
    {
        return t.back();
    }

    Vector3d getStartV()
    {
        return V.front();
    }

    Vector3d getEndV()
    {
        return V.back();
    }

    void setG(Vector3d &g)
    {
        grav = g;
    }

    int size()
    {
        return t.size();
    }
};