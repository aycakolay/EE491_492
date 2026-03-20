// file: IK.cpp (send_point_ik.cpp)

#include "cube_spawn.hpp"   // FK, TOUCH_HOLD_SEC, WAIT_AFTER_SPAWN, CMD_RATE_HZ
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>
#include <cmath>
#include <algorithm>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Dense>



// Default joint command topics
static const char* J1_TOPIC_DEF = "/robotic_arm/joint1_controller/command";
static const char* J2_TOPIC_DEF = "/robotic_arm/joint2_controller/command";
static const char* J3_TOPIC_DEF = "/robotic_arm/joint3_controller/command";
static const char* J4_TOPIC_DEF = "/robotic_arm/joint4_controller/command";
static const char* J5_TOPIC_DEF = "/robotic_arm/joint5_controller/command";

// Name of cube link in /gazebo/link_states (adjust if your SDF uses a different link name)
static const std::string CUBE_LINK_NAME = "target_cube::link";

// Global to store cube pose from /gazebo/link_states
static geometry_msgs::Pose g_cube_pose_gazebo;
static bool g_cube_pose_valid = false;

// ------------------ LINK STATES CALLBACK ------------------
void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    g_cube_pose_valid = false;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == CUBE_LINK_NAME) {
            g_cube_pose_gazebo = msg->pose[i];
            g_cube_pose_valid  = true;
            return;
        }
    }
}

// ------------------ FK FUNCTION ------------------
static void fk_from_thetas(double theta1, double theta2, double theta3,
                           double& px, double& py, double& pz)
{
    // CONSTANTS
    const double d1 = 0.08327;
    const double d2 = 0.08918;
    const double d3 = 0.03320;
    const double a2 = 0.18000;
    const double a3 = 0.22010;

    const double c1 = std::cos(theta1);
    const double s1 = std::sin(theta1);
    const double c2 = std::cos(theta2);
    const double s2 = std::sin(theta2);
    const double c3 = std::cos(theta3);
    const double s3 = std::sin(theta3);

    const double A = a2 * c2 + a3 * c2 * c3 - d3 * s2;
    const double B = d2 + a3 * s3;

    px = -0.07912 - A * c1 + B * s1;
    py =  0.020  + d1 + s2 * (a2 + a3 * c3) + d3 * c2;
    pz =  0.08412 + A * s1 + B * c1;
}

// ------------------ IK FUNCTION ------------------
static bool solveIK_global(const double px_global,
                           const double py_global,
                           const double pz_global,
                           double& theta1,
                           double& theta2,
                           double& theta3)
{
    // -----------------------------
    // Offsets (GLOBAL -> base frame)

    // -----------------------------
    const double px_offset = -0.07912;
    const double py_offset =  0.0200;
    const double pz_offset =  0.08412;

    // -----------------------------
    // Link parameters (m)
    // -----------------------------
    const double d1 = 0.08327;
    const double d2 = 0.08918;
    const double d3 = 0.03320;
    const double a2 = 0.18000;
    const double a3 = 0.22010;

    // -----------------------------
    // Joint limits (rad) (From URDF parameters)
    // -----------------------------
    const double t1_min = -M_PI/2.0, t1_max =  M_PI/2.0;
    const double t2_min =  -M_PI/2.0,      t2_max =  M_PI/2.0;
    const double t3_min = -M_PI/2.0, t3_max =  M_PI/2.0;

    // -----------------------------
    // End Effector coordinates from joint1 point

    const double px = px_global - px_offset;
    const double py = py_global - py_offset - d1;  // = (p_y - d1)
    const double pz = pz_global - pz_offset;

    const double r2 = px*px + pz*pz;

    // -----------------------------
    // Solve theta3 from:

    const double R = std::sqrt(a2*a2 + d2*d2);
    const double phi = std::atan2(d2, a2);

    const double K = r2 + py*py - (a2*a2 + a3*a3 + d2*d2 + d3*d3);
    double cos_delta = K / (2.0 * a3 * R);

    const double eps = 1e-9;

    // Reachability check for acos
    if (cos_delta > 1.0 + 1e-6 || cos_delta < -1.0 - 1e-6)
        return false;

    // Clamp to avoid NaN due to numerical noise
    if (cos_delta >  1.0) cos_delta =  1.0;
    if (cos_delta < -1.0) cos_delta = -1.0;

    const double delta = std::acos(cos_delta);

    // Helper: wrap angle to [-pi, pi]
    auto wrapToPi = [](double a)->double {
        while (a >  M_PI) a -= 2.0*M_PI;
        while (a < -M_PI) a += 2.0*M_PI;
        return a;
    };

    // Helper: wrap theta2 to [0, 2pi)
    auto wrapTo2Pi = [](double a)->double {
        while (a >= 2.0*M_PI) a -= 2.0*M_PI;
        while (a <  0.0)      a += 2.0*M_PI;
        return a;
    };

    // This one returns FK check error with the found IK angles
    auto residual = [&](double t1, double t2, double t3)->double {
        const double c1 = std::cos(t1), s1 = std::sin(t1);
        const double c2 = std::cos(t2), s2 = std::sin(t2);
        const double c3 = std::cos(t3), s3 = std::sin(t3);

        const double A = d3*s2 - a2*c2 - a3*c2*c3;
        const double B = d2 + a3*s3;

        const double px_hat = A*c1 + B*s1;
        const double pz_hat = -A*s1 + B*c1;
        const double py_hat = (a2 + a3*c3)*s2 + d3*c2;  // = (p_y - d1)

        const double ex = px_hat - px;
        const double ey = py_hat - py;
        const double ez = pz_hat - pz;
        return ex*ex + ey*ey + ez*ez;
    };

    // Candidate container (angles - residual error - validity flag)
    struct Sol { double t1, t2, t3; double err; bool ok; };

    Sol best{0,0,0,0,false}; // initialize a structure to hold the best solution

    auto tryCandidate = [&](double t3_candidate, int A_sign)->Sol {
        // Enforce theta3 limit early
        double t3c = wrapToPi(t3_candidate);
        if (t3c < t3_min - 1e-12 || t3c > t3_max + 1e-12)
            return {0,0,0,0,false};

        const double c3 = std::cos(t3c);
        const double s3 = std::sin(t3c);

        const double D = a2 + a3*c3;
        const double B = d2 + a3*s3;

        // A = ±sqrt(r2 - B^2)
        const double rad = r2 - B*B;
        if (rad < -1e-6)
            return {0,0,0,0,false};

        const double root = std::sqrt(std::max(0.0, rad));
        const double A = (A_sign >= 0) ? root : -root;

        // -----------------------------
        // Solve theta2 from:

        const double denom = D*D + d3*d3;
        const double s2 = (D*py + d3*A) / denom;
        const double c2 = (d3*py - D*A) / denom;


        double t2c = std::atan2(s2, c2);
        // Wrap theta2 to [-pi, pi] to allow negative values
        t2c = wrapToPi(t2c);

        if (t2c < t2_min - 1e-12 || t2c > t2_max + 1e-12)
            return {0,0,0,0,false};

        // -----------------------------
        // Solve theta1 from:
        double t1c = std::atan2(px*B - pz*A, px*A + pz*B);
        t1c = wrapToPi(t1c);

        if (t1c < t1_min - 1e-12 || t1c > t1_max + 1e-12)
            return {0,0,0,0,false};

        const double err = residual(t1c, t2c, t3c);
        return {t1c, t2c, t3c, err, true};
    };

    // Two theta3 branches: phi ± delta
    const double t3a = phi + delta;
    const double t3b = phi - delta;

    // Four total candidates: (t3a,t3b) × (A=+root, A=-root)
    Sol cand[4];
    cand[0] = tryCandidate(t3a, +1);
    cand[1] = tryCandidate(t3a, -1);
    cand[2] = tryCandidate(t3b, +1);
    cand[3] = tryCandidate(t3b, -1);


    // Pick the valid one with minimum FK residual
    bool found = false;
    for (int i = 0; i < 4; ++i)
    {
        if (!cand[i].ok) continue;
        if (!found || cand[i].err < best.err)
        {
            best = cand[i];
            found = true;
        }
    }

    if (!found)
        return false;

    theta1 = best.t1;
    theta2 = best.t2;
    theta3 = best.t3;
    return true;
}


// ------------------ MAIN ------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "send_point_geometric_ik");
    ros::NodeHandle nh("~");

    // -------------------------------------------------
    // 1) FIXED JOINT COMMANDS (THIS DEFINES THE FK POINT)
    // -------------------------------------------------
    const double Q1_CMD = 1.40;
    const double Q2_CMD = 0.0;
    const double Q3_CMD = -1.40;
    const double Q4_CMD = 0.0;
    const double Q5_CMD = 0.0;

    // -------------------------------------------------
    // Joint command topics (can override via params)
    // -------------------------------------------------
    std::string j1_topic = J1_TOPIC_DEF;
    std::string j2_topic = J2_TOPIC_DEF;
    std::string j3_topic = J3_TOPIC_DEF;
    std::string j4_topic = J4_TOPIC_DEF;
    std::string j5_topic = J5_TOPIC_DEF;

    nh.param<std::string>("j1_cmd_topic", j1_topic, j1_topic);
    nh.param<std::string>("j2_cmd_topic", j2_topic, j2_topic);
    nh.param<std::string>("j3_cmd_topic", j3_topic, j3_topic);
    nh.param<std::string>("j4_cmd_topic", j4_topic, j4_topic);
    nh.param<std::string>("j5_cmd_topic", j5_topic, j5_topic);

    ros::Publisher pub_j1 = nh.advertise<std_msgs::Float64>(j1_topic, 1, false);
    ros::Publisher pub_j2 = nh.advertise<std_msgs::Float64>(j2_topic, 1, false);
    ros::Publisher pub_j3 = nh.advertise<std_msgs::Float64>(j3_topic, 1, false);
    ros::Publisher pub_j4 = nh.advertise<std_msgs::Float64>(j4_topic, 1, false);
    ros::Publisher pub_j5 = nh.advertise<std_msgs::Float64>(j5_topic, 1, false);

    // Subscribe to /gazebo/link_states for cube pose
    ros::Subscriber sub_links = nh.subscribe("/gazebo/link_states", 1, linkStatesCallback);

    // Read cube SDF path from param
    std::string cube_sdf_path;
    nh.param<std::string>("cube_sdf_path", cube_sdf_path, std::string(""));
    if (cube_sdf_path.empty()) {
        ROS_ERROR("Parameter '~cube_sdf_path' is empty or not set.");
        return 1;
    }

    // ----------------- OPTIONAL: wait for controllers -----------------
    ROS_INFO_STREAM("Waiting for joint controller subscribers...");
    while (ros::ok() &&
           (pub_j1.getNumSubscribers() == 0 ||
            pub_j2.getNumSubscribers() == 0 ||
            pub_j3.getNumSubscribers() == 0 ||
            pub_j4.getNumSubscribers() == 0 ||
            pub_j5.getNumSubscribers() == 0)) {
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Controllers are ready.");

        // -------------------------------------------------
        // LOOP: 20 times random Q1_CMD, Q2_CMD, Q3_CMD
        // -------------------------------------------------
        const int NUM_ITER = 50;
        const double Q_MIN = -1.57;
        const double Q_MAX =  1.57;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(Q_MIN, Q_MAX);
        for (int iter = 0; iter < NUM_ITER && ros::ok(); ++iter) {
            double Q1_CMD = dis(gen);
            double Q2_CMD = dis(gen);
            double Q3_CMD = dis(gen);
            ROS_INFO_STREAM("[" << iter+1 << "/" << NUM_ITER << "] Random Q_CMDs: "
                            << Q1_CMD << ", " << Q2_CMD << ", " << Q3_CMD);

            geometry_msgs::Point ee_global = nCube::fk(Q1_CMD, Q2_CMD, Q3_CMD);
            ROS_INFO_STREAM("FK point (from commanded joints): x=" << ee_global.x
                            << ", y=" << ee_global.y
                            << ", z=" << ee_global.z);

            if (!nCube::spawnCubeAt(cube_sdf_path, ee_global, nh)) {
                ROS_ERROR("Failed to spawn cube at FK point.");
                continue;
            }
            ROS_INFO("Spawned cube at FK point. Waiting for cube pose from /gazebo/link_states...");

            const ros::Time t_spawn = ros::Time::now();
            ros::Rate wait_rate(50.0);
            g_cube_pose_valid = false;
            while (ros::ok() && (ros::Time::now() - t_spawn).toSec() < WAIT_AFTER_SPAWN && !g_cube_pose_valid) {
                ros::spinOnce();
                wait_rate.sleep();
            }
            if (!g_cube_pose_valid) {
                ROS_ERROR_STREAM("Cube link '" << CUBE_LINK_NAME
                                << "' not found in /gazebo/link_states.");
                continue;
            }
            const auto& pc = g_cube_pose_gazebo.position;
            ROS_INFO_STREAM("Cube pose from Gazebo (world): x=" << pc.x
                            << ", y=" << pc.y
                            << ", z=" << pc.z);

            double q1_ik = 0.0, q2_ik = 0.0, q3_ik = 0.0;
            if (!solveIK_global(pc.x, pc.y, pc.z, q1_ik, q2_ik, q3_ik)) {
                ROS_ERROR_STREAM("IK failed for cube position ("
                                << pc.x << ", " << pc.y << ", " << pc.z << ")");
                continue;
            }
            ROS_INFO_STREAM("IK solution (q1,q2,q3): "
                            << q1_ik << ", " << q2_ik << ", " << q3_ik);

            double hold_sec = TOUCH_HOLD_SEC;  // from cube_spawn.hpp
            nh.param("hold_sec", hold_sec, hold_sec);
            ROS_INFO_STREAM("Commanding robot to IK solution for " << hold_sec << " seconds...");
            ros::Rate loop(CMD_RATE_HZ);
            std_msgs::Float64 msg;
            const ros::Time t0 = ros::Time::now();
            while (ros::ok() && (ros::Time::now() - t0).toSec() < hold_sec) {
                msg.data = q1_ik;  pub_j1.publish(msg);
                msg.data = q2_ik;  pub_j2.publish(msg);
                msg.data = q3_ik;  pub_j3.publish(msg);
                msg.data = Q4_CMD; pub_j4.publish(msg);
                msg.data = Q5_CMD; pub_j5.publish(msg);
                ros::spinOnce();
                loop.sleep();
            }

            if (!nCube::deleteCube(nh)) {
                ROS_WARN("Failed to delete cube after iteration.");
            } else {
                ROS_INFO("Cube deleted after iteration.");
            }
            ros::Duration(0.5).sleep(); // Short pause between iterations
        }
        ROS_INFO("All iterations complete. Node exiting.");
        return 0;
}
