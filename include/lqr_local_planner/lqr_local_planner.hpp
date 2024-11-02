#pragma once

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_ros/buffer.h"

#include <Eigen/Dense>

namespace lqr_local_planner
{
    class LQRLocalPlanner : public nav2_core::Controller
    {
    public:
        LQRLocalPlanner() = default;
        ~LQRLocalPlanner() override = default;

        /***********controller plugin methods******/
        void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                       std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                       std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;

        void activate() override;

        void deactivate() override;

        void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &speed,
            nav2_core::GoalChecker *goal_checker) override;

        void setPlan(const nav_msgs::msg::Path &path) override;

    protected:
        geometry_msgs::msg::Point circleSegmentIntersection(
            const geometry_msgs::msg::Point &p1,
            const geometry_msgs::msg::Point &p2,
            double r);
        // std::vector<std::pair<double, double>> circleSegmentIntersection(const std::pair<double, double> &p1,
        //                                                                  const std::pair<double, double> &p2,
        //                                                                  double r);
        double getLookAheadDistance(const geometry_msgs::msg::Twist &current_speed);
        std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
            const geometry_msgs::msg::PoseStamped &carrot_pose);

        double regularizeAngle(double angle);
        double linearRegularization(const geometry_msgs::msg::Twist &base_velocity,
                                    double desired_velocity);
        double angularRegularization(const geometry_msgs::msg::Twist &base_velocity,
                                     double desired_angular_velocity);
        bool shouldRotateToGoal(const geometry_msgs::msg::PoseStamped &carrot_pose);
        bool shouldRotateToPath(geometry_msgs::msg::PoseStamped &carrot_pose,
                                double &angle_to_path);
        Eigen::Vector2d LQRComputeControl(const Eigen::Vector3d &current_state, const Eigen::Vector3d &desired_state, const Eigen::Vector2d &refrence_input);

        nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose);

        bool transformPose(const std::shared_ptr<tf2_ros::Buffer> tf,
                           const std::string frame,
                           const geometry_msgs::msg::PoseStamped &input_pose,
                           geometry_msgs::msg::PoseStamped &output_pose,
                           const rclcpp::Duration &transform_tolerance) const;

        geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &lookahead_dist,
                                                          const nav_msgs::msg::Path &transformed_plan);

        void rotateToHeading(
            double &linear_vel, double &angular_vel,
            const double &angle_to_path, const geometry_msgs::msg::Twist &curr_speed);

        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        rclcpp::Logger logger_{rclcpp::get_logger("LQRController")};
        rclcpp::Clock::SharedPtr clock_;
        rclcpp::Duration transform_tolerance_{0, 0};

        bool goal_reached_;

        /*algorithm parameter*/
        double lookhead_time_;
        double min_lookhead_distance_;
        double max_lookhead_distance_;

        double goal_dist_tol_;
        double rotate_tol_;
        std::string base_frame_;
        std::string map_frame_;

        double max_v_;
        double min_v_;
        double max_v_inc_;

        double max_w_;
        double min_w_;
        double max_w_inc_;

        double d_t_;      // time interval
        int max_iter_;    // maximum itereation for solving riccati equation
        double eps_iter_; // extended period simulation
        int controller_freqency_;

        std::vector<double> diag_vector;
        Eigen::Matrix3d Q_;
        Eigen::Matrix2d R_;

        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PointStamped>> carrot_pub_;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> current_pose_pub_;

        // goal parameters
        double goal_x_, goal_y_;
        Eigen::Vector3d goal_rpy_;

        nav_msgs::msg::Path global_plan_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
    };
}