#include <algorithm>
#include <memory>
#include <iostream>
#include <string>

#include "lqr_local_planner/lqr_local_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace lqr_local_planner
{
    template <typename Iter, typename Getter>
    Iter min_by(Iter begin, Iter end, Getter getCompareVal)
    {
        if (begin == end)
        {
            return end;
        }
        auto lowest = getCompareVal(*begin);
        Iter lowest_it = begin;
        for (Iter it = ++begin; it != end; ++it)
        {
            auto comp = getCompareVal(*it);
            if (comp < lowest)
            {
                lowest = comp;
                lowest_it = it;
            }
        }
        return lowest_it;
    }

    void LQRLocalPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                                    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                                    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent;
        auto node = node_.lock(); // transfer weak_ptr into refrenced shared_ptr

        /*assign parameters to class member vars*/
        plugin_name_ = name;
        costmap_ros_ = costmap_ros;
        tf_ = tf;

        /*get logger and clock objects from node shared_ptr */
        logger_ = node->get_logger();
        clock_ = node->get_clock();

        /*declare LQR controller params*/
        /*lookhead*/
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".lookhead_time", rclcpp::ParameterValue(1.0));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_lookhead_distance", rclcpp::ParameterValue(0.3));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_lookhead_distance", rclcpp::ParameterValue(0.9));
        /**base*/
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_dist_tol", rclcpp::ParameterValue(0.2));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_tol", rclcpp::ParameterValue(0.5));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".base_frame", rclcpp::ParameterValue("base_link"));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".map_frame", rclcpp::ParameterValue("map"));
        /***linear velocity*/
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_v", rclcpp::ParameterValue(0.8));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_v", rclcpp::ParameterValue(0.2));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_inc_v", rclcpp::ParameterValue(0.5));
        /**** angular velocity */
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_w", rclcpp::ParameterValue(1.75));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_w", rclcpp::ParameterValue(0.1));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_w_inc", rclcpp::ParameterValue(1.75));
        /***** iter */
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".controller_frequancy", rclcpp::ParameterValue(20));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_iter", rclcpp::ParameterValue(1000));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".eps_iter", rclcpp::ParameterValue(1e-1));
        nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));

        node->get_parameter(plugin_name_ + ".lookhead_time", lookhead_time_);
        node->get_parameter(plugin_name_ + ".min_lookhead_distance", min_lookhead_distance_);
        node->get_parameter(plugin_name_ + ".max_lookhead_distance", max_lookhead_distance_);
        node->get_parameter(plugin_name_ + ".goal_dist_tol", goal_dist_tol_);
        node->get_parameter(plugin_name_ + ".rotate_tol", rotate_tol_);
        node->get_parameter(plugin_name_ + ".base_frame", base_frame_);
        node->get_parameter(plugin_name_ + ".map_frame", map_frame_);
        node->get_parameter(plugin_name_ + ".max_v", max_v_);
        node->get_parameter(plugin_name_ + ".min_v", min_v_);
        node->get_parameter(plugin_name_ + ".max_inc_v", max_v_inc_);
        node->get_parameter(plugin_name_ + ".max_w", max_w_);
        node->get_parameter(plugin_name_ + ".min_w", min_w_);
        node->get_parameter(plugin_name_ + ".max_w_inc", max_w_inc_);
        node->get_parameter(plugin_name_ + ".controller_frequancy", controller_freqency_);

        d_t_ = 1 / controller_freqency_;

        node->get_parameter(plugin_name_ + ".max_iter", max_iter_);
        node->get_parameter(plugin_name_ + ".eps_iter", eps_iter_);
        double transform_tolerance;
        node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
        transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

        std::vector<double> q_diag_val{1.0, 1.0, 1.0};
        Q_ = Eigen::Matrix3d::Zero();
        node->get_parameter(plugin_name_ + ".Q_matrix_diag", q_diag_val);

        for (std::vector<double>::size_type i = 0; i < q_diag_val.size(); ++i)
        {
            Q_(i, i) = q_diag_val[i];
        }

        std::vector<double> r_diag_val{0.1, 0.1};
        R_ = Eigen::Matrix2d::Zero();
        node->get_parameter(plugin_name_ + ".R_matrix_diag", r_diag_val);

        for (std::vector<double>::size_type i = 0; i < r_diag_val.size(); ++i)
        {
            R_(i, i) = r_diag_val[i];
        }

        if (q_diag_val.size() != 3 || r_diag_val.size() != 2)
        {
            throw std::runtime_error("Invalid Q or R matrix diagonal sizes.");
        }

        global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)));

        carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("/lookahead_point", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)));
        current_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)));

        std::cout << "LQR_planner configured proparly !" << std::endl;
    }

    void LQRLocalPlanner::cleanup()
    {
        global_pub_.reset();
        current_pose_pub_.reset();
        carrot_pub_.reset();
    }
    void LQRLocalPlanner::activate()
    {

        global_pub_->on_activate();
    }

    void LQRLocalPlanner::deactivate()
    {

        global_pub_->on_deactivate();
    }

    void LQRLocalPlanner::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    double LQRLocalPlanner::regularizeAngle(double angle) //[180 ,-180]
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    void LQRLocalPlanner::setPlan(const nav_msgs::msg::Path &path)
    {
        global_pub_->publish(path); // for visulization purposes
        global_plan_ = path;        // fetch the global plan to the member var
    }

    Eigen::Vector2d LQRLocalPlanner::LQRComputeControl(const Eigen::Vector3d &current_state, const Eigen::Vector3d &desired_state, const Eigen::Vector2d &refrence_input)
    {

        std::cout << "desired state is :" << desired_state << "\n";
        std::cout << "current state is :" << current_state << "\n";
        std::cout << " ref_input is :" << refrence_input << "\n";

        Eigen::Vector2d input;
        Eigen::Vector3d error(current_state - desired_state); // [x y theta] = [0 1 2]

        error[2] = regularizeAngle(error[2]); // regularize error in theta

        // consier diff drive (speed) model on system equations

        Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
        A(0, 2) = -refrence_input[0] * sin(desired_state[2]) * d_t_;
        A(1, 2) = refrence_input[0] * cos(desired_state[2]) * d_t_;

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 2);
        B(0, 0) = cos(desired_state[2]) * d_t_;
        B(1, 0) = sin(desired_state[2]) * d_t_;
        B(2, 1) = d_t_;

        // solve DARE -discrete algeric riccati equation-

        // costate matrix P
        Eigen::Matrix3d P, P_t;
        P = Q_;
        for (int i = 0; i < max_iter_; ++i)
        {
            Eigen::Matrix2d tmp = R_ + B.transpose() * P * B;
            P_t = Q_ + A.transpose() * P * A - A.transpose() * P * B * tmp.inverse() * B.transpose() * P * A;
            if ((P - P_t).array().abs().maxCoeff() < eps_iter_)
                break;

            P = P_t;
        }
        Eigen::MatrixXd K = -(R_ + B.transpose() * P_t * B).inverse() * B.transpose() * P_t * A;

        input = refrence_input + K * error;
        std::cout << "Control Gain K: " << K << "\n";
        std::cout << "Computed Input: " << input << "\n";

        return input;
    }

    /* transform the global plan itn the robot frame which is moving along the glopal plan and remove the passed part of it  */
    nav_msgs::msg::Path LQRLocalPlanner::transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose)
    {
        /*check lobal plan*/
        if (global_plan_.poses.empty())
        {
            throw nav2_core::PlannerException("no global plan to follow");
        }

        geometry_msgs::msg::PoseStamped robot_pose;

        if (!transformPose(tf_, global_plan_.header.frame_id, pose, robot_pose, transform_tolerance_)) // robot pose in the global plan frame
        {
            throw nav2_core::PlannerException("error in pose transformation into global's plan frame");
        }

        // get the interval of global plan near to the robot
        /*first reduce the costmap size to local cost map as it will not be useful to copute controls out of it
         this is done via threshold */
        /*in conclosion just remove the overhead of whole costmap on the memory case*/

        nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap(); // get the master layered costmap

        double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) * costmap->getResolution() / 2.0; // working with cells requires res

        auto transformation_begin = min_by(global_plan_.poses.begin(), global_plan_.poses.end(), [&robot_pose](const geometry_msgs::msg::PoseStamped &ps)
                                           { return nav2_util::geometry_utils::euclidean_distance(robot_pose, ps); }); // begin pose in the global plan relative (nearest) to the robot pose

        auto transformation_end = std::find_if(transformation_begin, end(global_plan_.poses),
                                               [&](const auto &global_plan_pose)
                                               {
                                                   return nav2_util::geometry_utils::euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
                                               }); // end pose in the global plan relative (thresholded by) to the local costmap size

        auto transformGlobalPoseTolocal = [&](const auto &global_plan_pose)
        {
            geometry_msgs::msg::PoseStamped stamped_pose;
            stamped_pose.header.frame_id = global_plan_.header.frame_id;
            stamped_pose.header.stamp = global_plan_.header.stamp;
            stamped_pose.pose = global_plan_pose.pose;

            geometry_msgs::msg::PoseStamped transormed_pose; // pose in local frame to be inserted in local frame

            transformPose(tf_, costmap_ros_->getBaseFrameID(), stamped_pose, transormed_pose, transform_tolerance_);
            return transormed_pose;
        }; // transform pose by pose from global to local (base frame id )

        /* now insert poses into the local path */
        nav_msgs::msg::Path local_path;
        std::transform(transformation_begin, transformation_end, std::back_inserter(local_path.poses), transformGlobalPoseTolocal);
        local_path.header.frame_id = costmap_ros_->getBaseFrameID();
        local_path.header.stamp = pose.header.stamp;

        /* remove the part of the global plan the robot has passed */
        global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

        global_pub_->publish(local_path);

        if (local_path.poses.empty())
        {
            throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
        }

        return local_path;
    }

    /*transfrom the pose from frame to another */
    bool LQRLocalPlanner::transformPose(const std::shared_ptr<tf2_ros::Buffer> tf,
                                        const std::string frame,
                                        const geometry_msgs::msg::PoseStamped &input_pose,
                                        geometry_msgs::msg::PoseStamped &output_pose, // not const as its going to be edited
                                        const rclcpp::Duration &transform_tolerance) const
    {
        if (input_pose.header.frame_id == frame) // if same as our frame just assign the data
        {
            output_pose = input_pose;
            return true;
        }
        try // if frames not matched then transform
        {
            tf->transform(input_pose, output_pose, frame); // transform pose to the req frame
            return true;
        }
        catch (tf2::ExtrapolationException &ex) // notify that the requested value would have required extrapolation beyond current limits.
        {
            auto transform = tf->lookupTransform(frame, input_pose.header.frame_id, tf2::TimePointZero); // transform object

            // check for tolerance
            if (rclcpp::Time(input_pose.header.stamp) - rclcpp::Time(transform.header.stamp) > transform_tolerance) // tolerance violated
            {
                return false;
            }
            else
            {
                tf2::doTransform(input_pose, output_pose, transform);
                return true;
            }
        }
        catch (tf2::TransformException &ex) // kind of any exception class
        {
            return false;
        }
        return false;
    }

    geometry_msgs::msg::TwistStamped LQRLocalPlanner::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &speed,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)goal_checker;
        auto transformed_plan = transformGlobalPlan(pose);

        double look_dist = getLookAheadDistance(speed);

        auto carrot_pose = getLookAheadPoint(look_dist, transformed_plan);
        //[wrong] carrot_pose.header.frame_id = pose.header.frame_id;
        carrot_pub_->publish(createCarrotMsg(carrot_pose));

        double theta = tf2::getYaw(pose.pose.orientation);

        double linear_vel = 0.0, angular_vel = 0.0;
        double angle_to_heading = 0.0;

        // Define the current state
        Eigen::Vector3d current_state(pose.pose.position.x, pose.pose.position.y, theta);
        std::cout << "Current state: x = " << current_state[0] << ", y = " << current_state[1] << ", theta = " << current_state[2] << std::endl;

        Eigen::Vector3d desired_state(
            carrot_pose.pose.position.x,
            carrot_pose.pose.position.y,
            tf2::getYaw(carrot_pose.pose.orientation));

        std::cout << "Desired state: x = " << desired_state[0] << ", y = " << desired_state[1] << ", theta = " << desired_state[2] << std::endl;

        // Calculate curvature (kappa)
        const double carrot_dist2 = (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
                                    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);
        double kappa = 0.0;
        if (carrot_dist2 > 0.001)
        {
            kappa = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
        }
        std::cout << "Curvature (kappa): " << kappa << std::endl;
        linear_vel = min_v_;
        // Rotate if near the goal or path heading, else use LQR
        if (shouldRotateToGoal(carrot_pose))
        {
            double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
            rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
            std::cout << "Rotating to goal with angle: " << angle_to_goal << std::endl;
        }
        else if (shouldRotateToPath(carrot_pose, angle_to_heading))
        {
            rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
            std::cout << "Rotating to path heading with angle: " << angle_to_heading << std::endl;
        }
        else
        {

            Eigen::Vector2d reference_input(speed.linear.x, speed.linear.x * kappa);
            Eigen::Vector2d control_input = LQRComputeControl(current_state, desired_state, reference_input);
            std::cout << "Control input (pre-regularization): linear = " << control_input[0] << ", angular = " << control_input[1] << std::endl;

            // Regularize velocities
            linear_vel = linearRegularization(speed, control_input[0]);
            angular_vel = angularRegularization(speed, control_input[1]);
            std::cout << "Regularized velocities: linear = " << linear_vel << ", angular = " << angular_vel << std::endl;
        }

        // Prepare the command velocity message
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header = pose.header;
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;

        // Final output of cmd_vel for debugging
        std::cout << "Final cmd_vel: linear.x = " << cmd_vel.twist.linear.x << ", angular.z = " << cmd_vel.twist.angular.z << std::endl;

        return cmd_vel;
    }

    double LQRLocalPlanner::getLookAheadDistance(const geometry_msgs::msg::Twist &current_speed)
    {
        double lookahead_dist = fabs(current_speed.linear.x) * lookhead_time_;
        return std::clamp(lookahead_dist, min_lookhead_distance_, max_lookhead_distance_);
    }

    std::unique_ptr<geometry_msgs::msg::PointStamped> LQRLocalPlanner::createCarrotMsg(
        const geometry_msgs::msg::PoseStamped &carrot_pose)
    {
        auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
        carrot_msg->header = carrot_pose.header;
        carrot_msg->point.x = carrot_pose.pose.position.x;
        carrot_msg->point.y = carrot_pose.pose.position.y;
        carrot_msg->point.z = 0.01;
        return carrot_msg;
    }

    double LQRLocalPlanner::linearRegularization(const geometry_msgs::msg::Twist &base_velocity,
                                                 double desired_velocity)
    {
        double v = std::hypot(base_velocity.linear.x, base_velocity.linear.y);
        double v_inc = desired_velocity - v;

        if (std::fabs(v_inc) > max_v_inc_)
            v_inc = std::copysign(max_v_inc_, v_inc);

        double v_cmd = v + v_inc;
        if (std::fabs(v_cmd) > max_v_)
            v_cmd = std::copysign(max_v_, v_cmd);
        else if (std::fabs(v_cmd) < min_v_)
            v_cmd = std::copysign(min_v_, v_cmd);

        return v_cmd;
    }

    double LQRLocalPlanner::angularRegularization(const geometry_msgs::msg::Twist &base_velocity,
                                                  double desired_angular_velocity)
    {
        if (std::fabs(desired_angular_velocity) > max_w_)
            desired_angular_velocity = std::copysign(max_w_, desired_angular_velocity);

        double w = base_velocity.angular.z;
        double w_inc = desired_angular_velocity - w;

        if (std::fabs(w_inc) > max_w_inc_)
            w_inc = std::copysign(max_w_inc_, w_inc);

        double w_cmd = w + w_inc;
        if (std::fabs(w_cmd) > max_w_)
            w_cmd = std::copysign(max_w_, w_cmd);
        else if (std::fabs(w_cmd) < min_w_)
            w_cmd = std::copysign(min_w_, w_cmd);

        return w_cmd;
    }

    bool LQRLocalPlanner::shouldRotateToGoal(const geometry_msgs::msg::PoseStamped &carrot_pose)
    {
        double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);

        return dist_to_goal < goal_dist_tol_;
    }

    bool LQRLocalPlanner::shouldRotateToPath(geometry_msgs::msg::PoseStamped &carrot_pose,
                                             double &angle_to_path)
    {
        angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
        return fabs(angle_to_path) > rotate_tol_;
    }

    geometry_msgs::msg::Point LQRLocalPlanner::circleSegmentIntersection(
        const geometry_msgs::msg::Point &p1,
        const geometry_msgs::msg::Point &p2,
        double r)
    {
        double x1 = p1.x;
        double x2 = p2.x;
        double y1 = p1.y;
        double y2 = p2.y;

        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr2 = dx * dx + dy * dy;
        double D = x1 * y2 - x2 * y1;

        // Augmentation to only return point within segment
        double d1 = x1 * x1 + y1 * y1;
        double d2 = x2 * x2 + y2 * y2;
        double dd = d2 - d1;

        geometry_msgs::msg::Point p;
        double sqrt_term = std::sqrt(r * r * dr2 - D * D);
        p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
        p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
        return p;
    }
    geometry_msgs::msg::PoseStamped LQRLocalPlanner::getLookAheadPoint(const double &lookahead_dist,
                                                                       const nav_msgs::msg::Path &transformed_plan)
    {
        auto goal_pose_it = std::find_if(
            transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto &ps)
            { return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist; });
        if (goal_pose_it == transformed_plan.poses.end())
        {
            goal_pose_it = std::prev(transformed_plan.poses.end());
        }
        else if (goal_pose_it != transformed_plan.poses.begin())
        {
            // Find the point on the line segment between the two poses
            // that is exactly the lookahead distance away from the robot pose (the origin)
            // This can be found with a closed form for the intersection of a segment and a circle
            // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
            // and goal_pose is guaranteed to be outside the circle.
            auto prev_pose_it = std::prev(goal_pose_it);
            auto point = circleSegmentIntersection(
                prev_pose_it->pose.position,
                goal_pose_it->pose.position, lookahead_dist);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = prev_pose_it->header.frame_id;
            pose.header.stamp = goal_pose_it->header.stamp;
            pose.pose.position = point;
            return pose;
        }

        return *goal_pose_it;
    }

    void LQRLocalPlanner::rotateToHeading(
        double &linear_vel, double &angular_vel,
        const double &angle_to_path, const geometry_msgs::msg::Twist &curr_speed)
    {
        linear_vel = 0.0;
        const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
        angular_vel = sign * 1.8; // rotate_to_heading_angular_vel_

        const double &dt = d_t_;
        const double min_feasible_angular_speed = curr_speed.angular.z - max_w_inc_ * dt;
        const double max_feasible_angular_speed = curr_speed.angular.z + max_w_inc_ * dt;
        angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
    }

}
PLUGINLIB_EXPORT_CLASS(lqr_local_planner::LQRLocalPlanner, nav2_core::Controller)
