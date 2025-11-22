#include "roarm_moveit_cmd/roarm_server.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/utils/moveit_error_code.hpp>




std::string model = get_roarm_model();
// Helper function to convert MoveItErrorCode to bool


inline bool isPlanSuccessful(const moveit::core::MoveItErrorCode &code)
{
    return code.val == moveit::core::MoveItErrorCode::SUCCESS;
}


class RobotPoseSubscription : public rclcpp::Node
{
public:
    RobotPoseSubscription() : Node("roarm_server")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&RobotPoseSubscription::joint_states_callback, this, std::placeholders::_1));
    }

    std::array<double, 5> get_hand_pose() const { return pose; }

private:
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        const std::vector<std::string> &names = msg->name;
        const std::vector<double> &positions = msg->position;

        try {
        
            if (model == "roarm_m2") {
              double base = positions.at(std::distance(names.begin(), std::find(names.begin(), names.end(), "base_link_to_link1")));
              double shoulder = positions.at(std::distance(names.begin(), std::find(names.begin(), names.end(), "link1_to_link2")));
              double elbow = positions.at(std::distance(names.begin(), std::find(names.begin(), names.end(), "link2_to_link3")));
              double hand = positions.at(std::distance(names.begin(), std::find(names.begin(), names.end(), "link3_to_gripper_link")));
              
              pose = roarm_m2::computePosbyJointRad(base, shoulder, elbow, hand);
              
            } else if (model == "roarm_m3") {
              double base = positions.at(std::distance(names.begin(), std::find(names.begin(), names.end(), "base_link_to_link1")));
              double shoulder = positions.at(std::distance(names.begin(), std::find(names.begin(), names.end(), "link1_to_link2")));
              double elbow = positions.at(std::distance(names.begin(), std::find(names.begin(), names.end(), "link2_to_link3")));
              double wrist = positions.at(std::distance(names.begin(), std::find(names.begin(), names.end(), "link3_to_link4")));
              double roll = positions.at(std::distance(names.begin(), std::find(names.begin(), names.end(), "link4_to_link5")));
              double hand = positions.at(std::distance(names.begin(), std::find(names.begin(), names.end(), "link5_to_gripper_link")));
              
              pose = roarm_m3::computePosbyJointRad(base, shoulder, elbow, wrist, roll, hand);
            } 

        } catch (const std::out_of_range &e) {
            RCLCPP_WARN(this->get_logger(), "Joint name not found in joint_states message.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::array<double, 5> pose{};
};  

void get_pose_cmd_service(const std::shared_ptr<roarm_msgs::srv::GetPoseCmd::Request> request,
                    std::shared_ptr<roarm_msgs::srv::GetPoseCmd::Response> response,
                    std::shared_ptr<RobotPoseSubscription> node)
{
    auto logger = node->get_logger();
    std::array<double, 5> pose = node->get_hand_pose();
    response->x = pose[0]/1000.0;
    response->y = pose[1]/1000.0;
    response->z = pose[2]/1000.0;
    if (model == "roarm_m2") {
      response->roll = 0.0;
      response->pitch = 0.0;
      response->yaw = 0.0;
    } else if (model == "roarm_m3") {
      response->roll = pose[3];
      response->pitch = pose[4];
      response->yaw = 0.0;
    }    
}

void move_joint_cmd_service(const std::shared_ptr<roarm_msgs::srv::MoveJointCmd::Request> request,
                            std::shared_ptr<roarm_msgs::srv::MoveJointCmd::Response> response)
{
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("move_joint_cmd_service_node");
    auto logger = node->get_logger();
    moveit::planning_interface::MoveGroupInterface move_group(node, "hand");

    std::vector<double> target;
    if (model == "roarm_m2") {
        target = roarm_m2::computeJointRadbyPos(1000*request->x,1000*request->y,1000*request->z,0.0);
    } else if (model == "roarm_m3") {
        target = roarm_m3::computeJointRadbyPos(1000*request->x,1000*request->y,1000*request->z,request->roll, request->pitch);
    }

    move_group.setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto error_code = move_group.plan(plan);
    bool success = isPlanSuccessful(error_code);

    if(success) {
        move_group.execute(plan);
        response->success = true;
        response->message = "MoveJointCmd executed successfully";
    } else {
        response->success = false;
        response->message = "Planning failed!";
    }
}

void move_line_cmd_service(const std::shared_ptr<roarm_msgs::srv::MoveLineCmd::Request> request,
                           std::shared_ptr<roarm_msgs::srv::MoveLineCmd::Response> response,
                           std::shared_ptr<RobotPoseSubscription> node)
{
    auto logger = node->get_logger();
    moveit::planning_interface::MoveGroupInterface move_group(node, "hand");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    std::array<double, 5> start_pose = node->get_hand_pose();
    Pose startPose = {start_pose[0], start_pose[1], start_pose[2], start_pose[3], start_pose[4]};
    std::vector<double> endPoint = {1000*request->x, 1000*request->y, 1000*request->z};
    int numPoints = 10;
    std::vector<Pose> trajectory = generateLinearTrajectory(startPose, endPoint, numPoints);

    for (const auto& pose : trajectory) {
        std::vector<double> target;
        if (model == "roarm_m2") {
            target = roarm_m2::computeJointRadbyPos(pose.x, pose.y, pose.z, 0.0);
        } else if (model == "roarm_m3") {
            target = roarm_m3::computeJointRadbyPos(pose.x, pose.y, pose.z, pose.roll, pose.pitch);
        }

        move_group.setJointValueTarget(target);
        auto error_code = move_group.plan(my_plan);
        bool success = isPlanSuccessful(error_code);

        if(success) {
            move_group.execute(my_plan);
            response->success = true;
            response->message = "MoveLineCmd executed successfully";
        } else {
            response->success = false;
            response->message = "Planning failed!";
        }
    }
}

void move_circle_cmd_service(const std::shared_ptr<roarm_msgs::srv::MoveCircleCmd::Request> request,
                             std::shared_ptr<roarm_msgs::srv::MoveCircleCmd::Response> response,
                             std::shared_ptr<RobotPoseSubscription> node)
{
    auto logger = node->get_logger();
    moveit::planning_interface::MoveGroupInterface move_group(node, "hand");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    std::array<double, 5> start_pose = node->get_hand_pose();
    Pose startPose = {start_pose[0], start_pose[1], start_pose[2], start_pose[3], start_pose[4]};
    std::vector<double> viaPoint = {1000*request->x0, 1000*request->y0, 1000*request->z0};
    std::vector<double> endPoint = {1000*request->x1, 1000*request->y1, 1000*request->z1};
    int numPoints = 10;
    std::vector<Pose> trajectory = generateCircularTrajectory(startPose, viaPoint, endPoint, numPoints);

    for (const auto& pose : trajectory) {
        std::vector<double> target;
        if (model == "roarm_m2") {
            target = roarm_m2::computeJointRadbyPos(pose.x, pose.y, pose.z, 0.0);
        } else if (model == "roarm_m3") {
            target = roarm_m3::computeJointRadbyPos(pose.x, pose.y, pose.z, pose.roll, pose.pitch);
        }

        move_group.setJointValueTarget(target);
        auto error_code = move_group.plan(my_plan);
        bool success = isPlanSuccessful(error_code);

        if(success) {
            move_group.execute(my_plan);
            response->success = true;
            response->message = "MoveCircleCmd executed successfully";
        } else {
            response->success = false;
            response->message = "Planning failed!";
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPoseSubscription>();
    
    auto get_pose_cmd_server = node->create_service<roarm_msgs::srv::GetPoseCmd>("get_pose_cmd", 
                    std::bind(&get_pose_cmd_service, std::placeholders::_1, std::placeholders::_2, node));
    auto move_joint_cmd_server = node->create_service<roarm_msgs::srv::MoveJointCmd>("move_joint_cmd", &move_joint_cmd_service);
    auto move_line_cmd_server = node->create_service<roarm_msgs::srv::MoveLineCmd>("move_line_cmd", 
                    std::bind(&move_line_cmd_service, std::placeholders::_1, std::placeholders::_2, node));
    auto move_circle_cmd_server = node->create_service<roarm_msgs::srv::MoveCircleCmd>("move_circle_cmd", 
                    std::bind(&move_circle_cmd_service, std::placeholders::_1, std::placeholders::_2, node));
    
    RCLCPP_INFO(node->get_logger(), "roarm server is ready to receive requests.");
    rclcpp::spin(node);  
    rclcpp::shutdown(); 

    return 0;
}