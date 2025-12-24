#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <string>
#include <cmath>

using MoveGroup = moveit_msgs::action::MoveGroup;
using GoalHandleMoveGroup = rclcpp_action::ClientGoalHandle<MoveGroup>;

class XArm6ActionClient : public rclcpp::Node
{
public:
    XArm6ActionClient()
        : Node("xarm6_action_client")
    {
        // Cliente de la acción /move_action de MoveIt2
        client_ = rclcpp_action::create_client<MoveGroup>(this, "/move_action");

        RCLCPP_INFO(this->get_logger(), "Esperando al action server /move_action...");
        client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Action server disponible.");

        run_sequence();
    }

private:
    rclcpp_action::Client<MoveGroup>::SharedPtr client_;

    // Función para crear goals articulares
    moveit_msgs::msg::Constraints make_joint_goal(
        const std::vector<std::string>& names,
        const std::vector<double>& positions_deg)
    {
        moveit_msgs::msg::Constraints constraints;

        for (size_t i = 0; i < names.size(); ++i)
        {
            moveit_msgs::msg::JointConstraint jc;
            jc.joint_name = names[i];
            jc.position = positions_deg[i] * M_PI / 180.0;  // grados --> radianes
            jc.weight = 1.0;
            constraints.joint_constraints.push_back(jc);
        }

        return constraints;
    }

    // Enviar un goal con velocidad y aceleración configurables
    void send_goal_and_wait(
        const moveit_msgs::msg::Constraints& constraints,
        double vel_scaling,
        double acc_scaling)
    {
        MoveGroup::Goal goal;
        goal.request.group_name = "xarm6";

        // Parámetros dinámicos
        goal.request.max_velocity_scaling_factor = vel_scaling;
        goal.request.max_acceleration_scaling_factor = acc_scaling;

        goal.request.goal_constraints.push_back(constraints);

        auto future_goal = client_->async_send_goal(goal);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Error enviando goal");
            return;
        }

        auto goal_handle = future_goal.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal rechazada");
            return;
        }

        auto result_future = client_->async_get_result(goal_handle);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);

        RCLCPP_INFO(this->get_logger(), "Goal completada.");
    }

    // Secuencia completa
    void run_sequence()
    {
        // 1) Goal articular
        auto goal1 = make_joint_goal(
            {"joint1","joint2","joint3","joint4","joint5","joint6"},
            {-59.0, -23.0, -40.0, 0.0, 63.0, -59.0}
        );

        RCLCPP_INFO(this->get_logger(), "Enviando goal 1...");
        send_goal_and_wait(goal1, 0.3, 0.2);  // velocidad y aceleración personalizadas

        // 2) Volver a cero
        auto goal2 = make_joint_goal(
            {"joint1","joint2","joint3","joint4","joint5","joint6"},
            {0, 0, 0, 0, 0, 0}
        );

        RCLCPP_INFO(this->get_logger(), "Enviando goal 2 (volver a cero)...");
        send_goal_and_wait(goal2, 0.5, 0.3);

        RCLCPP_INFO(this->get_logger(), "Secuencia completa.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<XArm6ActionClient>();
    rclcpp::shutdown();
    return 0;
}
