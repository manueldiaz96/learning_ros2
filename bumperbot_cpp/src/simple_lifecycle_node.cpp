#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <thread>

class LifeCycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit LifeCycleNode(const std::string &node_name, bool use_intra_process_comms = false)
    : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(use_intra_process_comms))
    {

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        sub_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&LifeCycleNode::msgCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Lifecycle Node on_configure() called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Lifecycle Node on_shutdown() called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Lifecycle Node on_cleanup() called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        rclcpp_lifecycle::LifecycleNode::on_activate(state);
        RCLCPP_INFO(this->get_logger(), "Lifecycle Node on_activated() called");

        std::this_thread::sleep_for(std::chrono::seconds(2));

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
        sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Lifecycle Node on_deactivate() called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }


    void msgCallback(const std_msgs::msg::String &msg)
    {
        auto state = this->get_current_state();
        if (state.label() == "active")
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Lifecycle node head: " << msg.data.c_str());
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor ste;
    std::shared_ptr<LifeCycleNode> simple_lifecycle_node = std::make_shared<LifeCycleNode>("simple_lifecycle_node");
    ste.add_node(simple_lifecycle_node->get_node_base_interface());
    ste.spin();
    rclcpp::shutdown();
    return 0;
}