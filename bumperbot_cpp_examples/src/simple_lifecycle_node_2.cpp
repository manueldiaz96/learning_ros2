#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"

#include <thread>

class SimpleLifeCycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit SimpleLifeCycleNode(const std::string & node_name, bool intra_process_comms = false) 
    : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) 
    {
        
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state)
    {
        sub_ = create_subscription<std_msgs::msg::String>("chatter", 
                                                           10, 
                                                           std::bind(&SimpleLifeCycleNode::subCallback, 
                                                                     this, 
                                                                     std::placeholders::_1
                                                                    )
                                                        );

    RCLCPP_INFO(get_logger(), "Lifecycle on_configure called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state)
    {
        sub_.reset();
        RCLCPP_INFO(get_logger(), "Lifecycle on_shutdown called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state)
    {
        sub_.reset();
        RCLCPP_INFO(get_logger(), "Lifecycle on_cleanup called.");

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
    {
        LifecycleNode::on_activate(state);
        RCLCPP_INFO(get_logger(), "Lifecycle on_activate called.");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
    {
        LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(get_logger(), "Lifecycle on_deactivate called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    void subCallback(const std_msgs::msg::String msg){

        auto state = get_current_state();

        if(state.label() == "active"){
            RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg.data.c_str());
        }

        return;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor ste;
    std::shared_ptr<SimpleLifeCycleNode> simple_lifecycle_node = std::make_shared<SimpleLifeCycleNode>("simple_lifecycle_node");
    ste.add_node(simple_lifecycle_node->get_node_base_interface());
    ste.spin();
    rclcpp::shutdown();
    return 0;
}