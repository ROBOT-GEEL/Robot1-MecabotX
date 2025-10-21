#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <std_msgs/msg/float32.hpp>  // voeg bovenaan toe

//#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>

// =====================================
//  Node 1 - CheckBattery
// =====================================
class CheckBattery : public BT::SyncActionNode
{
public:
  CheckBattery(const std::string &name)
  : BT::SyncActionNode(name, {}), battery_level_(100.0)
  {
// Aanmaak ROS2 node om te kunnen subscriben / schrijven naar topics
    node_ = rclcpp::Node::make_shared("check_battery_node");
// Om batterijwaarden te lezen, moet deze node subscriben 
// <sensor_msgs::msg::BatteryState> geeft het berichttype aan dat je verwacht te ontvangen
// “/battery_state” geeft naam van topic waarnaar luisteren, 10 geeft aan dat buffersize 10 berichten is
// [this] betekent dat die lambda functie toegang heeft tot de variabelen van dit checkbattery-object
// ROS ontvangt bericht = aanmaak shared pointer naar dat bericht, en dit geven aan callbackfunctie
    sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      "/PowerVoltage", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg)
      {
        battery_level_ = msg->data;
      });
  }

  BT::NodeStatus tick() override
  {
// Bij aanroep rclcpp::spin_some(node_) worden laatste statusberichten verwerkt, callback uitgevoerd, battey_level naar nieuwste waarde zetten
    rclcpp::spin_some(node_);
    std::cout << "[CheckBattery] Level: " << battery_level_ << "%" << std::endl;

    if (battery_level_ < 25.0)
    {
      std::cout << "[CheckBattery] Battery low!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    std::cout << "[CheckBattery] Battery OK." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  double battery_level_;
};

// =====================================
//  Node 2 - GoCharge
// =====================================
class GoCharge : public BT::SyncActionNode
{
public:
  GoCharge(const std::string &name)
  : BT::SyncActionNode(name, {})
  {
    node_ = rclcpp::Node::make_shared("go_charge_node");
    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    pub_->publish(stop);

    std::cout << "[GoCharge] Battery low, heading to charge station (simulation)." << std::endl;
    rclcpp::spin_some(node_);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

// =====================================
//  Node 3 - Drive
// =====================================
class Drive : public BT::SyncActionNode
{
public:
  Drive(const std::string &name)
  : BT::SyncActionNode(name, {})
  {
    node_ = rclcpp::Node::make_shared("drive_node");
    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 1.0;   // Veiligheid: stilstaand
    msg.angular.z = 0.0;
    pub_->publish(msg);

    std::cout << "[Drive] Battery OK, driving (simulation: zero speed)." << std::endl;
    rclcpp::spin_some(node_);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

// =====================================
//  MAIN
// =====================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CheckBattery>("CheckBattery");
  factory.registerNodeType<GoCharge>("GoCharge");
  factory.registerNodeType<Drive>("Drive");

  auto tree = factory.createTreeFromFile("src/mecabot_bt_exp2/trees/charge_behavior.xml");

  std::cout << "=== Starting Mecabot BT EXP2 ===" << std::endl;

  // Loop voor continue gedrag
  rclcpp::Rate rate(1.0); // 1 Hz
  while (rclcpp::ok())
  {
    tree.tickRoot();
    rate.sleep();
  }

  std::cout << "=== BT stopped ===" << std::endl;
  rclcpp::shutdown();
  return 0;
}

