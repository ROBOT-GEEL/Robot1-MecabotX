#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <chrono>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <iostream>

using namespace std::chrono_literals;


// -------------
// Decorator atijd-SUCCES
class ForceSuccess : public BT::DecoratorNode
{
public:
    ForceSuccess(const std::string& name, const BT::NodeConfiguration& config)
        : BT::DecoratorNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        const BT::NodeStatus child_state = child_node_->executeTick();

        if (child_state == BT::NodeStatus::RUNNING) {
            return BT::NodeStatus::RUNNING;
        }
        // Altijd SUCCESS teruggeven, ongeacht wat het kind doet
        return BT::NodeStatus::SUCCESS;
    }
};

class StopNode : public BT::SyncActionNode
{
public:
    StopNode(const std::string &name) : BT::SyncActionNode(name, {}) {
        node_ = rclcpp::Node::make_shared("btStopNode");
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }
    BT::NodeStatus tick() override {
        std::cout << "[StopNode] STOP DRIVING" << std::endl;
        std::string state = "StopNode";
    	std_msgs::msg::String msg;
        msg.data = state;
        pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
        private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class WaitDriving : public BT::SyncActionNode
{
public:
    WaitDriving(const std::string &name) : BT::SyncActionNode(name, {}) {
    node_ = rclcpp::Node::make_shared("btWaitDriving");
    pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);}
    BT::NodeStatus tick() override {
        std::string state = "WaitDriving";
    	std_msgs::msg::String msg;
        msg.data = state;
        pub_->publish(msg);
        std::cout << "[WaitDriving] STOP DRIVING" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
        private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class CheckNetworkError : public BT::StatefulActionNode
{
public:
    CheckNetworkError(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), level_(100.0)
    {


    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {


        // Als batterij al te laag is, meteen FAILURE
        if (level_ < 30.0)
        {
	std::cout << "[CheckNetworkError] NETWORK ERROR -> FAILURE. Level: " << level_ << std::endl;
	level_ += 0;

            return BT::NodeStatus::FAILURE;
        }

        // Anders RUNNING totdat de volgende tick komt
        level_ -= 0.0;
        std::cout << "[CheckNetworkError] NETWORK oke: " << level_ << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // Simuleer dat de batterij afneemt
        level_ -= 0;


        if (level_ < 30.0)
        {
            level_ += 0;
	std::cout << "[CheckNetworkError] NETWORK ERROR -> FAILURE. Level: " << level_ << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        std::cout << "[CheckNetworkError] NETWORK oke: " << level_ << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[CheckNetworkError] HALTED" << std::endl;
    }

private:
    double level_;
};

class CheckCollision : public BT::StatefulActionNode
{
public:
    CheckCollision(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), level_(100.0)
    {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {
       

        // Als batterij al te laag is, meteen FAILURE
        if (level_ < 30.0)
        {
            std::cout << "[CheckCollision] COLLSION!!!" << std::endl;
            level_ += 0;
            return BT::NodeStatus::FAILURE;
        }

        // Anders RUNNING totdat de volgende tick komt
        std::cout << "[CheckCollision] noCollisionDetected" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        
        level_ -= 0;

        if (level_ < 30.0)
        {
            std::cout << "[CheckCollision] COLLSION!!!" << std::endl;
            level_ += 0;
            return BT::NodeStatus::FAILURE;
        }
        std::cout << "[CheckCollision] noCollisionDetected" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[CheckCollision] HALTED" << std::endl;
    }

private:
    double level_;
};

class BatteryOk : public BT::StatefulActionNode
{
public:
    BatteryOk(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), level_(100.0)
    {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {
        std::cout << "[BatteryOk] Starting check, level = " << level_ << "%" << std::endl;
        level_ -= 0.0;
        // Als batterij al te laag is, meteen FAILURE
        if (level_ < 30.0)
        {
      
            std::cout << "[BatteryOk] Battery too low! -> FAILURE" << std::endl;
            level_ += 0;
            return BT::NodeStatus::FAILURE;
        }

        // Anders RUNNING totdat de volgende tick komt
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // Simuleer dat de batterij afneemt
        level_ -= 0.0;
        std::cout << "[BatteryOk] Battery level = " << level_ << "% -> RUNNING" << std::endl;

        if (level_ < 30.0)
        {
            std::cout << "[BatteryOk] Battery too low! -> FAILURE" << std::endl;
            level_ += 0;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[BatteryOk] HALTED" << std::endl;
    }

private:
    double level_;
};



// -------------------------
// Stateful timer baseclass

class TimedCondition : public BT::StatefulActionNode
{
public:
    TimedCondition(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config), timeout_(7.0) {
       {
        // ROS2 node + publisher aanmaken
        node_ = rclcpp::Node::make_shared("bt_" + name);
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("timeout") };
    }

    BT::NodeStatus onStart() override {
        // haal timeout uit XML, default 7s
        if (!getInput<double>("timeout", timeout_)) {
            timeout_ = 7.0;
        }
                std_msgs::msg::String msg;
        msg.data = name();
        pub_->publish(msg);
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "[" << name() << "] START with timeout = " << timeout_ << "s" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        std::cout << "[" << name() << "] Running... (" << elapsed << "s)" << std::endl;

        if (elapsed >= timeout_) {
            std::cout << "[" << name() << "] Timeout reached " << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[" << name() << "] HALTED" << std::endl;
    }

protected:
    double timeout_;
    std::chrono::steady_clock::time_point start_time_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class InWorkingZone : public BT::SyncActionNode
{
public:
    InWorkingZone(const std::string &name) : BT::SyncActionNode(name, {}) {
       node_ = rclcpp::Node::make_shared("btInWorkingZone");
       pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
       }
    BT::NodeStatus tick() override {
        std::cout << "[InWorkingZone] Checking if in work zone (sim)" << std::endl;
        std::string state = "InWorkingZone";
    	std_msgs::msg::String msg;
        msg.data = state;
        pub_->publish(msg);
        return BT::NodeStatus::FAILURE;
    }
        private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class InChargingStation : public BT::SyncActionNode
{
public:
    InChargingStation(const std::string &name) : BT::SyncActionNode(name, {}) {
            node_ = rclcpp::Node::make_shared("btInChargingStation");
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);}
    BT::NodeStatus tick() override {
        	std::string state = "InChargingStation";
    	std_msgs::msg::String msg;
        msg.data = state;
        pub_->publish(msg);
        std::cout << "[InChargingStation] Checking if in InChargingStation (sim)" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
        private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class DriveToChargingStation : public BT::SyncActionNode
{
public:
    DriveToChargingStation(const std::string &name) : BT::SyncActionNode(name, {}) {
        node_ = rclcpp::Node::make_shared("btDriveToChargingStation");
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);}
    BT::NodeStatus tick() override {
      	std::string state = "DriveToChargingStation";
    	std_msgs::msg::String msg;
        msg.data = state;
        pub_->publish(msg);
        std::cout << "[DriveToChargingStation] Drive to charging station" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
        private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};


class MoveLocationWorkarea : public BT::SyncActionNode
{
public:
    MoveLocationWorkarea(const std::string &name) : BT::SyncActionNode(name, {}) {
            node_ = rclcpp::Node::make_shared("btMoveLocationWorkarea");
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
}
    BT::NodeStatus tick() override {
        std::string state = "MoveLocationWorkarea";
    	std_msgs::msg::String msg;
        msg.data = state;
        pub_->publish(msg);
        std::cout << "[MoveLocationWorkarea] Moving to work area (sim)" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
        private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class RobotExplore : public BT::SyncActionNode
{
public:
    RobotExplore(const std::string &name) : BT::SyncActionNode(name, {}) {
    node_ = rclcpp::Node::make_shared("btRobotExplore");
    pub_quiz_ = node_->create_publisher<std_msgs::msg::String>("/quiz_pi_con", 10);
    pub_bt_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);


    }
    BT::NodeStatus tick() override {
    	std::string state = "exploring";
    	std_msgs::msg::String msg;
        msg.data = state;
        pub_quiz_->publish(msg);
        
        std::string bt_state = "RobotExplore";
        std_msgs::msg::String bt_msg;
        bt_msg.data = bt_state;
        pub_bt_->publish(bt_msg);

        std::cout << "[RobotExplore] Exploring environment (sim)" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;  // bestaande publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;    // nieuwe publisher
};


class CheckCoordAvailable : public BT::SyncActionNode
{
public:
    CheckCoordAvailable(const std::string &name) : BT::SyncActionNode(name, {}) {
            node_ = rclcpp::Node::make_shared("btCheckCoordAvailable");
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }
    BT::NodeStatus tick() override {
        std::string state = "CheckCoordAvailable";
    	std_msgs::msg::String msg;
        msg.data = state;
        pub_->publish(msg);
        std::cout << "[CheckCoordAvailable] Checking coordinate availability (sim)" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
        private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class MoveToVisitor : public BT::SyncActionNode
{
public:
    MoveToVisitor(const std::string &name) : BT::SyncActionNode(name, {}) {
            node_ = rclcpp::Node::make_shared("btMoveToVisitor");
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
}
    BT::NodeStatus tick() override {
        	std::string state = "MoveToVisitor";
    	std_msgs::msg::String msg;
        msg.data = state;
        pub_->publish(msg);

        std::cout << "[MoveToVisitor] Moving to visitor (sim)" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
        private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class Check_At_ChargingStation: public TimedCondition 
{ 
public: 
     Check_At_ChargingStation(const std::string &name, const BT::NodeConfiguration &config) : TimedCondition(name, config){} 
     };

class robotAtPerson : public BT::SyncActionNode
{
public:
    robotAtPerson(const std::string &name) : BT::SyncActionNode(name, {}) {
    node_ = rclcpp::Node::make_shared("btRobotAtPerson");
    pub_quiz_ = node_->create_publisher<std_msgs::msg::String>("/quiz_pi_con", 10);
            pub_bt_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
}
    BT::NodeStatus tick() override {
    	std::string state = "robot-arrived-at-visitors";
    	std_msgs::msg::String msg;
        msg.data = state;
        pub_quiz_->publish(msg);
        
        std::string bt_state = "robotAtPerson";
        std_msgs::msg::String bt_msg;
        bt_msg.data = bt_state;
        pub_bt_->publish(bt_msg);

        std::cout << "[robotAtPerson] robot arrived at person" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;    // nieuwe publisher

};
//VOOR TESTEN ZAL CHECKSTARTBUTTON SUCCES GEVEN ALS NODE NIET JUISTE STATUS GEEFT NA 10 SECONDEN
class CheckStartButton : public BT::StatefulActionNode
{
public:
    CheckStartButton(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), timeout_(10.0), received_(false)
    {
        node_= rclcpp::Node::make_shared("btCheckStartButton");
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/quiz_status", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                if(msg->data == "on_drive_to_quiz_location") {
                    received_ = true;
                }
            });
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("timeout") };
    }

    BT::NodeStatus onStart() override {
        if (!getInput<double>("timeout", timeout_)) {
            timeout_ = 10.0; // default
        }
        start_time_ = std::chrono::steady_clock::now();
        received_ = false;
        std::cout << "[" << name() << "] START waiting for start button or drive_to_quiz, timeout = " << timeout_ << "s" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        if (received_) {
            std::cout << "[" << name() << "] Received 'on_drive_to_quiz_location', returning SUCCESS" << std::endl;
            return BT::NodeStatus::FAILURE; // geinverteerd om te kunnen testen !!!!!
        }

        if (elapsed >= timeout_) {
            std::cout << "[" << name() << "] Timeout reached, returning FAILURE" << std::endl;
            return BT::NodeStatus::SUCCESS;  // geinverteerd om te kunnen testen !!!!!
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[" << name() << "] HALTED" << std::endl;
    }

private:
    double timeout_;
    bool received_;
    std::chrono::steady_clock::time_point start_time_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};




class DriveQuizLocation : public BT::SyncActionNode
{
public:
    DriveQuizLocation(const std::string &name) : BT::SyncActionNode(name, {})
    {
        node_ = rclcpp::Node::make_shared("btDriveQuizLocation");

        // bestaande publisher behouden
        pub_bt_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);

        // nieuwe publisher voor coördinaten
        pub_coord_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/btDriveCoord", 10);
    }

    BT::NodeStatus tick() override
    {
        
        std::string state = "DriveQuizLocation";
        std_msgs::msg::String bt_msg;
        bt_msg.data = state;
        pub_bt_->publish(bt_msg);
        std::cout << "[DriveQuizLocation] Published BT node state: " << state << std::endl;

        geometry_msgs::msg::PoseStamped coord_msg;
        coord_msg.header.stamp = node_->get_clock()->now();
        coord_msg.header.frame_id = "map";  

        coord_msg.pose.position.x = 5.0;
        coord_msg.pose.position.y = 2.5;
        coord_msg.pose.position.z = 0.0;

        coord_msg.pose.orientation.w = 1.0;
        coord_msg.pose.orientation.x = 0.0;
        coord_msg.pose.orientation.y = 0.0;
        coord_msg.pose.orientation.z = 0.0;

        pub_coord_->publish(coord_msg);
        std::cout << "[DriveQuizLocation] Published coordinate to /btDriveCoord: ("
                  << coord_msg.pose.position.x << ", "
                  << coord_msg.pose.position.y << ", "
                  << coord_msg.pose.position.z << ")" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_coord_;
};



class IsRobotAtQuiz : public BT::StatefulActionNode
{
public:
    IsRobotAtQuiz(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          timeout_(10.0),
          success_received_(false),
          failure_received_(false)
    {
        // Maak ROS2 node en subscriber
        node_ = rclcpp::Node::make_shared("btIsRobotAtQuiz");
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/driveCoordStatus", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                if (msg->data == "") {
                    success_received_ = true;
                    failure_received_ = false;
                }
                else if(msg->data == " "){
                    success_received_ = false;
                    failure_received_ = true;
                }
            });

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        // timeout komt uit XML
        return { BT::InputPort<double>("timeout") };
    }

    BT::NodeStatus onStart() override
    {
        // Reset flags en start timer
        success_received_ = false;
        failure_received_ = false;
        start_time_ = std::chrono::steady_clock::now();

        if (!getInput<double>("timeout", timeout_)) {
            timeout_ = 10.0;
        }

        // Publiceer start
        std_msgs::msg::String msg;
        msg.data = "IsRobotAtQuiz";
        pub_->publish(msg);

        std::cout << "[IsRobotAtQuiz] START monitoring /quiz_status for 'succesfullyDriven' (timeout "
                  << timeout_ << "s)" << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);  // Laat ROS2 callbacks lopen

        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        if (success_received_)
        {
            std::cout << "[IsRobotAtQuiz] Received 'succesfullyDriven' -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        
	else if (failure_received_){
	    std::cout << "[IsRobotAtQuiz] Failure 'succesfullyDriven' -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
	}
	
	
        if (elapsed >= timeout_)
        {
            std::cout << "[IsRobotAtQuiz] Timeout (" << timeout_ << "s) reached -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[IsRobotAtQuiz] Waiting... elapsed: " << elapsed << "s" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[IsRobotAtQuiz] HALTED" << std::endl;
    }

private:
    double timeout_;
    bool success_received_;
    bool failure_received_;
    std::chrono::steady_clock::time_point start_time_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class RobotAtQuiz : public BT::SyncActionNode
{
public:
    RobotAtQuiz(const std::string &name) : BT::SyncActionNode(name, {}) {
        node_ = rclcpp::Node::make_shared("robot_at_quiz_node");

        // Bestaande publisher behouden
        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>("/quiz_pi_con", 10);

        // Nieuwe publisher voor BehaviorTree-node status
        pub_bt_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    BT::NodeStatus tick() override {

        std::string state = "robot-arrived-at-quiz-location";
        std_msgs::msg::String msg;
        msg.data = state;
        pub_quiz_->publish(msg);


        std::string bt_state = "RobotAtQuiz";
        std_msgs::msg::String bt_msg;
        bt_msg.data = bt_state;
        pub_bt_->publish(bt_msg);

        std::cout << "[RobotAtQuiz] Robot arrived at quiz location, published to /quiz_pi_con and /BehaviorTreeNode" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;  // bestaande publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;    // nieuwe publisher
};


// -------------------------
// Timer nodes (met timeout uit XML)
// -------------------------
class Check_at_workarea: public TimedCondition 
{ 
public: 
     Check_at_workarea(const std::string &name, const BT::NodeConfiguration &config) : TimedCondition(name, config){} 
     };
     
class check_ArrivedAtPerson : public TimedCondition 
{ 
public: 
      check_ArrivedAtPerson(const std::string &name, const BT::NodeConfiguration &config) : TimedCondition(name, config) {} 
};


class WaitQuizToEnd : public TimedCondition 
{ 
public: WaitQuizToEnd(const std::string &name, const BT::NodeConfiguration &config) : TimedCondition(name, config) {} 
};

// -------------------------
// MAIN
// -------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    // registreer nodes
    factory.registerNodeType<InWorkingZone>("InWorkingZone");
    factory.registerNodeType<MoveLocationWorkarea>("MoveLocationWorkarea");
    factory.registerNodeType<Check_at_workarea>("Check_at_workarea");
    factory.registerNodeType<RobotExplore>("RobotExplore");
    factory.registerNodeType<CheckCoordAvailable>("CheckCoordAvailable");
    factory.registerNodeType<MoveToVisitor>("MoveToVisitor");
    factory.registerNodeType<check_ArrivedAtPerson>("check_ArrivedAtPerson");
    factory.registerNodeType<robotAtPerson>("robotAtPerson");
    factory.registerNodeType<CheckStartButton>("CheckStartButton");
    factory.registerNodeType<DriveQuizLocation>("DriveQuizLocation");
    factory.registerNodeType<IsRobotAtQuiz>("IsRobotAtQuiz");
    factory.registerNodeType<RobotAtQuiz>("RobotAtQuiz");
    factory.registerNodeType<WaitQuizToEnd>("WaitQuizToEnd");
    factory.registerNodeType<BatteryOk>("BatteryOk");
    factory.registerNodeType<InChargingStation>("InChargingStation");
    factory.registerNodeType<DriveToChargingStation>("DriveToChargingStation");
    factory.registerNodeType<Check_At_ChargingStation>("Check_At_ChargingStation");
    factory.registerNodeType<CheckCollision>("CheckCollision");
    factory.registerNodeType<CheckNetworkError>("CheckNetworkError");
    factory.registerNodeType<StopNode>("StopNode");
    factory.registerNodeType<WaitDriving>("WaitDriving");
    factory.registerNodeType<ForceSuccess>("RightBranch");

    // laad boom uit XML
    auto tree = factory.createTreeFromFile("src/mecabot_bt/trees/behavior_tree.xml");

    std::cout << "--- Starting BT in continuous mode ---" << std::endl;
    rclcpp::Rate loop_rate(1.0); // 1 Hz tick = max rate

    while (rclcpp::ok())
    {
        BT::NodeStatus status = tree.tickRoot();

        if (status == BT::NodeStatus::SUCCESS) {
            std::cout << "--- Tree ticked to SUCCESS ---" << std::endl;
            // Optioneel: reset de boom zodat sommige nodes opnieuw kunnen uitvoeren
            tree.rootNode()->halt();
        }
        else if (status == BT::NodeStatus::FAILURE) {
            std::cout << "--- Tree ticked to FAILURE ---" << std::endl;
            // Optioneel: reset de boom om opnieuw te proberen
            tree.rootNode()->halt();
        }

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

