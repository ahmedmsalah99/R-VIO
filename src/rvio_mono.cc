#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "rvio/System.h"

class RVIONode : public rclcpp::Node
{
public:
    RVIONode(const std::string &config_path)
        : Node("rvio"), config_path_(config_path)
    {
        using std::placeholders::_1;

        // --- Callback groups ---
        image_group_      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        imu_group_        = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        processing_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions image_opts;
        image_opts.callback_group = image_group_;

        rclcpp::SubscriptionOptions imu_opts;
        imu_opts.callback_group = imu_group_;

        // --- QoS ---
        rclcpp::QoS image_qos(10);
        image_qos.best_effort();       // usually correct for images

        rclcpp::QoS imu_qos(100);
        imu_qos.reliable();            // IMU should be reliable

        // --- Subscribers ---
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            image_qos,
            std::bind(&RVIONode::GrabImage, this, _1),
            image_opts);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            imu_qos,
            std::bind(&RVIONode::GrabImu, this, _1),
            imu_opts);


        auto period = rclcpp::Duration::from_seconds(1.0 / 50);
        processing_timer_ = rclcpp::create_timer(
            this->get_node_base_interface(),         // node_base
            this->get_node_timers_interface(),       // node_timers
            this->get_clock(),                       // clock (so it can follow /clock)
            period,                                  // rclcpp::Duration
            std::bind(&RVIONode::ProcessVIO, this),
            processing_group_                   // optional callback group
        );
        

        RCLCPP_INFO(this->get_logger(), "RVIO Node initialized.");
    }
    void InitSystem()
    {
        Sys = std::make_shared<RVIO::System>(config_path_, this->shared_from_this());
        RCLCPP_INFO(this->get_logger(), "RVIO System initialized.");
    }

private:
    std::string config_path_;
    std::shared_ptr<RVIO::System> Sys;
    bool need_processing = false;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr image_group_;
    rclcpp::CallbackGroup::SharedPtr imu_group_;
    rclcpp::CallbackGroup::SharedPtr processing_group_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Processing timer
    rclcpp::TimerBase::SharedPtr processing_timer_;

    // --- Subscriber Callbacks ---
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        auto *data = new RVIO::ImageData();
        data->Image = cv_ptr->image.clone();
        data->Timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        Sys->PushImageData(data);
        need_processing = true;
    }

    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto *data = new RVIO::ImuData();

        data->AngularVel = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);

        data->LinearAccel = Eigen::Vector3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        static double last = -1;
        data->TimeInterval = (last < 0 ? 0 : t - last);
        last = t;

        data->Timestamp = t;
        Sys->PushImuData(data);
    }

    // --- VIO processing thread implemented with a timer ---
    void ProcessVIO()
    {
        if(need_processing){
            
            auto start = std::chrono::high_resolution_clock::now();
            Sys->MonoVIO();
            auto end = std::chrono::high_resolution_clock::now();
            // Compute duration in milliseconds
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            need_processing = false;
            // Print time
            RCLCPP_INFO(this->get_logger(), "MonoVIO took %ld ms", duration);
        }
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: rvio <config_file>" << std::endl;
        return 1;
    }

    // You must use shared pointer creation because RVIONode
    // uses shared_from_this() internally.
    auto node = std::make_shared<RVIONode>(argv[1]);
    node->InitSystem();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
