#ifndef SEQUOIA_CAMERA_COMPONENT_HPP_
#define SEQUOIA_CAMERA_COMPONENT_HPP_

#include <memory>
#include <string>
#include <atomic>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sequoia_camera/msg/sunshine_sensor.hpp"
#include "sequoia_camera/srv/camera_control.hpp"
#include "sequoia_camera/ptpy_wrapper.hpp"

namespace sequoia_camera
{

  class SequoiaCameraComponent : public rclcpp::Node
  {
  public:
    explicit SequoiaCameraComponent(const rclcpp::NodeOptions &options);
    virtual ~SequoiaCameraComponent();

  private:
    // Service callbacks
    void handleCameraControl(
        const std::shared_ptr<sequoia_camera::srv::CameraControl::Request> request,
        std::shared_ptr<sequoia_camera::srv::CameraControl::Response> response);

    // Publisher methods
    void publishImuData();
    void publishSunshineData();

    // Camera control methods
    void startSingleCapture();
    void startTimelapse(double interval);
    void stopCapture();

    // Background thread for timelapse
    void timelapseThread();

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sequoia_camera::msg::SunshineSensor>::SharedPtr sunshine_pub_;

    // Python wrapper for camera communication
    std::unique_ptr<PTPyWrapper> camera_;

    // Services
    rclcpp::Service<sequoia_camera::srv::CameraControl>::SharedPtr camera_control_service_;

    // Timers
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr sunshine_timer_;

    // Parameters
    double imu_rate_;
    double sunshine_rate_;
    bool enable_imu_;
    bool enable_sunshine_;

    // Timelapse control
    std::atomic<bool> timelapse_active_;
    std::atomic<bool> shutdown_requested_;
    double timelapse_interval_;
    std::unique_ptr<std::thread> timelapse_thread_;
  };

} // namespace sequoia_camera

#endif // SEQUOIA_CAMERA_COMPONENT_HPP_
