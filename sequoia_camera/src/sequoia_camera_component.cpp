#include "sequoia_camera/sequoia_camera_component.hpp"
#include <cmath>

namespace sequoia_camera
{

  SequoiaCameraComponent::SequoiaCameraComponent(const rclcpp::NodeOptions &options)
      : Node("sequoia_camera", options),
        timelapse_active_(false),
        shutdown_requested_(false),
        timelapse_interval_(1.0)
  {
    // Declare parameters
    this->declare_parameter<double>("imu_rate", 1.0);
    this->declare_parameter<double>("sunshine_rate", 1.0);
    this->declare_parameter<bool>("enable_imu", true);
    this->declare_parameter<bool>("enable_sunshine", true);

    // Get parameters
    imu_rate_ = this->get_parameter("imu_rate").as_double();
    sunshine_rate_ = this->get_parameter("sunshine_rate").as_double();
    enable_imu_ = this->get_parameter("enable_imu").as_bool();
    enable_sunshine_ = this->get_parameter("enable_sunshine").as_bool();

    // Initialize camera wrapper
    RCLCPP_INFO(this->get_logger(), "Initializing Sequoia camera connection...");
    camera_ = std::make_unique<PTPyWrapper>();

    if (!camera_->initialize())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Sequoia camera. No USB PTP device found");
      throw std::runtime_error("Camera initialization failed - no device found");
    }

    // Get and log device information
    std::string device_info = camera_->get_device_info();
    RCLCPP_INFO(this->get_logger(), "Camera connected successfully!");
    RCLCPP_INFO(this->get_logger(), "\n%s", device_info.c_str());

    // Create services
    camera_control_service_ = this->create_service<sequoia_camera::srv::CameraControl>(
        "~/camera_control",
        std::bind(&SequoiaCameraComponent::handleCameraControl, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Create timers for periodic publishing
    if (enable_imu_)
    {
      imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu/data", 10);
      mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("~/mag/data", 10);
      auto imu_period = std::chrono::duration<double>(1.0 / imu_rate_);
      imu_timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(imu_period),
          std::bind(&SequoiaCameraComponent::publishImuData, this));
      RCLCPP_INFO(this->get_logger(), "IMU publishing enabled at %.2f Hz", imu_rate_);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "IMU publishing disabled");
    }

    if (enable_sunshine_)
    {
      sunshine_pub_ = this->create_publisher<sequoia_camera::msg::SunshineSensor>("~/sunshine/data", 10);

      auto sunshine_period = std::chrono::duration<double>(1.0 / sunshine_rate_);
      sunshine_timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(sunshine_period),
          std::bind(&SequoiaCameraComponent::publishSunshineData, this));
      RCLCPP_INFO(this->get_logger(), "Sunshine sensor publishing enabled at %.2f Hz", sunshine_rate_);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Sunshine sensor publishing disabled");
    }

    RCLCPP_INFO(this->get_logger(), "Sequoia Camera Component ready");
  }

  SequoiaCameraComponent::~SequoiaCameraComponent()
  {
    shutdown_requested_ = true;
    if (timelapse_thread_ && timelapse_thread_->joinable())
    {
      timelapse_thread_->join();
    }
    RCLCPP_INFO(this->get_logger(), "Sequoia Camera Node shutting down");
  }

  void SequoiaCameraComponent::publishImuData()
  {
    if (!camera_ || !camera_->is_connected() || imu_pub_->get_subscription_count() == 0)
    {
      return;
    }

    IMUData imu_data;
    if (!camera_->get_imu_values(imu_data))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Failed to get IMU data from camera");
      return;
    }

    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "sequoia_imu";

    // Gyroscope = microradian/s -> radians/s
    constexpr double gyro_scale = 1.0e-6;
    imu_msg.angular_velocity.x = static_cast<double>(imu_data.gyro_x) * gyro_scale;
    imu_msg.angular_velocity.y = static_cast<double>(imu_data.gyro_y) * gyro_scale;
    imu_msg.angular_velocity.z = static_cast<double>(imu_data.gyro_z) * gyro_scale;

    // Accelerometer : micrometer/s^2 -> meters/s^2
    constexpr double accel_scale = 1.0e-6;
    imu_msg.linear_acceleration.x = static_cast<double>(imu_data.accel_x) * accel_scale;
    imu_msg.linear_acceleration.y = static_cast<double>(imu_data.accel_y) * accel_scale;
    imu_msg.linear_acceleration.z = static_cast<double>(imu_data.accel_z) * accel_scale;

    // Convert angles from microdegree to radians
    constexpr double microdeg_to_rad = 1.0e-6 * (M_PI / 180.0);
    double roll = static_cast<double>(imu_data.roll) * microdeg_to_rad;
    double pitch = static_cast<double>(imu_data.pitch) * microdeg_to_rad;
    double yaw = static_cast<double>(imu_data.yaw) * microdeg_to_rad;

    // Convert roll, pitch, yaw to quaternion (using ZYX intrinsic rotation order)
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

    imu_pub_->publish(imu_msg);

    auto mag_msg = sensor_msgs::msg::MagneticField();
    mag_msg.header.stamp = imu_msg.header.stamp;
    mag_msg.header.frame_id = "sequoia_imu"; // Usually same frame as IMU

    // Magnetometer: nanotesla -> Tesla
    constexpr double mag_scale = 1.0e-9;

    mag_msg.magnetic_field.x = static_cast<double>(imu_data.mag_x) * mag_scale;
    mag_msg.magnetic_field.y = static_cast<double>(imu_data.mag_y) * mag_scale;
    mag_msg.magnetic_field.z = static_cast<double>(imu_data.mag_z) * mag_scale;

    mag_pub_->publish(mag_msg);
  }

  void SequoiaCameraComponent::publishSunshineData()
  {
    if (!camera_ || !camera_->is_connected() || sunshine_pub_->get_subscription_count() == 0)
    {
      return;
    }

    SunshineData sunshine_data;
    if (!camera_->get_sunshine_values(sunshine_data))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Failed to get sunshine sensor data from camera");
      return;
    }

    auto sunshine_msg = sequoia_camera::msg::SunshineSensor();
    sunshine_msg.header.stamp = this->now();
    sunshine_msg.header.frame_id = "sequoia_sunshine";

    // Cast UINT32 to double for ROS message
    sunshine_msg.green[0] = static_cast<double>(sunshine_data.green[0]);
    sunshine_msg.green[1] = static_cast<double>(sunshine_data.green[1]);

    sunshine_msg.red[0] = static_cast<double>(sunshine_data.red[0]);
    sunshine_msg.red[1] = static_cast<double>(sunshine_data.red[1]);

    sunshine_msg.red_edge[0] = static_cast<double>(sunshine_data.red_edge[0]);
    sunshine_msg.red_edge[1] = static_cast<double>(sunshine_data.red_edge[1]);

    sunshine_msg.nir[0] = static_cast<double>(sunshine_data.nir[0]);
    sunshine_msg.nir[1] = static_cast<double>(sunshine_data.nir[1]);

    sunshine_pub_->publish(sunshine_msg);
  }

  void SequoiaCameraComponent::handleCameraControl(
      const std::shared_ptr<sequoia_camera::srv::CameraControl::Request> request,
      std::shared_ptr<sequoia_camera::srv::CameraControl::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Camera control request received - mode: %d", request->mode);

    try
    {
      // NOTE: Cause camera to hang for some reason!!!
      // if (!request->folder_name.empty())
      // {
      //   if (camera_->set_media_folder_name(request->folder_name))
      //   {
      //     RCLCPP_INFO(this->get_logger(), "Media folder name set to: %s", request->folder_name.c_str());
      //   }
      //   else
      //   {
      //     RCLCPP_WARN(this->get_logger(), "Failed to set media folder name to: %s", request->folder_name.c_str());
      //   }
      // }

      switch (request->mode)
      {
      case sequoia_camera::srv::CameraControl::Request::MODE_STOP:
        stopCapture();
        response->success = true;
        response->message = "Capture stopped successfully";
        break;

      case sequoia_camera::srv::CameraControl::Request::MODE_SINGLE:
        startSingleCapture();
        response->success = true;
        response->message = "Single capture triggered";
        break;

      case sequoia_camera::srv::CameraControl::Request::MODE_TIMELAPSE:
        if (request->timelapse_interval <= 0.0)
        {
          response->success = false;
          response->message = "Invalid timelapse interval. Must be > 0";
        }
        else
        {
          startTimelapse(request->timelapse_interval);
          response->success = true;
          response->message = "Timelapse started with interval: " +
                              std::to_string(request->timelapse_interval) + "s";
        }
        break;

      default:
        response->success = false;
        response->message = "Invalid mode. Use MODE_STOP(0), MODE_SINGLE(1), or MODE_TIMELAPSE(2)";
      }
    }
    catch (const std::exception &e)
    {
      response->success = false;
      response->message = std::string("Error: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Camera control error: %s", e.what());
    }

    RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
  }

  void SequoiaCameraComponent::startSingleCapture()
  {
    if (!camera_ || !camera_->is_connected())
    {
      RCLCPP_ERROR(this->get_logger(), "Camera not connected");
      return;
    }

    // Set camera to single capture mode
    if (!camera_->set_still_capture_mode_single())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to set single capture mode");
      return;
    }

    // Small delay to allow camera to process mode change
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (camera_->initiate_capture())
    {
      RCLCPP_INFO(this->get_logger(), "Single capture triggered successfully");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to trigger single capture");
    }
  }

  void SequoiaCameraComponent::startTimelapse(double interval)
  {
    // Stop existing timelapse if running
    stopCapture();

    if (!camera_ || !camera_->is_connected())
    {
      RCLCPP_ERROR(this->get_logger(), "Camera not connected");
      return;
    }

    // Set camera to timelapse mode
    if (!camera_->set_still_capture_mode_timelapse(interval))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to set timelapse mode");
      return;
    }

    timelapse_interval_ = interval;
    timelapse_active_ = true;

    // Start timelapse thread
    timelapse_thread_ = std::make_unique<std::thread>(
        std::bind(&SequoiaCameraComponent::timelapseThread, this));

    RCLCPP_INFO(this->get_logger(), "Timelapse started with %.2fs interval", interval);
  }

  void SequoiaCameraComponent::stopCapture()
  {
    if (timelapse_active_)
    {
      timelapse_active_ = false;
      if (timelapse_thread_ && timelapse_thread_->joinable())
      {
        timelapse_thread_->join();
      }

      // Reset camera to idle mode
      if (camera_ && camera_->is_connected())
      {
        camera_->set_still_capture_mode_stop();
      }

      RCLCPP_INFO(this->get_logger(), "Timelapse stopped");
    }
  }

  void SequoiaCameraComponent::timelapseThread()
  {
    RCLCPP_INFO(this->get_logger(), "Timelapse thread started");

    // Calculate minimum safe interval (found empirically)
    const double min_safe_interval = 10.0;
    double effective_interval = std::max(timelapse_interval_, min_safe_interval);

    if (effective_interval > timelapse_interval_)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Requested interval %.2fs is too short. Using %.2fs for camera stability.",
                  timelapse_interval_, effective_interval);
    }

    // Retry settings for DeviceBusy errors
    const int max_retries = 5;
    const double retry_delay = 1.0; // seconds between retries

    while (timelapse_active_ && !shutdown_requested_)
    {
      if (!camera_ || !camera_->is_connected())
      {
        RCLCPP_ERROR(this->get_logger(), "Camera not connected, stopping timelapse");
        timelapse_active_ = false;
        break;
      }

      auto capture_start = std::chrono::steady_clock::now();

      // Attempt capture with retry logic for DeviceBusy errors
      bool capture_success = false;
      int retry_count = 0;

      while (!capture_success && retry_count < max_retries && timelapse_active_ && !shutdown_requested_)
      {
        capture_success = camera_->initiate_capture();

        if (capture_success)
        {
          RCLCPP_DEBUG(this->get_logger(), "Timelapse capture successful");
        }
        else
        {
          retry_count++;
          if (retry_count < max_retries)
          {
            RCLCPP_WARN(this->get_logger(),
                        "Timelapse capture failed (attempt %d/%d). Retrying in %.1fs...",
                        retry_count, max_retries, retry_delay);
            std::this_thread::sleep_for(std::chrono::duration<double>(retry_delay));
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(),
                         "Timelapse capture failed after %d attempts. Skipping this capture.",
                         max_retries);
          }
        }
      }

      // Calculate time spent on capture (including retries)
      auto capture_end = std::chrono::steady_clock::now();
      auto capture_duration = std::chrono::duration<double>(capture_end - capture_start).count();
      static int idx = 0;
      RCLCPP_INFO(this->get_logger(), "ID: %d, timestamp: %f", idx++, this->get_clock()->now().seconds());
      // Sleep for remaining interval time
      double remaining_time = effective_interval - capture_duration;
      if (remaining_time > 0.1) // Only sleep if more than 100ms remaining
      {
        auto sleep_duration = std::chrono::duration<double>(remaining_time);
        std::this_thread::sleep_for(sleep_duration);
      }
      else if (capture_duration > effective_interval)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                             "Capture took %.2fs, longer than interval %.2fs",
                             capture_duration, effective_interval);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Timelapse thread stopped");
  }

} // namespace sequoia_camera
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sequoia_camera::SequoiaCameraComponent)
