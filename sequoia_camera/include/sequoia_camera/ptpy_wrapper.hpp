#ifndef SEQUOIA_CAMERA__PTPY_WRAPPER_HPP_
#define SEQUOIA_CAMERA__PTPY_WRAPPER_HPP_

#include <Python.h>
#include <memory>
#include <string>
#include <array>

namespace sequoia_camera
{

struct IMUData
{
  // Gyroscope: Array of 3 INT32 [X, Y, Z]
  // Unit: microradian per second
  int32_t gyro_x;
  int32_t gyro_y;
  int32_t gyro_z;

  // Accelerometer: Array of 3 INT32 [X, Y, Z]
  // Unit: micrometer per second squared (μm.s-2)
  int32_t accel_x;
  int32_t accel_y;
  int32_t accel_z;

  // Magnetometer: Array of 3 INT32 [X, Y, Z]
  // Unit: nanotesla
  int32_t mag_x;
  int32_t mag_y;
  int32_t mag_z;

  // Orientation angles: Array of INT32
  // Unit: microdegree
  int32_t yaw;
  int32_t pitch;
  int32_t roll;
};

struct SunshineData
{
  // Raw UInt32 values - each sensor has two values
  std::array<uint32_t, 2> green;
  std::array<uint32_t, 2> red;
  std::array<uint32_t, 2> red_edge;
  std::array<uint32_t, 2> nir;
};

class PTPyWrapper
{
public:
  PTPyWrapper();
  ~PTPyWrapper();

  // Initialize camera connection
  bool initialize(bool disable_gps = true, bool disable_wifi = true);

  // Get device information
  std::string get_device_info();

  // Close camera connection
  void shutdown();

  // Camera capture operations
  bool initiate_capture();

  // Capture mode control
  bool set_still_capture_mode_single();
  bool set_still_capture_mode_timelapse(double interval_seconds);
  bool set_still_capture_mode_stop();

  // Sensor data retrieval
  bool get_imu_values(IMUData & data);
  bool get_sunshine_values(SunshineData & data);

  // Power management
  bool set_gps_enable(bool enable);
  bool set_wifi_status(bool enable);

  // Storage management
  bool set_media_folder_name(const std::string& folder_name);

  // Check if camera is connected
  bool is_connected() const { return camera_obj_ != nullptr; }

private:
  PyObject * camera_obj_;
  PyObject * session_context_;
  PyObject * ptpy_module_;
  bool initialized_;

  // Helper methods
  bool import_ptpy();
  void cleanup_python();
  uint32_t extract_uint32_from_pyobject(PyObject * obj);
};

}  // namespace sequoia_camera

#endif  // SEQUOIA_CAMERA__PTPY_WRAPPER_HPP_
