#include "sequoia_camera/ptpy_wrapper.hpp"
#include <iostream>
#include <stdexcept>
#include <dlfcn.h>

namespace sequoia_camera
{

  PTPyWrapper::PTPyWrapper()
      : camera_obj_(nullptr),
        session_context_(nullptr),
        ptpy_module_(nullptr),
        initialized_(false)
  {
    // Initialize Python interpreter
    if (!Py_IsInitialized())
    {
      // CRITICAL: Load libpython with RTLD_GLOBAL to export Python symbols globally
      // This allows Python C extension modules to resolve symbols like PyFloat_Type
      // Use RTLD_NOLOAD first to check if already loaded, otherwise load it
      void *python_lib = dlopen(nullptr, RTLD_NOW | RTLD_GLOBAL | RTLD_NOLOAD);

// If not already loaded with GLOBAL, we need to reload the Python symbols globally
// The Python library is already linked to our library, so this gets the same handle
// but with RTLD_GLOBAL flag which exports symbols for dynamic loading
#if defined(__x86_64__) || defined(_M_X64)
      const char *python_so = "libpython3.10.so.1.0";
#elif defined(__aarch64__) || defined(_M_ARM64)
      const char *python_so = "libpython3.10.so.1.0"; // Same on ARM
#else
      const char *python_so = "libpython3.10.so"; // Fallback
#endif

      python_lib = dlopen(python_so, RTLD_NOW | RTLD_GLOBAL | RTLD_NOLOAD);
      if (!python_lib)
      {
        // Library not loaded yet, load it now
        python_lib = dlopen(python_so, RTLD_NOW | RTLD_GLOBAL);
        if (!python_lib)
        {
          std::cerr << "Warning: Failed to preload Python library with RTLD_GLOBAL: "
                    << dlerror() << std::endl;
        }
      }

      // Use Py_InitializeEx(0) to avoid signal handler issues in embedded mode
      Py_InitializeEx(0);
    }
  }

  PTPyWrapper::~PTPyWrapper()
  {
    shutdown();
    cleanup_python();
  }

  bool PTPyWrapper::initialize(bool disable_gps, bool disable_wifi)
  {
    if (initialized_)
    {
      return true;
    }

    if (!import_ptpy())
    {
      std::cerr << "Failed to import ptpy module" << std::endl;
      return false;
    }

    // Create camera object: camera = ptpy.PTPy()
    PyObject *ptpy_class = PyObject_GetAttrString(ptpy_module_, "PTPy");
    if (!ptpy_class)
    {
      PyErr_Print();
      std::cerr << "Failed to get PTPy class" << std::endl;
      return false;
    }

    camera_obj_ = PyObject_CallObject(ptpy_class, nullptr);
    Py_DECREF(ptpy_class);

    if (!camera_obj_)
    {
      PyErr_Print();
      std::cerr << "Failed to create PTPy camera object" << std::endl;
      return false;
    }

    // Start session: with camera.session():
    PyObject *session_method = PyObject_GetAttrString(camera_obj_, "session");
    if (!session_method)
    {
      PyErr_Print();
      std::cerr << "Failed to get session method" << std::endl;
      return false;
    }

    session_context_ = PyObject_CallObject(session_method, nullptr);
    Py_DECREF(session_method);

    if (!session_context_)
    {
      PyErr_Print();
      std::cerr << "Failed to create session context" << std::endl;
      return false;
    }

    // Enter context manager (__enter__)
    PyObject *enter_method = PyObject_GetAttrString(session_context_, "__enter__");
    if (enter_method)
    {
      PyObject *result = PyObject_CallObject(enter_method, nullptr);
      Py_XDECREF(result);
      Py_DECREF(enter_method);
    }

    // Mark as initialized before configuring device properties
    initialized_ = true;

    // Configure power saving
    if (disable_wifi)
    {
      std::cout << "Attempting to disable WiFi..." << std::endl;
      if (set_wifi_status(false))
      {
        std::cout << "WiFi disabled for power saving" << std::endl;
      }
      else
      {
        std::cout << "Failed to disable WiFi" << std::endl;
      }
    }

    if (disable_gps)
    {
      std::cout << "Attempting to disable GPS..." << std::endl;
      if (set_gps_enable(false))
      {
        std::cout << "GPS disabled for power saving" << std::endl;
      }
      else
      {
        std::cout << "Failed to disable GPS" << std::endl;
      }
    }

    std::cout << "PTPy camera initialized successfully" << std::endl;
    return true;
  }

  void PTPyWrapper::shutdown()
  {
    if (!initialized_)
    {
      return;
    }

    // Exit context manager (__exit__)
    if (session_context_)
    {
      PyObject *exit_method = PyObject_GetAttrString(session_context_, "__exit__");
      if (exit_method)
      {
        PyObject *args = Py_BuildValue("(OOO)", Py_None, Py_None, Py_None);
        PyObject *result = PyObject_CallObject(exit_method, args);
        Py_XDECREF(result);
        Py_DECREF(args);
        Py_DECREF(exit_method);
      }
      Py_DECREF(session_context_);
      session_context_ = nullptr;
    }

    if (camera_obj_)
    {
      Py_DECREF(camera_obj_);
      camera_obj_ = nullptr;
    }

    initialized_ = false;
  }

  bool PTPyWrapper::initiate_capture()
  {
    if (!initialized_ || !camera_obj_)
    {
      std::cerr << "Camera not initialized" << std::endl;
      return false;
    }

    // Call camera.initiate_capture()
    PyObject *capture_method = PyObject_GetAttrString(camera_obj_, "initiate_capture");
    if (!capture_method)
    {
      PyErr_Print();
      std::cerr << "initiate_capture: Failed to get initiate_capture method" << std::endl;
      return false;
    }

    PyObject *result = PyObject_CallObject(capture_method, nullptr);
    Py_DECREF(capture_method);

    if (!result)
    {
      // Check if this was a timeout or device busy error
      if (PyErr_Occurred())
      {
        PyObject *ptype, *pvalue, *ptraceback;
        PyErr_Fetch(&ptype, &pvalue, &ptraceback);

        if (pvalue)
        {
          PyObject *pstr = PyObject_Str(pvalue);
          if (pstr && PyUnicode_Check(pstr))
          {
            const char *err_msg = PyUnicode_AsUTF8(pstr);
            if (err_msg)
            {
              std::string err_str(err_msg);
              // Check for device busy error
              if (err_str.find("Device_Busy") != std::string::npos ||
                  err_str.find("DeviceBusy") != std::string::npos)
              {
                std::cerr << "initiate_capture: Camera is busy (previous operation not complete)" << std::endl;
                Py_XDECREF(pstr);
                Py_XDECREF(ptype);
                Py_XDECREF(pvalue);
                Py_XDECREF(ptraceback);
                return false;
              }
            }
          }
          Py_XDECREF(pstr);
        }

        // Restore error and print it
        PyErr_Restore(ptype, pvalue, ptraceback);
        PyErr_Print();
      }
      return false;
    }

    // Check ResponseCode attribute
    PyObject *response_code = PyObject_GetAttrString(result, "ResponseCode");
    bool success = false;

    if (response_code && PyUnicode_Check(response_code))
    {
      const char *code_str = PyUnicode_AsUTF8(response_code);
      if (code_str)
      {
        std::string code_str_s(code_str);
        if (code_str_s == "OK")
        {
          success = true;
        }
        else
        {
          std::cerr << "initiate_capture: Camera returned: " << code_str_s << std::endl;
        }
      }
    }

    Py_XDECREF(response_code);
    Py_DECREF(result);
    return success;
  }

  bool PTPyWrapper::get_imu_values(IMUData &data)
  {
    if (!initialized_ || !camera_obj_)
    {
      return false;
    }

    // Call camera.get_imu_values()
    PyObject *imu_method = PyObject_GetAttrString(camera_obj_, "get_imu_values");
    if (!imu_method)
    {
      PyErr_Print();
      return false;
    }

    PyObject *result = PyObject_CallObject(imu_method, nullptr);
    Py_DECREF(imu_method);

    if (!result)
    {
      PyErr_Print();
      return false;
    }

    // Extract IMU data from Container object
    // Result structure: Container(Gyroscope, Accelerometer, Magnetometer, Angle)
    PyObject *gyro = PyObject_GetAttrString(result, "Gyroscope");
    PyObject *accel = PyObject_GetAttrString(result, "Accelerometer");
    PyObject *mag = PyObject_GetAttrString(result, "Magnetometer");
    PyObject *angle = PyObject_GetAttrString(result, "Angle");

    if (gyro && accel && mag && angle)
    {
      // Extract gyroscope
      PyObject *gyro_x = PyObject_GetAttrString(gyro, "X");
      PyObject *gyro_y = PyObject_GetAttrString(gyro, "Y");
      PyObject *gyro_z = PyObject_GetAttrString(gyro, "Z");

      // Extract accelerometer
      PyObject *accel_x = PyObject_GetAttrString(accel, "X");
      PyObject *accel_y = PyObject_GetAttrString(accel, "Y");
      PyObject *accel_z = PyObject_GetAttrString(accel, "Z");

      // Extract magnetometer
      PyObject *mag_x = PyObject_GetAttrString(mag, "X");
      PyObject *mag_y = PyObject_GetAttrString(mag, "Y");
      PyObject *mag_z = PyObject_GetAttrString(mag, "Z");

      // Extract angles
      PyObject *yaw = PyObject_GetAttrString(angle, "Yaw");
      PyObject *pitch = PyObject_GetAttrString(angle, "Pitch");
      PyObject *roll = PyObject_GetAttrString(angle, "Roll");

      // Convert to uint32
      data.gyro_x = extract_uint32_from_pyobject(gyro_x);
      data.gyro_y = extract_uint32_from_pyobject(gyro_y);
      data.gyro_z = extract_uint32_from_pyobject(gyro_z);

      data.accel_x = extract_uint32_from_pyobject(accel_x);
      data.accel_y = extract_uint32_from_pyobject(accel_y);
      data.accel_z = extract_uint32_from_pyobject(accel_z);

      data.mag_x = extract_uint32_from_pyobject(mag_x);
      data.mag_y = extract_uint32_from_pyobject(mag_y);
      data.mag_z = extract_uint32_from_pyobject(mag_z);

      data.yaw = extract_uint32_from_pyobject(yaw);
      data.pitch = extract_uint32_from_pyobject(pitch);
      data.roll = extract_uint32_from_pyobject(roll);

      // Clean up
      Py_XDECREF(gyro_x);
      Py_XDECREF(gyro_y);
      Py_XDECREF(gyro_z);
      Py_XDECREF(accel_x);
      Py_XDECREF(accel_y);
      Py_XDECREF(accel_z);
      Py_XDECREF(mag_x);
      Py_XDECREF(mag_y);
      Py_XDECREF(mag_z);
      Py_XDECREF(yaw);
      Py_XDECREF(pitch);
      Py_XDECREF(roll);
    }

    Py_XDECREF(gyro);
    Py_XDECREF(accel);
    Py_XDECREF(mag);
    Py_XDECREF(angle);
    Py_DECREF(result);

    return true;
  }

  bool PTPyWrapper::get_sunshine_values(SunshineData &data)
  {
    if (!initialized_ || !camera_obj_)
    {
      return false;
    }

    // Call camera.get_sunshine_values()
    PyObject *sunshine_method = PyObject_GetAttrString(camera_obj_, "get_sunshine_values");
    if (!sunshine_method)
    {
      PyErr_Print();
      return false;
    }

    PyObject *result = PyObject_CallObject(sunshine_method, nullptr);
    Py_DECREF(sunshine_method);

    if (!result)
    {
      PyErr_Print();
      return false;
    }

    // Extract sunshine data from Container object
    // Result structure: Container(Green, Red, RedEdge, NIR) - each is tuple of 2 values
    PyObject *green = PyObject_GetAttrString(result, "Green");
    PyObject *red = PyObject_GetAttrString(result, "Red");
    PyObject *red_edge = PyObject_GetAttrString(result, "RedEdge");
    PyObject *nir = PyObject_GetAttrString(result, "NIR");

    if (green && red && red_edge && nir)
    {
      // Extract tuples
      if (PyTuple_Check(green) && PyTuple_Size(green) == 2)
      {
        data.green[0] = extract_uint32_from_pyobject(PyTuple_GetItem(green, 0));
        data.green[1] = extract_uint32_from_pyobject(PyTuple_GetItem(green, 1));
      }

      if (PyTuple_Check(red) && PyTuple_Size(red) == 2)
      {
        data.red[0] = extract_uint32_from_pyobject(PyTuple_GetItem(red, 0));
        data.red[1] = extract_uint32_from_pyobject(PyTuple_GetItem(red, 1));
      }

      if (PyTuple_Check(red_edge) && PyTuple_Size(red_edge) == 2)
      {
        data.red_edge[0] = extract_uint32_from_pyobject(PyTuple_GetItem(red_edge, 0));
        data.red_edge[1] = extract_uint32_from_pyobject(PyTuple_GetItem(red_edge, 1));
      }

      if (PyTuple_Check(nir) && PyTuple_Size(nir) == 2)
      {
        data.nir[0] = extract_uint32_from_pyobject(PyTuple_GetItem(nir, 0));
        data.nir[1] = extract_uint32_from_pyobject(PyTuple_GetItem(nir, 1));
      }
    }

    Py_XDECREF(green);
    Py_XDECREF(red);
    Py_XDECREF(red_edge);
    Py_XDECREF(nir);
    Py_DECREF(result);

    return true;
  }

  std::string PTPyWrapper::get_device_info()
  {
    if (!initialized_ || !camera_obj_)
    {
      return "Camera not initialized";
    }

    // Call camera.get_device_info()
    PyObject *info_method = PyObject_GetAttrString(camera_obj_, "get_device_info");
    if (!info_method)
    {
      PyErr_Print();
      return "Failed to get device info method";
    }

    PyObject *result = PyObject_CallObject(info_method, nullptr);
    Py_DECREF(info_method);

    if (!result)
    {
      PyErr_Print();
      return "Failed to get device info";
    }

    // Extract useful information
    std::string info = "Device Info:\n";

    PyObject *manufacturer = PyObject_GetAttrString(result, "Manufacturer");
    PyObject *model = PyObject_GetAttrString(result, "Model");
    PyObject *version = PyObject_GetAttrString(result, "DeviceVersion");
    PyObject *serial = PyObject_GetAttrString(result, "SerialNumber");

    if (manufacturer && PyUnicode_Check(manufacturer))
    {
      const char *str = PyUnicode_AsUTF8(manufacturer);
      if (str)
        info += "  Manufacturer: " + std::string(str) + "\n";
    }

    if (model && PyUnicode_Check(model))
    {
      const char *str = PyUnicode_AsUTF8(model);
      if (str)
        info += "  Model: " + std::string(str) + "\n";
    }

    if (version && PyUnicode_Check(version))
    {
      const char *str = PyUnicode_AsUTF8(version);
      if (str)
        info += "  Version: " + std::string(str) + "\n";
    }

    if (serial && PyUnicode_Check(serial))
    {
      const char *str = PyUnicode_AsUTF8(serial);
      if (str)
        info += "  Serial: " + std::string(str);
    }

    Py_XDECREF(manufacturer);
    Py_XDECREF(model);
    Py_XDECREF(version);
    Py_XDECREF(serial);
    Py_DECREF(result);

    return info;
  }

  bool PTPyWrapper::import_ptpy()
  {
    // Import ptpy module
    ptpy_module_ = PyImport_ImportModule("ptpy");
    if (!ptpy_module_)
    {
      PyErr_Print();
      return false;
    }
    return true;
  }

  void PTPyWrapper::cleanup_python()
  {
    if (ptpy_module_)
    {
      Py_DECREF(ptpy_module_);
      ptpy_module_ = nullptr;
    }
  }

  uint32_t PTPyWrapper::extract_uint32_from_pyobject(PyObject *obj)
  {
    if (!obj)
    {
      return 0;
    }

    if (PyLong_Check(obj))
    {
      return static_cast<uint32_t>(PyLong_AsUnsignedLong(obj));
    }
    else if (PyFloat_Check(obj))
    {
      return static_cast<uint32_t>(PyFloat_AsDouble(obj));
    }

    return 0;
  }

  bool PTPyWrapper::set_gps_enable(bool enable)
  {
    if (!initialized_ || !camera_obj_)
    {
      std::cerr << "set_gps_enable: Camera not initialized" << std::endl;
      return false;
    }

    // Call camera.set_device_prop_value('GPSEnable', enable)
    PyObject *method = PyObject_GetAttrString(camera_obj_, "set_device_prop_value");
    if (!method)
    {
      std::cerr << "set_gps_enable: Failed to get set_device_prop_value method" << std::endl;
      PyErr_Print();
      return false;
    }

    PyObject *args = Py_BuildValue("(si)", "GPSEnable", enable ? 1 : 0);
    PyObject *result = PyObject_CallObject(method, args);

    Py_DECREF(args);
    Py_DECREF(method);

    if (!result)
    {
      std::cerr << "set_gps_enable: Failed to call set_device_prop_value" << std::endl;
      PyErr_Print();
      return false;
    }

    Py_DECREF(result);
    return true;
  }

  bool PTPyWrapper::set_wifi_status(bool enable)
  {
    if (!initialized_ || !camera_obj_)
    {
      std::cerr << "set_wifi_status: Camera not initialized" << std::endl;
      return false;
    }
    PyObject *method = PyObject_GetAttrString(camera_obj_, "set_device_prop_value");

    if (!method)

    {

      std::cerr << "set_wifi_status: Failed to get set_device_prop_value method" << std::endl;

      PyErr_Print();

      return false;
    }

    PyObject *args = Py_BuildValue("(ss)", "WifiStatus", enable ? "ON" : "OFF");
    PyObject *result = PyObject_CallObject(method, args);

    if (!result)
    {
      std::cerr << "set_wifi_status: Failed to call set_device_prop_value (Python Exception)" << std::endl;
      PyErr_Print();
      Py_DECREF(args);
      Py_DECREF(method);
      return false;
    }

    if (result == Py_False)
    {
      std::cerr << "set_wifi_status: Camera refused command (returned False)" << std::endl;
      Py_DECREF(args);
      Py_DECREF(method);
      Py_DECREF(result);
      return false;
    }

    Py_DECREF(args);
    Py_DECREF(method);
    Py_DECREF(result);
    return true;
  }

  bool PTPyWrapper::set_still_capture_mode_single()
  {
    if (!initialized_ || !camera_obj_)
    {
      std::cerr << "set_still_capture_mode_single: Camera not initialized" << std::endl;
      return false;
    }

    // Set StillCaptureMode to 0x0001 (Single Shot)
    // Standard PTP value for single capture mode
    PyObject *method = PyObject_GetAttrString(camera_obj_, "set_device_prop_value");
    if (!method)
    {
      std::cerr << "set_still_capture_mode_single: Failed to get set_device_prop_value method" << std::endl;
      PyErr_Print();
      return false;
    }

    PyObject *args = Py_BuildValue("(si)", "StillCaptureMode", 0x0001);
    PyObject *result = PyObject_CallObject(method, args);

    Py_DECREF(args);
    Py_DECREF(method);

    if (!result)
    {
      std::cerr << "set_still_capture_mode_single: Failed to set StillCaptureMode" << std::endl;
      PyErr_Print();
      return false;
    }

    Py_DECREF(result);
    std::cout << "Camera set to single capture mode" << std::endl;
    return true;
  }

  bool PTPyWrapper::set_still_capture_mode_timelapse(double interval_seconds)
  {
    if (!initialized_ || !camera_obj_)
    {
      std::cerr << "set_still_capture_mode_timelapse: Camera not initialized" << std::endl;
      return false;
    }

    // For timelapse mode, we use TimelapseInterval property
    // Note: The Sequoia camera timelapse mode has issues with automatic captures,
    // so we set it to single shot mode and handle timing externally.

    // Set StillCaptureMode to single shot for manual timelapse control
    if (!set_still_capture_mode_single())
    {
      return false;
    }

    std::cout << "Timelapse mode ready (interval: " << interval_seconds << "s)" << std::endl;
    return true;
  }

  bool PTPyWrapper::set_still_capture_mode_stop()
  {
    if (!initialized_ || !camera_obj_)
    {
      std::cerr << "set_still_capture_mode_stop: Camera not initialized" << std::endl;
      return false;
    }

    // Just ensure we're in single shot mode (idle state)
    return set_still_capture_mode_single();
  }

  bool PTPyWrapper::set_media_folder_name(const std::string& folder_name)
  {
    if (!initialized_ || !camera_obj_)
    {
      std::cerr << "set_media_folder_name: Camera not initialized" << std::endl;
      return false;
    }

    // Call camera.set_device_prop_value('MediaFolderName', folder_name)
    PyObject *method = PyObject_GetAttrString(camera_obj_, "set_device_prop_value");
    if (!method)
    {
      std::cerr << "set_media_folder_name: Failed to get set_device_prop_value method" << std::endl;
      PyErr_Print();
      return false;
    }

    PyObject *args = Py_BuildValue("(ss)", "MediaFolderName", folder_name.c_str());
    PyObject *result = PyObject_CallObject(method, args);

    Py_DECREF(args);
    Py_DECREF(method);

    if (!result)
    {
      std::cerr << "set_media_folder_name: Failed to set folder name" << std::endl;
      PyErr_Print();
      return false;
    }

    // Check if the response indicates success
    PyObject *response_code = PyObject_GetAttrString(result, "ResponseCode");
    bool success = false;

    if (response_code && PyUnicode_Check(response_code))
    {
      const char *code_str = PyUnicode_AsUTF8(response_code);
      if (code_str && std::string(code_str) == "OK")
      {
        success = true;
        std::cout << "set_media_folder_name: Folder name set to '" << folder_name << "'" << std::endl;
      }
      else
      {
        std::cerr << "set_media_folder_name: Camera returned: " << (code_str ? code_str : "unknown") << std::endl;
      }
    }

    Py_XDECREF(response_code);
    Py_DECREF(result);
    return success;
  }

} // namespace sequoia_camera
