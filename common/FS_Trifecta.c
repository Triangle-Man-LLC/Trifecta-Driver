/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "FS_Trifecta.h"
#include "FS_Trifecta_Interfaces.h"
#include "FS_Trifecta_Networked.h"
#include "FS_Trifecta_Serial.h"
#include "FS_Trifecta_Device.h"

fs_device_info device_handle = FS_DEVICE_INFO_UNINITIALIZED;
fs_driver_config driver_config = FS_DRIVER_CONFIG_DEFAULT;

/// @brief Start the driver in networked mode, this will attempt to connect to the device at given IP
/// @param device_ip_address String representation of the device IP address
int fs_initialize_networked(fs_driver_config *dconfig, const char *device_ip_address)
{
  if (dconfig != NULL)
  {
    memcpy(&driver_config, dconfig, sizeof(fs_driver_config));
  }

  if (device_handle.communication_mode != FS_COMMUNICATION_MODE_UNINITIALIZED)
  {
    fs_log_output("[Trifecta] Error: Driver already running with active device! Shut that one down before continuing.\n");
    return -1;
  }
  
  if(fs_network_set_driver_config(&driver_config) != 0)
  {
    fs_log_output("[Trifecta] Error: Failed to set the driver configuration!\n");
    return -1;
  }

  device_handle.communication_mode = FS_COMMUNICATION_MODE_TCP_UDP;

  if (fs_network_start(device_ip_address, &device_handle) != 0)
  {
    fs_log_output("[Trifecta] Error: Could not start networked device!\n");
    device_handle.communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
    return -1;
  }

  if (device_handle.communication_mode == FS_COMMUNICATION_MODE_TCP_UDP)
  {
    fs_log_output("[Trifecta] Info: Initialized network driver for device: %s (Port %s), Baud rate: %d\n", device_handle.device_name, device_handle.serial_port, device_handle.baudrate);
    return 0;
  }
  else
  {
    fs_log_output("[Trifecta] Error: Network device initialization failed!\n");
    return -1;
  }
}

/// @brief Start the IMU driver in serial mode
/// @param fd The serial port (file descriptor on POSIX systems, UART_NUM on microcontrollers),
/// if -1 then the device-specific implementation should attempt to scan available ports
/// @return 0 if succeeded
int fs_initialize_serial(fs_driver_config *dconfig, int fd)
{
  if (dconfig != NULL)
  {
    memcpy(&driver_config, dconfig, sizeof(fs_driver_config));
  }

  if (device_handle.communication_mode != FS_COMMUNICATION_MODE_UNINITIALIZED)
  {
    fs_log_output("[Trifecta] Error: Driver already running with active device! Shut that one down before continuing.\n");
    return -1;
  }
  
  if(fs_serial_set_driver_config(&driver_config) != 0)
  {
    fs_log_output("[Trifecta] Error: Failed to set the driver configuration!\n");
    return -1;
  }

  device_handle.communication_mode = FS_COMMUNICATION_MODE_SERIAL;
  device_handle.serial_port = fd;

  if (fs_serial_start(&device_handle) != 0)
  {
    fs_log_output("[Trifecta] Error: Failed to start the serial driver!\n");
    device_handle.communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
    return -1;
  }

  if (device_handle.communication_mode == FS_COMMUNICATION_MODE_SERIAL)
  {
    device_handle.baudrate = FS_TRIFECTA_SERIAL_BAUDRATE;
    fs_log_output("[Trifecta] Info: Initialized serial driver for device: (Port %d), Baud rate: %d\n", device_handle.serial_port, device_handle.baudrate);
    return 0;
  }
  else
  {
    fs_log_output("[Trifecta] Error: Serial device initialization failed!\n");
    return -1;
  }
}

/// @brief Begin device data stream
/// @return 0 if succeeded, -1 if failed
int fs_start_stream()
{
  switch (device_handle.communication_mode)
  {
  case FS_COMMUNICATION_MODE_SERIAL:
    if (fs_serial_start_device_stream(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not start device serial stream!");
      return -1;
    }
    break;
  case FS_COMMUNICATION_MODE_TCP_UDP:
    if (fs_network_start_device_stream(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not start device network stream!");
      return -1;
    }
    break;
  default:
    fs_log_output("[Trifecta] Error: Failed to start stream, driver was not operating!");
    return -1;
    break;
  }
  return 0;
}

/// @brief End device data stream
/// @return 0 if succeeded, -1 if failed
int fs_stop_stream()
{
  switch (device_handle.communication_mode)
  {
  case FS_COMMUNICATION_MODE_SERIAL:
    if (fs_serial_stop_device_stream(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not stop device serial stream!");
      return -1;
    }
    break;
  case FS_COMMUNICATION_MODE_TCP_UDP:
    if (fs_network_stop_device_stream(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not stop device network stream!");
      return -1;
    }
    break;
  default:
    fs_log_output("[Trifecta] Error: Invalid operating mode!");
    return -1;
    break;
  }

  return 0;
}

/// @brief It is helpful to use "one-shot" reading mode instead of continuous streaming instead. 
/// This function reads a single packet into the fs_device buffer, and returns once it is succeeded or timed out.
/// This is generally preferable in most use cases compared to streaming mode, as it negates the need for reconnection
/// logic in the application layer.
/// @return 0 if succeeded, -1 if failed
int fs_read_one_shot()
{
  switch (device_handle.communication_mode)
  {
  case FS_COMMUNICATION_MODE_SERIAL:
    if (fs_serial_read_one_shot(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not stop device serial stream!");
      return -1;
    }
    break;
  case FS_COMMUNICATION_MODE_TCP_UDP:
    if (fs_network_read_one_shot(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not stop device network stream!");
      return -1;
    }
    break;
  default:
    fs_log_output("[Trifecta] Error: Invalid operating mode!");
    return -1;
    break;
  }
  return 0;
}

/// @brief Order the device to restart
/// @return 0 if succeeded, -1 if failed
int fs_reboot_device()
{
  switch (device_handle.communication_mode)
  {
  case FS_COMMUNICATION_MODE_SERIAL:
    if (fs_serial_device_restart(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not restart device!");
      return -1;
    }
    break;
  case FS_COMMUNICATION_MODE_TCP_UDP:
    if (fs_network_device_restart(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not restart device!");
      return -1;
    }
    break;
  default:
    fs_log_output("[Trifecta] Error: Failed to restart device, driver was not operating!");
    return -1;
    break;
  }
  return 0;
}

/// @brief Change the device communication mode. (Doesn't appear to be implemented for now.)
/// @param mode 
/// @return 
int fs_set_communication_mode(fs_communication_mode mode)
{
  switch (device_handle.communication_mode)
  {
  case FS_COMMUNICATION_MODE_SERIAL:
    if (fs_serial_device_restart(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not restart device!");
      return -1;
    }
    break;
  case FS_COMMUNICATION_MODE_TCP_UDP:
    if (fs_network_device_restart(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not restart device!");
      return -1;
    }
    break;
  default:
    fs_log_output("[Trifecta] Error: Failed to restart device, driver was not operating!");
    return -1;
    break;
  }
  return 0;
}

/// @brief 
/// @param ssid 
/// @param password 
/// @return 
int fs_set_wifi_params(char *ssid, char *password)
{
  switch (device_handle.communication_mode)
  {
  case FS_COMMUNICATION_MODE_SERIAL:
    if (fs_serial_set_network_params(&device_handle, ssid, password) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not set device wifi configuration!");
      return -1;
    }
    break;
  case FS_COMMUNICATION_MODE_TCP_UDP:
    if (fs_network_set_network_params(&device_handle, ssid, password) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not set device wifi configuration!");
      return -1;
    }
    break;
  default:
    fs_log_output("[Trifecta] Error: Failed to set device wifi configuration, driver was not operating!");
    return -1;
    break;
  }
  return 0;
}

/// @brief Toggle logging
/// @param do_enable TRUE to enable logging, FALSE to disable
/// @return Logging level
int fs_enable_logging(bool do_enable)
{
  return fs_toggle_logging(do_enable);
}

/// @brief Shut down the device driver
/// @return 0 if succeeded, -1 if failed
int fs_closedown()
{
  switch (device_handle.communication_mode)
  {
  case FS_COMMUNICATION_MODE_SERIAL:
    if (fs_serial_exit(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Warning: Closedown of serial driver was abnormal.");
      return -1;
    }
    break;
  case FS_COMMUNICATION_MODE_TCP_UDP:
    if (fs_network_exit(&device_handle) != 0)
    {
      fs_log_output("[Trifecta] Warning: Closedown of network driver was abnormal.");
      return -1;
    }
    break;
  default:
    fs_log_output("[Trifecta] Warning: Closedown did not need to be performed, driver was not initialized.");
    return -1;
    break;
  }

  device_handle.communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
  memset(&device_handle, 0, sizeof(fs_device_info));

  fs_log_output("[Trifecta] Closedown of driver succeeded, all resources are now released.");
  return 0;
}

/// @brief Manually set the AHRS yaw angle. This should only be done if the device is not operating in GPS mode.
/// @param heading_deg The desired angle
/// @return 0 on success
int fs_set_ahrs_heading(float heading_deg)
{
  char command[16];
  snprintf(command, sizeof(command), "%c%.8f;", CMD_SET_YAW_DEG, heading_deg);
  switch (device_handle.communication_mode)
  {
  case FS_COMMUNICATION_MODE_SERIAL:
    if (fs_serial_send_message(&device_handle, &command, strnlen(command, sizeof(command))) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not set device AHRS heading!");
      return -1;
    }
    break;
  case FS_COMMUNICATION_MODE_TCP_UDP:
    if (fs_network_send_message(&device_handle, &command, strnlen(command, sizeof(command))) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not set device AHRS heading!");
      return -1;
    }
    break;
  default:
    fs_log_output("[Trifecta] Error: Failed to set device AHRS heading, driver was not operating!");
    return -1;
    break;
  }
  return 0;
}

/// @brief Manually set the INS position. Coupled with fs_set_ahrs_heading(), it is possible to perform INS alignment.
/// However, this procedure is usually performed automatically when connected with a compatible GNSS.
/// @param position The position to set to (for now, this function only re-sets position to zero)
/// @return 0 on success
int fs_set_ins_position(fs_vector3 *position)
{
  char command[16];
  snprintf(command, sizeof(command), "%c0;", CMD_REZERO_INS);
  switch (device_handle.communication_mode)
  {
  case FS_COMMUNICATION_MODE_SERIAL:
    if (fs_serial_send_message(&device_handle, &command, strnlen(command, sizeof(command))) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not set INS position!");
      return -1;
    }
    break;
  case FS_COMMUNICATION_MODE_TCP_UDP:
    if (fs_network_send_message(&device_handle, &command, strnlen(command, sizeof(command))) != 0)
    {
      fs_log_output("[Trifecta] Error: Could not set INS position!");
      return -1;
    }
    break;
  default:
    fs_log_output("[Trifecta] Error: Failed to set INS position!");
    return -1;
    break;
  }
  return 0;
}
