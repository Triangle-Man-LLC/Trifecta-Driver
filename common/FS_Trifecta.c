/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
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
#include "FS_Trifecta_Defs.h"
#include "FS_Trifecta_Interfaces.h"
#include "FS_Trifecta_Networked.h"
#include "FS_Trifecta_Serial.h"
#include "FS_Trifecta_Device.h"

static inline int fs_send_command(fs_device_info_t *device_handle, void *payload, size_t length)
{
  int result = -1;
  switch (device_handle->communication_mode)
  {
  case FS_COMMUNICATION_MODE_UART:
  case FS_COMMUNICATION_MODE_USB_CDC:
  case FS_COMMUNICATION_MODE_I2C:
  case FS_COMMUNICATION_MODE_SPI:
    result = fs_serial_send_message(device_handle, payload, length);
    break;
  case FS_COMMUNICATION_MODE_TCP_UDP:
    result = fs_network_send_message(device_handle, payload, length);
    break;
  default:
    fs_log_output("[Trifecta] Error: Unsupported communication mode!");
    return -1;
  }
  return result;
}

fs_device_info_t *fs_export_allocate_device()
{
  fs_device_info_t *dev = (fs_device_info_t *)calloc(1, sizeof(fs_device_info_t));
  if (dev)
  {
    *dev = FS_DEVICE_INFO_UNINITIALIZED;
  }
  return dev;
}

void fs_export_free_device(fs_device_info_t *device)
{
  if (device)
  {
    free(device);
  }
}

int fs_set_driver_parameters(fs_device_info_t *device_handle, fs_driver_config_t *dconfig)
{
  if (dconfig == NULL)
  {
    return -1;
  }
  memcpy(&device_handle->driver_config, dconfig, sizeof(fs_driver_config_t));
  return 0;
}

int fs_initialize_networked(fs_device_info_t *device_handle, const char *device_ip_address)
{
  if (device_handle->status == FS_RUN_STATUS_RUNNING)
  {
    fs_log_output("[Trifecta] Error: Driver already running with active device! Shut that one down before continuing.\n");
    return -1;
  }

  if (device_handle->driver_config.task_stack_size_bytes == 0)
  {
    fs_driver_config_t dconfig = FS_DRIVER_CONFIG_DEFAULT;
    if (fs_set_driver_parameters(device_handle, &dconfig) != 0)
    {
      fs_log_output("[Trifecta] Error: Failed to set the driver configuration!\n");
      return -1;
    }
  }

  device_handle->communication_mode = FS_COMMUNICATION_MODE_TCP_UDP;

  if (fs_network_start(device_ip_address, device_handle) != 0)
  {
    fs_log_output("[Trifecta] Error: Could not start networked device!\n");
    device_handle->communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
    return -1;
  }

  if (device_handle->communication_mode == FS_COMMUNICATION_MODE_TCP_UDP)
  {
    fs_log_output("[Trifecta] Opened TCP sock %d and UDP sock %d corresponding to device IPs %s:%d,  %s:%d\n",
                  device_handle->device_params.tcp_sock,
                  device_handle->device_params.udp_sock,
                  device_handle->device_params.ip_addr,
                  device_handle->device_params.tcp_port,
                  device_handle->device_params.ip_addr,
                  device_handle->device_params.udp_port);
    return 0;
  }
  else
  {
    fs_log_output("[Trifecta] Error: Network device initialization failed!\n");
    return -1;
  }
}

int fs_initialize_serial(fs_device_info_t *device_handle, int fd, fs_communication_mode_t serial_mode)
{
  if (device_handle->status == FS_RUN_STATUS_RUNNING)
  {
    fs_log_output("[Trifecta] Error: Driver already running with active device! Shut that one down before continuing.\n");
    return -1;
  }

  if (device_handle->driver_config.task_stack_size_bytes == 0)
  {
    fs_driver_config_t dconfig = FS_DRIVER_CONFIG_DEFAULT;
    if (fs_set_driver_parameters(device_handle, &dconfig) != 0)
    {
      fs_log_output("[Trifecta] Error: Failed to set the driver configuration!\n");
      return -1;
    }
  }

  device_handle->communication_mode = serial_mode;
  device_handle->device_params.serial_port = fd;

  if (fs_serial_start(device_handle) != 0)
  {
    fs_log_output("[Trifecta] Error: Failed to start the serial driver!\n");
    device_handle->communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
    return -1;
  }

  if (device_handle->communication_mode == FS_COMMUNICATION_MODE_UART)
  {
    device_handle->device_params.baudrate = FS_TRIFECTA_SERIAL_BAUDRATE;
    fs_log_output("[Trifecta] Info: Initialized UART driver for device: (Port %d), Baud rate: %d\n",
                  device_handle->device_params.serial_port, device_handle->device_params.baudrate);
    return 0;
  }
  else if (device_handle->communication_mode == FS_COMMUNICATION_MODE_USB_CDC)
  {
    // CDC ACM will be supported on a number of platforms,
    // though note that on Linux/Posix systems they are treated the same as serial ports.
    fs_log_output("[Trifecta] Info: Initialized CDC driver for device: (Port %d), Baud rate: %d\n",
                  device_handle->device_params.serial_port, device_handle->device_params.baudrate);
    return 0;
  }
  else if (device_handle->communication_mode == FS_COMMUNICATION_MODE_I2C)
  {
    // TODO: In I2C mode, use the "baudrate" field to instead store device address.
    // device_handle->device_params.baudrate = FS_TRIFECTA_SERIAL_BAUDRATE;
    fs_log_output("[Trifecta] Info: Initialized I2C driver for device: (Port %d), Baud rate: %d\n",
                  device_handle->device_params.serial_port, device_handle->device_params.baudrate);
    return 0;
  }

  else if (device_handle->communication_mode == FS_COMMUNICATION_MODE_SPI)
  {
    fs_log_output("[Trifecta] Info: Initialized SPI driver for device: (Port %d), Baud rate: %d\n",
                  device_handle->device_params.serial_port, device_handle->device_params.baudrate);
    return 0;
  }
  else
  {
    fs_log_output("[Trifecta] Error: Serial device initialization failed!\n");
    return -1;
  }
}

int fs_start_stream(fs_device_info_t *device_handle)
{
  char send_buf[FS_MAX_CMD_LENGTH];
  snprintf(send_buf, sizeof(send_buf), "%c%d;", CMD_STREAM, 1);
  size_t send_len = fs_safe_strnlen(send_buf, sizeof(send_buf));
  return fs_send_command(device_handle, send_buf, send_len);
}

int fs_stop_stream(fs_device_info_t *device_handle)
{
  char send_buf[FS_MAX_CMD_LENGTH];
  snprintf(send_buf, sizeof(send_buf), "%c%d;", CMD_STREAM, 0);
  size_t send_len = fs_safe_strnlen(send_buf, sizeof(send_buf));
  return fs_send_command(device_handle, send_buf, send_len);
}

int fs_read_one_shot(fs_device_info_t *device_handle)
{
  char send_buf[FS_MAX_CMD_LENGTH];
  snprintf(send_buf, sizeof(send_buf), "%c%d;", CMD_STREAM, 2);
  size_t send_len = fs_safe_strnlen(send_buf, sizeof(send_buf));
  return fs_send_command(device_handle, send_buf, send_len);
}

int fs_reboot_device(fs_device_info_t *device_handle)
{
  char send_buf[FS_MAX_CMD_LENGTH];
  snprintf(send_buf, FS_MAX_CMD_LENGTH, ";%c%d;", CMD_RESTART, 0);
  size_t send_len = fs_safe_strnlen(send_buf, sizeof(send_buf));
  return fs_send_command(device_handle, send_buf, send_len);
}

int fs_enable_logging(bool do_enable)
{
  return fs_toggle_logging(do_enable);
}

int fs_closedown(fs_device_info_t *device_handle)
{
  switch (device_handle->communication_mode)
  {
  case FS_COMMUNICATION_MODE_UART:
  case FS_COMMUNICATION_MODE_USB_CDC:
  case FS_COMMUNICATION_MODE_I2C:
  case FS_COMMUNICATION_MODE_SPI:
    if (fs_serial_exit(device_handle) != 0)
    {
      fs_log_output("[Trifecta] Warning: Closedown of serial driver was abnormal.");
      return -1;
    }
    break;
  case FS_COMMUNICATION_MODE_TCP_UDP:
    if (fs_network_exit(device_handle) != 0)
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
  device_handle->communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
  memset(device_handle, 0, sizeof(fs_device_info_t));
  fs_log_output("[Trifecta] Closedown of driver succeeded, all resources are now released.");
  return 0;
}

int fs_set_ahrs_heading(fs_device_info_t *device_handle, float heading_deg)
{
  char send_buf[FS_MAX_CMD_LENGTH];
  snprintf(send_buf, FS_MAX_CMD_LENGTH, ";%c%.8f;", CMD_SET_YAW_DEG, heading_deg);
  size_t send_len = fs_safe_strnlen(send_buf, sizeof(send_buf));
  return fs_send_command(device_handle, send_buf, send_len);
}

int fs_set_ins_position(fs_device_info_t *device_handle, fs_vector3_t *position)
{
  // char send_buf[FS_MAX_CMD_LENGTH];
  // snprintf(send_buf, FS_MAX_CMD_LENGTH, ";%c%.8f;", CMD_SET_YAW_DEG, heading_deg);
  // size_t send_len = fs_safe_strnlen(send_buf, sizeof(send_buf));
  // return fs_send_command(device_handle, send_buf, send_len);
  return -1; // Not yet supported
}

int fs_set_communication_mode(fs_device_info_t *device_handle, int modes)
{
  char send_buf[FS_MAX_CMD_LENGTH];
  snprintf(send_buf, FS_MAX_CMD_LENGTH, ";%c%d;", CMD_IDENTIFY_PARAM_TRANSMIT, modes);
  size_t send_len = fs_safe_strnlen(send_buf, sizeof(send_buf));
  return fs_send_command(device_handle, send_buf, send_len);
}

int fs_set_network_parameters(fs_device_info_t *device_handle, const char ssid[32], const char pw[64], bool access_point)
{
  char send_buf[2 * FS_MAX_CMD_LENGTH];
  if (access_point)
  {
    snprintf(send_buf, 2 * FS_MAX_CMD_LENGTH, ";%c%s;%c%s", CMD_SET_SSID_AP, ssid, CMD_SET_PASSWORD_AP, pw);
  }
  else
  {
    snprintf(send_buf, 2 * FS_MAX_CMD_LENGTH, ";%c%s;%c%s", CMD_SET_SSID, ssid, CMD_SET_PASSWORD, pw);
  }
  size_t send_len = fs_safe_strnlen(send_buf, sizeof(send_buf));
  return fs_send_command(device_handle, send_buf, send_len);
}

int fs_set_network_udp_port(fs_device_info_t *device_handle, int udp_port)
{
  char send_buf[FS_MAX_CMD_LENGTH];
  if (udp_port < 1024 || udp_port > 65535)
  {
    fs_log_output("[Trifecta] %d is an invalid UDP port!", udp_port);
    return -1;
  }
  snprintf(send_buf, FS_MAX_CMD_LENGTH, ";%c%d;", CMD_SET_LISTENING_PORT, udp_port);
  size_t send_len = fs_safe_strnlen(send_buf, FS_MAX_CMD_LENGTH) + 1;
  return fs_send_command(device_handle, send_buf, send_len);
}

int fs_set_serial_uart_baudrate(fs_device_info_t *device_handle, int baudrate)
{
  char send_buf[FS_MAX_CMD_LENGTH];
  snprintf(send_buf, FS_MAX_CMD_LENGTH, ";%c%d;", CMD_IDENTIFY_PARAM_UART_BAUD_RATE, baudrate);
  size_t send_len = fs_safe_strnlen(send_buf, sizeof(send_buf));
  return fs_send_command(device_handle, send_buf, send_len);
}

int fs_get_device_operating_state(fs_device_info_t *device_handle, fs_device_params_t *device_params_info)
{
  if (device_handle == NULL || device_params_info == NULL)
  {
    return -1;
  }
  char send_buf[FS_MAX_DATA_LENGTH] = {0};
  snprintf(send_buf, sizeof(send_buf),
           ";%c-1;%c-1;%c-1;%c-1;%c-1;%c-1;%c-1;%c-1;%c-1;",
           CMD_IDENTIFY,
           CMD_IDENTIFY_PARAM_DEVFWVERSION,
           CMD_IDENTIFY_PARAM_DEVDESC,
           CMD_IDENTIFY_PARAM_DEV_SN,
           CMD_IDENTIFY_PARAM_TRANSMIT,
           CMD_IDENTIFY_PARAM_SSID,
           CMD_IDENTIFY_PARAM_SSID_AP,
           CMD_IDENTIFY_PARAM_PASSWORD_AP,
           CMD_IDENTIFY_PARAM_UART_BAUD_RATE);
  ssize_t send_len = fs_safe_strnlen(send_buf, sizeof(send_buf));
  return fs_send_command(device_handle, send_buf, send_len) > 0 ? 0 : -1;
}