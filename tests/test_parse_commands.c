#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "FS_Trifecta.h"
#include "FS_Trifecta_Device.h"
#include "FS_Trifecta_Device_Utils.h"

int fs_logging_level = 1;

int fs_log_output(const char *format, ...)
{
    int chars_printed = 0;
    if (fs_logging_level > 0)
    {
        va_list args;
        va_start(args, format);
        chars_printed = vprintf(format, args);
        if (format[chars_printed - 1] != '\n')
        {
            printf("\n");
            chars_printed++;
        }
        va_end(args);
    }
    return chars_printed;
}

int main()
{
    fs_device_info_t device = FS_DEVICE_INFO_UNINITIALIZED;
    char cmd_buf[FS_MAX_DATA_LENGTH];
    int ret_val = 0;

    // Simulated values
    const char *expected_name = "Trifecta-Test-Harness";
    const int expected_baudrate = 115200;
    const char *expected_ssid = "MyWiFiNetwork";
    const char *expected_ap_ssid = "MyAccessPoint";
    const char *expected_ap_password = "SuperSecretPassword";
    const int expected_comm_mode = 2;
    const char *expected_serial = "SN123456789";
    const char *expected_model = "TrifectaModelX";
    const char *expected_fw = "1.2.3";
    const char *expected_desc = "Test Device Description";

    // Build compound command string
    snprintf(cmd_buf, sizeof(cmd_buf),
             ";%c%s;"
             "%c%d;"
             "%c%s;"
             "%c%s;"
             "%c%s;"
             "%c%d;"
             "%c%s;"
             "%c%s;"
             "%c%s;"
             "%c%s;",
             CMD_IDENTIFY, expected_name,
             CMD_IDENTIFY_PARAM_UART_BAUD_RATE, expected_baudrate,
             CMD_IDENTIFY_PARAM_SSID, expected_ssid,
             CMD_IDENTIFY_PARAM_SSID_AP, expected_ap_ssid,
             CMD_IDENTIFY_PARAM_PASSWORD_AP, expected_ap_password,
             CMD_IDENTIFY_PARAM_TRANSMIT, expected_comm_mode,
             CMD_IDENTIFY_PARAM_DEV_SN, expected_serial,
             CMD_IDENTIFY_PARAM_DEVMODEL, expected_model,
             CMD_IDENTIFY_PARAM_DEVFWVERSION, expected_fw,
             CMD_IDENTIFY_PARAM_DEVDESC, expected_desc);

    // Feed all at once
    fs_handle_received_commands(&device, cmd_buf, strlen(cmd_buf));

    // Validate all fields
    if (strcmp(device.device_descriptor.device_name, expected_name) != 0)
    {
        printf("❌ Device name mismatch: got '%s'\n", device.device_descriptor.device_name);
        ret_val++;
    }

    if (device.device_params.baudrate != expected_baudrate)
    {
        printf("❌ Baudrate mismatch: got %d\n", device.device_params.baudrate);
        ret_val++;
    }

    if (strcmp(device.device_params.ssid, expected_ssid) != 0)
    {
        printf("❌ STA SSID mismatch: got '%s'\n", device.device_params.ssid);
        ret_val++;
    }

    if (strcmp(device.device_params.ssid_ap, expected_ap_ssid) != 0)
    {
        printf("❌ AP SSID mismatch: got '%s'\n", device.device_params.ssid_ap);
        ret_val++;
    }

    if (strcmp(device.device_params.pw_ap, expected_ap_password) != 0)
    {
        printf("❌ AP password mismatch: got '%s'\n", device.device_params.pw_ap);
        ret_val++;
    }

    if (device.device_params.all_enabled_interfaces != expected_comm_mode)
    {
        printf("❌ Communication mode mismatch: got %d\n", device.device_params.all_enabled_interfaces);
        ret_val++;
    }

    if (strcmp(device.device_descriptor.device_sn, expected_serial) != 0)
    {
        printf("❌ Serial number mismatch: got '%s'\n", device.device_descriptor.device_sn);
        ret_val++;
    }

    if (strcmp(device.device_descriptor.device_model, expected_model) != 0)
    {
        printf("❌ Device model mismatch: got '%s'\n", device.device_descriptor.device_model);
        ret_val++;
    }

    if (strcmp(device.device_descriptor.device_fw, expected_fw) != 0)
    {
        printf("❌ Firmware version mismatch: got '%s'\n", device.device_descriptor.device_fw);
        ret_val++;
    }

    if (strcmp(device.device_descriptor.device_desc, expected_desc) != 0)
    {
        printf("❌ Device description mismatch: got '%s'\n", device.device_descriptor.device_desc);
        ret_val++;
    }

    if (ret_val == 0)
        printf("\n✅ All simulated fields populated correctly.\n");
    else
        printf("\n❌ %d field(s) failed simulation validation.\n", ret_val);

    return ret_val;
}