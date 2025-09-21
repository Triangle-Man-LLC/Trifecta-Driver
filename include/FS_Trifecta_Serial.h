/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_SERIAL_H
#define TRIFECTA_SERIAL_H

#include "FS_Trifecta_Defs.h"
#include "FS_Trifecta_Interfaces.h"

#include "FS_Trifecta_Device.h"

#ifdef __cplusplus
extern "C"
{
#endif

    int fs_serial_send_message(fs_device_info_t *device_handle, char* message, size_t len);
    int fs_serial_start(fs_device_info_t *device_handle);
    int fs_serial_exit(fs_device_info_t *device_handle);
    int fs_serial_device_restart(fs_device_info_t *device_handle);

#ifdef __cplusplus
}
#endif

#endif
