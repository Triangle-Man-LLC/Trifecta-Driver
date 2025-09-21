/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_DEFS_RINGBUFFER_H
#define TRIFECTA_DEFS_RINGBUFFER_H

/// @brief Ringbuffer generator.
#define FS_RINGBUFFER_DECLARE(type, name, size) \
    typedef struct                              \
    {                                           \
        type buffer[size];                      \
        volatile uint16_t head;                 \
        volatile uint16_t tail;                 \
        volatile uint16_t count;                \
    } name
  
/// @section Packet ring buffer functions (implemented using MACROS to allow use with all fs_ringbuffers)
#define FS_RINGBUFFER_INIT(rb) \
    do                         \
    {                          \
        (rb)->head = 0;        \
        (rb)->tail = 0;        \
        (rb)->count = 0;       \
    } while (0)

/// @brief Ringbuffer push
/// @param rb Ringbuffer handle
/// @param buffer_size Ringbuffer size
/// @param value_ptr Pointer to the thing to enqueue
/// @return TRUE on success, FALSE if failed (out of space)
#define FS_RINGBUFFER_PUSH(rb, buffer_size, value_ptr) \
    (((rb)->count >= (buffer_size)) ? false : ((rb)->buffer[(rb)->head] = *(value_ptr), (rb)->head = ((rb)->head + 1) % (buffer_size), (rb)->count++, true))

/// @brief Ringbuffer push, but overwrite oldest element if full
/// @param rb Ringbuffer handle
/// @param buffer_size Ringbuffer size
/// @param value_ptr Pointer to the thing to enqueue
/// @return TRUE on success (always)
#define FS_RINGBUFFER_PUSH_FORCE(rb, buffer_size, value_ptr)                                            \
    ((rb)->buffer[(rb)->head] = *(value_ptr),                                                           \
     (rb)->head = ((rb)->head + 1) % (buffer_size),                                                     \
     ((rb)->count < (buffer_size)) ? ((rb)->count++) : ((rb)->tail = ((rb)->tail + 1) % (buffer_size)), \
     true)

/// @brief Ringbuffer pop
/// @param rb Ringbuffer handle
/// @param buffer_size Ringbuffer size
/// @param value_ptr Pointer to the thing to dequeue
/// @return TRUE on success, FALSE if failed (no items)
#define FS_RINGBUFFER_POP(rb, buffer_size, out_ptr) \
    (((rb)->count == 0) ? false : (*(out_ptr) = (rb)->buffer[(rb)->tail], (rb)->tail = ((rb)->tail + 1) % (buffer_size), (rb)->count--, true))

/// @brief Ringbuffer peek
/// @param rb Ringbuffer handle
/// @param out_ptr Pointer to the thing to enqueue
/// @return TRUE on success, FALSE if failed (no items)
#define FS_RINGBUFFER_PEEK(rb, out_ptr) \
    (((rb)->count == 0) ? false : (*(out_ptr) = (rb)->buffer[(rb)->tail], true))

/// @brief Ringbuffer peek, but at an indicated index
/// @param rb Ringbuffer handle
/// @param out_ptr Pointer to the thing to enqueue
/// @return TRUE on success, FALSE if failed (no items)
#define FS_RINGBUFFER_PEEK_AT(rb, buffer_size, index, out_ptr) \
    (((index) >= (rb)->count) ? false : (*(out_ptr) = (rb)->buffer[((rb)->tail + (index)) % (buffer_size)], true))

#endif