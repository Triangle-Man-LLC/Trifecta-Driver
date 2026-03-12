#include <jni.h>
#include "FS_Trifecta_Defs.h"

/* ---------- Regular packet field cache ---------- */

static jboolean g_regular_inited = JNI_FALSE;
static jfieldID g_reg_type;
static jfieldID g_reg_time;
static jfieldID g_reg_omega_x0;
static jfieldID g_reg_omega_y0;
static jfieldID g_reg_omega_z0;
static jfieldID g_reg_q0;
static jfieldID g_reg_q1;
static jfieldID g_reg_q2;
static jfieldID g_reg_q3;
static jfieldID g_reg_mag_x;
static jfieldID g_reg_mag_y;
static jfieldID g_reg_mag_z;
static jfieldID g_reg_acc_x;
static jfieldID g_reg_acc_y;
static jfieldID g_reg_acc_z;
static jfieldID g_reg_reserved_0_1;
static jfieldID g_reg_reserved_0_2;
static jfieldID g_reg_reserved_0_3;
static jfieldID g_reg_temperature;      // short[]
static jfieldID g_reg_device_motion_status;
static jfieldID g_reg_diagnostic_flag;
static jfieldID g_reg_reserved;         // byte[]
static jfieldID g_reg_c;
static jfieldID g_reg_barometric_pressure;

static void init_regular_fields(JNIEnv *env, jobject outObj)
{
    if (g_regular_inited) return;

    jclass cls = (*env)->GetObjectClass(env, outObj);

    g_reg_type      = (*env)->GetFieldID(env, cls, "type", "B");
    g_reg_time      = (*env)->GetFieldID(env, cls, "time", "I");
    g_reg_omega_x0  = (*env)->GetFieldID(env, cls, "omega_x0", "F");
    g_reg_omega_y0  = (*env)->GetFieldID(env, cls, "omega_y0", "F");
    g_reg_omega_z0  = (*env)->GetFieldID(env, cls, "omega_z0", "F");
    g_reg_q0        = (*env)->GetFieldID(env, cls, "q0", "F");
    g_reg_q1        = (*env)->GetFieldID(env, cls, "q1", "F");
    g_reg_q2        = (*env)->GetFieldID(env, cls, "q2", "F");
    g_reg_q3        = (*env)->GetFieldID(env, cls, "q3", "F");
    g_reg_mag_x     = (*env)->GetFieldID(env, cls, "mag_x", "F");
    g_reg_mag_y     = (*env)->GetFieldID(env, cls, "mag_y", "F");
    g_reg_mag_z     = (*env)->GetFieldID(env, cls, "mag_z", "F");
    g_reg_acc_x     = (*env)->GetFieldID(env, cls, "acc_x", "F");
    g_reg_acc_y     = (*env)->GetFieldID(env, cls, "acc_y", "F");
    g_reg_acc_z     = (*env)->GetFieldID(env, cls, "acc_z", "F");
    g_reg_reserved_0_1 = (*env)->GetFieldID(env, cls, "reserved_0_1", "F");
    g_reg_reserved_0_2 = (*env)->GetFieldID(env, cls, "reserved_0_2", "F");
    g_reg_reserved_0_3 = (*env)->GetFieldID(env, cls, "reserved_0_3", "F");

    g_reg_temperature = (*env)->GetFieldID(env, cls, "temperature", "[S");
    g_reg_device_motion_status = (*env)->GetFieldID(env, cls, "device_motion_status", "B");
    g_reg_diagnostic_flag      = (*env)->GetFieldID(env, cls, "diagnostic_flag", "B");
    g_reg_reserved             = (*env)->GetFieldID(env, cls, "reserved", "[B");
    g_reg_c                    = (*env)->GetFieldID(env, cls, "c", "B");
    g_reg_barometric_pressure  = (*env)->GetFieldID(env, cls, "barometric_pressure", "F");

    g_regular_inited = JNI_TRUE;
}

static void fill_regular_packet(JNIEnv *env, jobject outObj, const fs_imu_regular_packet_t *src)
{
    init_regular_fields(env, outObj);

    (*env)->SetByteField(env, outObj, g_reg_type, (jbyte)src->type);
    (*env)->SetIntField(env, outObj, g_reg_time, (jint)src->time);

    (*env)->SetFloatField(env, outObj, g_reg_omega_x0, src->omega_x0);
    (*env)->SetFloatField(env, outObj, g_reg_omega_y0, src->omega_y0);
    (*env)->SetFloatField(env, outObj, g_reg_omega_z0, src->omega_z0);

    (*env)->SetFloatField(env, outObj, g_reg_q0, src->q0);
    (*env)->SetFloatField(env, outObj, g_reg_q1, src->q1);
    (*env)->SetFloatField(env, outObj, g_reg_q2, src->q2);
    (*env)->SetFloatField(env, outObj, g_reg_q3, src->q3);

    (*env)->SetFloatField(env, outObj, g_reg_mag_x, src->mag_x);
    (*env)->SetFloatField(env, outObj, g_reg_mag_y, src->mag_y);
    (*env)->SetFloatField(env, outObj, g_reg_mag_z, src->mag_z);

    (*env)->SetFloatField(env, outObj, g_reg_acc_x, src->acc_x);
    (*env)->SetFloatField(env, outObj, g_reg_acc_y, src->acc_y);
    (*env)->SetFloatField(env, outObj, g_reg_acc_z, src->acc_z);

    (*env)->SetFloatField(env, outObj, g_reg_reserved_0_1, src->reserved_0_1);
    (*env)->SetFloatField(env, outObj, g_reg_reserved_0_2, src->reserved_0_2);
    (*env)->SetFloatField(env, outObj, g_reg_reserved_0_3, src->reserved_0_3);

    // temperature[3] (short[])
    jshortArray tempArr = (jshortArray)(*env)->GetObjectField(env, outObj, g_reg_temperature);
    jshort tvals[3] = { src->temperature[0], src->temperature[1], src->temperature[2] };
    (*env)->SetShortArrayRegion(env, tempArr, 0, 3, tvals);

    (*env)->SetByteField(env, outObj, g_reg_device_motion_status, (jbyte)src->device_motion_status);
    (*env)->SetByteField(env, outObj, g_reg_diagnostic_flag, (jbyte)src->diagnostic_flag);

    // reserved[3] (byte[])
    jbyteArray resArr = (jbyteArray)(*env)->GetObjectField(env, outObj, g_reg_reserved);
    jbyte rvals[3] = { src->reserved[0], src->reserved[1], src->reserved[2] };
    (*env)->SetByteArrayRegion(env, resArr, 0, 3, rvals);

    (*env)->SetByteField(env, outObj, g_reg_c, (jbyte)src->c);
    (*env)->SetFloatField(env, outObj, g_reg_barometric_pressure, src->barometric_pressure);
}

/* ---------- Composite packet field cache ---------- */

static jboolean g_comp_inited = JNI_FALSE;
static jfieldID g_comp_type;
static jfieldID g_comp_time;
static jfieldID g_comp_ax0, g_comp_ay0, g_comp_az0, g_comp_gx0, g_comp_gy0, g_comp_gz0;
static jfieldID g_comp_ax1, g_comp_ay1, g_comp_az1, g_comp_gx1, g_comp_gy1, g_comp_gz1;
static jfieldID g_comp_ax2, g_comp_ay2, g_comp_az2, g_comp_gx2, g_comp_gy2, g_comp_gz2;
static jfieldID g_comp_q0, g_comp_q1, g_comp_q2, g_comp_q3;
static jfieldID g_comp_mag_x, g_comp_mag_y, g_comp_mag_z;
static jfieldID g_comp_acc_x, g_comp_acc_y, g_comp_acc_z;
static jfieldID g_comp_omega_x0, g_comp_omega_y0, g_comp_omega_z0;
static jfieldID g_comp_temperature;      // short[]
static jfieldID g_comp_device_motion_status;
static jfieldID g_comp_diagnostic_flag;
static jfieldID g_comp_reserved;         // byte[]
static jfieldID g_comp_c;
static jfieldID g_comp_barometric_pressure;

static void init_composite_fields(JNIEnv *env, jobject outObj)
{
    if (g_comp_inited) return;

    jclass cls = (*env)->GetObjectClass(env, outObj);

    g_comp_type = (*env)->GetFieldID(env, cls, "type", "B");
    g_comp_time = (*env)->GetFieldID(env, cls, "time", "I");

    g_comp_ax0 = (*env)->GetFieldID(env, cls, "ax0", "F");
    g_comp_ay0 = (*env)->GetFieldID(env, cls, "ay0", "F");
    g_comp_az0 = (*env)->GetFieldID(env, cls, "az0", "F");
    g_comp_gx0 = (*env)->GetFieldID(env, cls, "gx0", "F");
    g_comp_gy0 = (*env)->GetFieldID(env, cls, "gy0", "F");
    g_comp_gz0 = (*env)->GetFieldID(env, cls, "gz0", "F");

    g_comp_ax1 = (*env)->GetFieldID(env, cls, "ax1", "F");
    g_comp_ay1 = (*env)->GetFieldID(env, cls, "ay1", "F");
    g_comp_az1 = (*env)->GetFieldID(env, cls, "az1", "F");
    g_comp_gx1 = (*env)->GetFieldID(env, cls, "gx1", "F");
    g_comp_gy1 = (*env)->GetFieldID(env, cls, "gy1", "F");
    g_comp_gz1 = (*env)->GetFieldID(env, cls, "gz1", "F");

    g_comp_ax2 = (*env)->GetFieldID(env, cls, "ax2", "F");
    g_comp_ay2 = (*env)->GetFieldID(env, cls, "ay2", "F");
    g_comp_az2 = (*env)->GetFieldID(env, cls, "az2", "F");
    g_comp_gx2 = (*env)->GetFieldID(env, cls, "gx2", "F");
    g_comp_gy2 = (*env)->GetFieldID(env, cls, "gy2", "F");
    g_comp_gz2 = (*env)->GetFieldID(env, cls, "gz2", "F");

    g_comp_q0 = (*env)->GetFieldID(env, cls, "q0", "F");
    g_comp_q1 = (*env)->GetFieldID(env, cls, "q1", "F");
    g_comp_q2 = (*env)->GetFieldID(env, cls, "q2", "F");
    g_comp_q3 = (*env)->GetFieldID(env, cls, "q3", "F");

    g_comp_mag_x = (*env)->GetFieldID(env, cls, "mag_x", "F");
    g_comp_mag_y = (*env)->GetFieldID(env, cls, "mag_y", "F");
    g_comp_mag_z = (*env)->GetFieldID(env, cls, "mag_z", "F");

    g_comp_acc_x = (*env)->GetFieldID(env, cls, "acc_x", "F");
    g_comp_acc_y = (*env)->GetFieldID(env, cls, "acc_y", "F");
    g_comp_acc_z = (*env)->GetFieldID(env, cls, "acc_z", "F");

    g_comp_omega_x0 = (*env)->GetFieldID(env, cls, "omega_x0", "F");
    g_comp_omega_y0 = (*env)->GetFieldID(env, cls, "omega_y0", "F");
    g_comp_omega_z0 = (*env)->GetFieldID(env, cls, "omega_z0", "F");

    g_comp_temperature = (*env)->GetFieldID(env, cls, "temperature", "[S");
    g_comp_device_motion_status = (*env)->GetFieldID(env, cls, "device_motion_status", "B");
    g_comp_diagnostic_flag      = (*env)->GetFieldID(env, cls, "diagnostic_flag", "B");
    g_comp_reserved             = (*env)->GetFieldID(env, cls, "reserved", "[B");
    g_comp_c                    = (*env)->GetFieldID(env, cls, "c", "B");
    g_comp_barometric_pressure  = (*env)->GetFieldID(env, cls, "barometric_pressure", "F");

    g_comp_inited = JNI_TRUE;
}

static void fill_composite_packet(JNIEnv *env, jobject outObj, const fs_imu_composite_packet_t *src)
{
    init_composite_fields(env, outObj);

    (*env)->SetByteField(env, outObj, g_comp_type, (jbyte)src->type);
    (*env)->SetIntField(env, outObj, g_comp_time, (jint)src->time);

    #define SF(fid, val) (*env)->SetFloatField(env, outObj, fid, (jfloat)(val))

    SF(g_comp_ax0, src->ax0); SF(g_comp_ay0, src->ay0); SF(g_comp_az0, src->az0);
    SF(g_comp_gx0, src->gx0); SF(g_comp_gy0, src->gy0); SF(g_comp_gz0, src->gz0);

    SF(g_comp_ax1, src->ax1); SF(g_comp_ay1, src->ay1); SF(g_comp_az1, src->az1);
    SF(g_comp_gx1, src->gx1); SF(g_comp_gy1, src->gy1); SF(g_comp_gz1, src->gz1);

    SF(g_comp_ax2, src->ax2); SF(g_comp_ay2, src->ay2); SF(g_comp_az2, src->az2);
    SF(g_comp_gx2, src->gx2); SF(g_comp_gy2, src->gy2); SF(g_comp_gz2, src->gz2);

    SF(g_comp_q0, src->q0); SF(g_comp_q1, src->q1); SF(g_comp_q2, src->q2); SF(g_comp_q3, src->q3);

    SF(g_comp_mag_x, src->mag_x); SF(g_comp_mag_y, src->mag_y); SF(g_comp_mag_z, src->mag_z);
    SF(g_comp_acc_x, src->acc_x); SF(g_comp_acc_y, src->acc_y); SF(g_comp_acc_z, src->acc_z);
    SF(g_comp_omega_x0, src->omega_x0); SF(g_comp_omega_y0, src->omega_y0); SF(g_comp_omega_z0, src->omega_z0);

    #undef SF

    // temperature
    jshortArray tempArr = (jshortArray)(*env)->GetObjectField(env, outObj, g_comp_temperature);
    jshort tvals[3] = { src->temperature[0], src->temperature[1], src->temperature[2] };
    (*env)->SetShortArrayRegion(env, tempArr, 0, 3, tvals);

    (*env)->SetByteField(env, outObj, g_comp_device_motion_status, (jbyte)src->device_motion_status);
    (*env)->SetByteField(env, outObj, g_comp_diagnostic_flag, (jbyte)src->diagnostic_flag);

    // reserved[3]
    jbyteArray resArr = (jbyteArray)(*env)->GetObjectField(env, outObj, g_comp_reserved);
    jbyte rvals[3] = { src->reserved[0], src->reserved[1], src->reserved[2] };
    (*env)->SetByteArrayRegion(env, resArr, 0, 3, rvals);

    (*env)->SetByteField(env, outObj, g_comp_c, (jbyte)src->c);
    (*env)->SetFloatField(env, outObj, g_comp_barometric_pressure, src->barometric_pressure);
}

/* ---------- Composite2 packet field cache ---------- */

static jboolean g_comp2_inited = JNI_FALSE;
static jfieldID g_c2_type;
static jfieldID g_c2_time;
static jfieldID g_c2_ax0, g_c2_ay0, g_c2_az0, g_c2_gx0, g_c2_gy0, g_c2_gz0;
static jfieldID g_c2_ax1, g_c2_ay1, g_c2_az1, g_c2_gx1, g_c2_gy1, g_c2_gz1;
static jfieldID g_c2_ax2, g_c2_ay2, g_c2_az2, g_c2_gx2, g_c2_gy2, g_c2_gz2;
static jfieldID g_c2_q0, g_c2_q1, g_c2_q2, g_c2_q3;
static jfieldID g_c2_mag_x, g_c2_mag_y, g_c2_mag_z;
static jfieldID g_c2_omega_x0, g_c2_omega_y0, g_c2_omega_z0;
static jfieldID g_c2_acc_x, g_c2_acc_y, g_c2_acc_z;
static jfieldID g_c2_vx, g_c2_vy, g_c2_vz;
static jfieldID g_c2_rx, g_c2_ry, g_c2_rz;   // double
static jfieldID g_c2_temperature;           // short[]
static jfieldID g_c2_device_motion_status;
static jfieldID g_c2_diagnostic_flag;
static jfieldID g_c2_reserved;              // byte[]
static jfieldID g_c2_c;
static jfieldID g_c2_barometric_pressure;

static void init_composite2_fields(JNIEnv *env, jobject outObj)
{
    if (g_comp2_inited) return;

    jclass cls = (*env)->GetObjectClass(env, outObj);

    g_c2_type = (*env)->GetFieldID(env, cls, "type", "B");
    g_c2_time = (*env)->GetFieldID(env, cls, "time", "I");

    g_c2_ax0 = (*env)->GetFieldID(env, cls, "ax0", "F");
    g_c2_ay0 = (*env)->GetFieldID(env, cls, "ay0", "F");
    g_c2_az0 = (*env)->GetFieldID(env, cls, "az0", "F");
    g_c2_gx0 = (*env)->GetFieldID(env, cls, "gx0", "F");
    g_c2_gy0 = (*env)->GetFieldID(env, cls, "gy0", "F");
    g_c2_gz0 = (*env)->GetFieldID(env, cls, "gz0", "F");

    g_c2_ax1 = (*env)->GetFieldID(env, cls, "ax1", "F");
    g_c2_ay1 = (*env)->GetFieldID(env, cls, "ay1", "F");
    g_c2_az1 = (*env)->GetFieldID(env, cls, "az1", "F");
    g_c2_gx1 = (*env)->GetFieldID(env, cls, "gx1", "F");
    g_c2_gy1 = (*env)->GetFieldID(env, cls, "gy1", "F");
    g_c2_gz1 = (*env)->GetFieldID(env, cls, "gz1", "F");

    g_c2_ax2 = (*env)->GetFieldID(env, cls, "ax2", "F");
    g_c2_ay2 = (*env)->GetFieldID(env, cls, "ay2", "F");
    g_c2_az2 = (*env)->GetFieldID(env, cls, "az2", "F");
    g_c2_gx2 = (*env)->GetFieldID(env, cls, "gx2", "F");
    g_c2_gy2 = (*env)->GetFieldID(env, cls, "gy2", "F");
    g_c2_gz2 = (*env)->GetFieldID(env, cls, "gz2", "F");

    g_c2_q0 = (*env)->GetFieldID(env, cls, "q0", "F");
    g_c2_q1 = (*env)->GetFieldID(env, cls, "q1", "F");
    g_c2_q2 = (*env)->GetFieldID(env, cls, "q2", "F");
    g_c2_q3 = (*env)->GetFieldID(env, cls, "q3", "F");

    g_c2_mag_x = (*env)->GetFieldID(env, cls, "mag_x", "F");
    g_c2_mag_y = (*env)->GetFieldID(env, cls, "mag_y", "F");
    g_c2_mag_z = (*env)->GetFieldID(env, cls, "mag_z", "F");

    g_c2_omega_x0 = (*env)->GetFieldID(env, cls, "omega_x0", "F");
    g_c2_omega_y0 = (*env)->GetFieldID(env, cls, "omega_y0", "F");
    g_c2_omega_z0 = (*env)->GetFieldID(env, cls, "omega_z0", "F");

    g_c2_acc_x = (*env)->GetFieldID(env, cls, "acc_x", "F");
    g_c2_acc_y = (*env)->GetFieldID(env, cls, "acc_y", "F");
    g_c2_acc_z = (*env)->GetFieldID(env, cls, "acc_z", "F");

    g_c2_vx = (*env)->GetFieldID(env, cls, "vx", "F");
    g_c2_vy = (*env)->GetFieldID(env, cls, "vy", "F");
    g_c2_vz = (*env)->GetFieldID(env, cls, "vz", "F");

    g_c2_rx = (*env)->GetFieldID(env, cls, "rx", "D");
    g_c2_ry = (*env)->GetFieldID(env, cls, "ry", "D");
    g_c2_rz = (*env)->GetFieldID(env, cls, "rz", "D");

    g_c2_temperature = (*env)->GetFieldID(env, cls, "temperature", "[S");
    g_c2_device_motion_status = (*env)->GetFieldID(env, cls, "device_motion_status", "B");
    g_c2_diagnostic_flag      = (*env)->GetFieldID(env, cls, "diagnostic_flag", "B");
    g_c2_reserved             = (*env)->GetFieldID(env, cls, "reserved", "[B");
    g_c2_c                    = (*env)->GetFieldID(env, cls, "c", "B");
    g_c2_barometric_pressure  = (*env)->GetFieldID(env, cls, "barometric_pressure", "F");

    g_comp2_inited = JNI_TRUE;
}

static void fill_composite2_packet(JNIEnv *env, jobject outObj, const fs_imu_composite_packet_2_t *src)
{
    init_composite2_fields(env, outObj);

    (*env)->SetByteField(env, outObj, g_c2_type, (jbyte)src->type);
    (*env)->SetIntField(env, outObj, g_c2_time, (jint)src->time);

    #define SF(fid, val) (*env)->SetFloatField(env, outObj, fid, (jfloat)(val))

    SF(g_c2_ax0, src->ax0); SF(g_c2_ay0, src->ay0); SF(g_c2_az0, src->az0);
    SF(g_c2_gx0, src->gx0); SF(g_c2_gy0, src->gy0); SF(g_c2_gz0, src->gz0);

    SF(g_c2_ax1, src->ax1); SF(g_c2_ay1, src->ay1); SF(g_c2_az1, src->az1);
    SF(g_c2_gx1, src->gx1); SF(g_c2_gy1, src->gy1); SF(g_c2_gz1, src->gz1);

    SF(g_c2_ax2, src->ax2); SF(g_c2_ay2, src->ay2); SF(g_c2_az2, src->az2);
    SF(g_c2_gx2, src->gx2); SF(g_c2_gy2, src->gy2); SF(g_c2_gz2, src->gz2);

    SF(g_c2_q0, src->q0); SF(g_c2_q1, src->q1); SF(g_c2_q2, src->q2); SF(g_c2_q3, src->q3);

    SF(g_c2_mag_x, src->mag_x); SF(g_c2_mag_y, src->mag_y); SF(g_c2_mag_z, src->mag_z);
    SF(g_c2_omega_x0, src->omega_x0); SF(g_c2_omega_y0, src->omega_y0); SF(g_c2_omega_z0, src->omega_z0);
    SF(g_c2_acc_x, src->acc_x); SF(g_c2_acc_y, src->acc_y); SF(g_c2_acc_z, src->acc_z);

    SF(g_c2_vx, src->vx); SF(g_c2_vy, src->vy); SF(g_c2_vz, src->vz);

    #undef SF

    (*env)->SetDoubleField(env, outObj, g_c2_rx, src->rx);
    (*env)->SetDoubleField(env, outObj, g_c2_ry, src->ry);
    (*env)->SetDoubleField(env, outObj, g_c2_rz, src->rz);

    // temperature
    jshortArray tempArr = (jshortArray)(*env)->GetObjectField(env, outObj, g_c2_temperature);
    jshort tvals[3] = { src->temperature[0], src->temperature[1], src->temperature[2] };
    (*env)->SetShortArrayRegion(env, tempArr, 0, 3, tvals);

    (*env)->SetByteField(env, outObj, g_c2_device_motion_status, (jbyte)src->device_motion_status);
    (*env)->SetByteField(env, outObj, g_c2_diagnostic_flag, (jbyte)src->diagnostic_flag);

    // reserved[3]
    jbyteArray resArr = (jbyteArray)(*env)->GetObjectField(env, outObj, g_c2_reserved);
    jbyte rvals[3] = { src->reserved[0], src->reserved[1], src->reserved[2] };
    (*env)->SetByteArrayRegion(env, resArr, 0, 3, rvals);

    (*env)->SetByteField(env, outObj, g_c2_c, (jbyte)src->c);
    (*env)->SetFloatField(env, outObj, g_c2_barometric_pressure, src->barometric_pressure);
}

/* ---------- JNI entry points ---------- */

JNIEXPORT jint JNICALL
Java_com_trifecta_fs_1unpack_1regular_1packet(JNIEnv *env, jclass clazz, jlong devPtr, jobject outObj)
{
    fs_device_info_t *dev = (fs_device_info_t *)devPtr;
    if (!dev || !outObj) return -1;
    fill_regular_packet(env, outObj, &dev->last_received_packet.regular);
    return 0;
}

JNIEXPORT jint JNICALL
Java_com_trifecta_fs_1unpack_1composite_1packet(JNIEnv *env, jclass clazz, jlong devPtr, jobject outObj)
{
    fs_device_info_t *dev = (fs_device_info_t *)devPtr;
    if (!dev || !outObj) return -1;
    fill_composite_packet(env, outObj, &dev->last_received_packet.composite);
    return 0;
}

JNIEXPORT jint JNICALL
Java_com_trifecta_fs_1unpack_1composite2_1packet(JNIEnv *env, jclass clazz, jlong devPtr, jobject outObj)
{
    fs_device_info_t *dev = (fs_device_info_t *)devPtr;
    if (!dev || !outObj) return -1;
    fill_composite2_packet(env, outObj, &dev->last_received_packet.composite2);
    return 0;
}
