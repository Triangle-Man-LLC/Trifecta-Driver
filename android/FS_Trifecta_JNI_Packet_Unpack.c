#include <jni.h>

#include "FS_Trifecta_Defs.h"

#include <jni.h>
#include "FS_Trifecta_Defs.h"

static void fill_regular_packet(JNIEnv *env, jobject outObj, const fs_imu_regular_packet_t *src)
{
    jclass cls = (*env)->GetObjectClass(env, outObj);

    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "type", "B"),
                         (jbyte)src->type);

    (*env)->SetIntField(env, outObj,
                        (*env)->GetFieldID(env, cls, "time", "I"),
                        (jint)src->time);

    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "omega_x0", "F"),
                          src->omega_x0);
    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "omega_y0", "F"),
                          src->omega_y0);
    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "omega_z0", "F"),
                          src->omega_z0);

    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "q0", "F"),
                          src->q0);
    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "q1", "F"),
                          src->q1);
    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "q2", "F"),
                          src->q2);
    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "q3", "F"),
                          src->q3);

    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "mag_x", "F"),
                          src->mag_x);
    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "mag_y", "F"),
                          src->mag_y);
    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "mag_z", "F"),
                          src->mag_z);

    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "acc_x", "F"),
                          src->acc_x);
    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "acc_y", "F"),
                          src->acc_y);
    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "acc_z", "F"),
                          src->acc_z);

    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "reserved_0_1", "F"),
                          src->reserved_0_1);
    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "reserved_0_2", "F"),
                          src->reserved_0_2);
    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "reserved_0_3", "F"),
                          src->reserved_0_3);

    (*env)->SetShortField(env, outObj,
                          (*env)->GetFieldID(env, cls, "reserved_x", "S"),
                          (jshort)src->reserved_x);
    (*env)->SetShortField(env, outObj,
                          (*env)->GetFieldID(env, cls, "reserved_y", "S"),
                          (jshort)src->reserved_y);
    (*env)->SetShortField(env, outObj,
                          (*env)->GetFieldID(env, cls, "reserved_z", "S"),
                          (jshort)src->reserved_z);

    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "device_motion_status", "B"),
                         (jbyte)src->device_motion_status);
    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "label_2", "B"),
                         (jbyte)src->label_2);

    // temperature[3]
    jfieldID fidTemp = (*env)->GetFieldID(env, cls, "temperature", "[B");
    jobject tempArrObj = (*env)->GetObjectField(env, outObj, fidTemp);
    jbyteArray tempArr = (jbyteArray)tempArrObj;
    jbyte tempVals[3];
    tempVals[0] = (jbyte)src->temperature[0];
    tempVals[1] = (jbyte)src->temperature[1];
    tempVals[2] = (jbyte)src->temperature[2];
    (*env)->SetByteArrayRegion(env, tempArr, 0, 3, tempVals);

    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "c", "B"),
                         (jbyte)src->c);

    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "barometric_pressure", "F"),
                          src->barometric_pressure);
}

static void fill_composite_packet(JNIEnv *env, jobject outObj, const fs_imu_composite_packet_t *src)
{
    jclass cls = (*env)->GetObjectClass(env, outObj);

    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "type", "B"),
                         (jbyte)src->type);
    (*env)->SetIntField(env, outObj,
                        (*env)->GetFieldID(env, cls, "time", "I"),
                        (jint)src->time);

    // ax0..gz2
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ax0", "F"), src->ax0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ay0", "F"), src->ay0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "az0", "F"), src->az0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gx0", "F"), src->gx0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gy0", "F"), src->gy0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gz0", "F"), src->gz0);

    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ax1", "F"), src->ax1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ay1", "F"), src->ay1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "az1", "F"), src->az1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gx1", "F"), src->gx1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gy1", "F"), src->gy1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gz1", "F"), src->gz1);

    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ax2", "F"), src->ax2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ay2", "F"), src->ay2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "az2", "F"), src->az2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gx2", "F"), src->gx2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gy2", "F"), src->gy2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gz2", "F"), src->gz2);

    // q0..q3
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "q0", "F"), src->q0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "q1", "F"), src->q1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "q2", "F"), src->q2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "q3", "F"), src->q3);

    // mag, acc, omega
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "mag_x", "F"), src->mag_x);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "mag_y", "F"), src->mag_y);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "mag_z", "F"), src->mag_z);

    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "acc_x", "F"), src->acc_x);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "acc_y", "F"), src->acc_y);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "acc_z", "F"), src->acc_z);

    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "omega_x0", "F"), src->omega_x0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "omega_y0", "F"), src->omega_y0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "omega_z0", "F"), src->omega_z0);

    // reserved shorts
    (*env)->SetShortField(env, outObj, (*env)->GetFieldID(env, cls, "reserved_x", "S"), (jshort)src->reserved_x);
    (*env)->SetShortField(env, outObj, (*env)->GetFieldID(env, cls, "reserved_y", "S"), (jshort)src->reserved_y);
    (*env)->SetShortField(env, outObj, (*env)->GetFieldID(env, cls, "reserved_z", "S"), (jshort)src->reserved_z);

    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "device_motion_status", "B"),
                         (jbyte)src->device_motion_status);
    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "label_2", "B"),
                         (jbyte)src->label_2);

    jfieldID fidTemp = (*env)->GetFieldID(env, cls, "temperature", "[B");
    jobject tempArrObj = (*env)->GetObjectField(env, outObj, fidTemp);
    jbyteArray tempArr = (jbyteArray)tempArrObj;
    jbyte tempVals[3];
    tempVals[0] = (jbyte)src->temperature[0];
    tempVals[1] = (jbyte)src->temperature[1];
    tempVals[2] = (jbyte)src->temperature[2];
    (*env)->SetByteArrayRegion(env, tempArr, 0, 3, tempVals);

    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "c", "B"),
                         (jbyte)src->c);

    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "barometric_pressure", "F"),
                          src->barometric_pressure);
}

static void fill_composite2_packet(JNIEnv *env, jobject outObj, const fs_imu_composite_packet_2_t *src)
{
    jclass cls = (*env)->GetObjectClass(env, outObj);

    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "type", "B"),
                         (jbyte)src->type);
    (*env)->SetIntField(env, outObj,
                        (*env)->GetFieldID(env, cls, "time", "I"),
                        (jint)src->time);

    // ax0..gz2
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ax0", "F"), src->ax0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ay0", "F"), src->ay0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "az0", "F"), src->az0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gx0", "F"), src->gx0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gy0", "F"), src->gy0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gz0", "F"), src->gz0);

    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ax1", "F"), src->ax1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ay1", "F"), src->ay1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "az1", "F"), src->az1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gx1", "F"), src->gx1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gy1", "F"), src->gy1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gz1", "F"), src->gz1);

    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ax2", "F"), src->ax2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "ay2", "F"), src->ay2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "az2", "F"), src->az2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gx2", "F"), src->gx2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gy2", "F"), src->gy2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "gz2", "F"), src->gz2);

    // q0..q3
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "q0", "F"), src->q0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "q1", "F"), src->q1);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "q2", "F"), src->q2);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "q3", "F"), src->q3);

    // mag, omega, acc
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "mag_x", "F"), src->mag_x);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "mag_y", "F"), src->mag_y);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "mag_z", "F"), src->mag_z);

    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "omega_x0", "F"), src->omega_x0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "omega_y0", "F"), src->omega_y0);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "omega_z0", "F"), src->omega_z0);

    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "acc_x", "F"), src->acc_x);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "acc_y", "F"), src->acc_y);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "acc_z", "F"), src->acc_z);

    // velocity
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "vx", "F"), src->vx);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "vy", "F"), src->vy);
    (*env)->SetFloatField(env, outObj, (*env)->GetFieldID(env, cls, "vz", "F"), src->vz);

    // position
    (*env)->SetDoubleField(env, outObj, (*env)->GetFieldID(env, cls, "rx", "D"), src->rx);
    (*env)->SetDoubleField(env, outObj, (*env)->GetFieldID(env, cls, "ry", "D"), src->ry);
    (*env)->SetDoubleField(env, outObj, (*env)->GetFieldID(env, cls, "rz", "D"), src->rz);

    // reserved shorts
    (*env)->SetShortField(env, outObj, (*env)->GetFieldID(env, cls, "reserved_x", "S"), (jshort)src->reserved_x);
    (*env)->SetShortField(env, outObj, (*env)->GetFieldID(env, cls, "reserved_y", "S"), (jshort)src->reserved_y);
    (*env)->SetShortField(env, outObj, (*env)->GetFieldID(env, cls, "reserved_z", "S"), (jshort)src->reserved_z);

    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "device_motion_status", "B"),
                         (jbyte)src->device_motion_status);
    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "label_2", "B"),
                         (jbyte)src->label_2);

    jfieldID fidTemp = (*env)->GetFieldID(env, cls, "temperature", "[B");
    jobject tempArrObj = (*env)->GetObjectField(env, outObj, fidTemp);
    jbyteArray tempArr = (jbyteArray)tempArrObj;
    jbyte tempVals[3];
    tempVals[0] = (jbyte)src->temperature[0];
    tempVals[1] = (jbyte)src->temperature[1];
    tempVals[2] = (jbyte)src->temperature[2];
    (*env)->SetByteArrayRegion(env, tempArr, 0, 3, tempVals);

    (*env)->SetByteField(env, outObj,
                         (*env)->GetFieldID(env, cls, "c", "B"),
                         (jbyte)src->c);

    (*env)->SetFloatField(env, outObj,
                          (*env)->GetFieldID(env, cls, "barometric_pressure", "F"),
                          src->barometric_pressure);
}

JNIEXPORT jint JNICALL
Java_com_trifecta_fs_1unpack_1composite_1packet(
    JNIEnv *env, jclass clazz, jlong devPtr, jobject outObj)
{
    fs_device_info_t *dev = (fs_device_info_t *)devPtr;
    const fs_imu_composite_packet_t *src = &dev->last_received_packet.composite;

    if (!dev || !outObj)
        return -1;

    fill_composite_packet(env, outObj, src);
    return 0;
}

JNIEXPORT jint JNICALL
Java_com_trifecta_fs_1unpack_1regular_1packet(
    JNIEnv *env, jclass clazz, jlong devPtr, jobject outObj)
{
    fs_device_info_t *dev = (fs_device_info_t *)devPtr;
    const fs_imu_regular_packet_t *src = &dev->last_received_packet.regular;

    // fill Java fields here...
    return 0;
}

JNIEXPORT jint JNICALL
Java_com_trifecta_fs_1unpack_1composite2_1packet(
    JNIEnv *env, jclass clazz, jlong devPtr, jobject outObj)
{
    fs_device_info_t *dev = (fs_device_info_t *)devPtr;
    const fs_imu_composite_packet_2_t *src = &dev->last_received_packet.composite2;

    if (!dev || !outObj)
        return -1;

    fill_composite2_packet(env, outObj, src);
    return 0;
}
