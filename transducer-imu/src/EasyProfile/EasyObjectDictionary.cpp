/**
 * EasyObjectDictionary.cpp
 * @author COPYRIGHT(c) 2017 SYD Dynamics ApS
 * @see    EasyObjectDictionary.h for more descriptions.
 */
#include "EasyProfile/EasyObjectDictionary.hpp"


EasyObjectDictionary::EasyObjectDictionary() {
    // Find the maximum size:
    int iDS = 0, oDS = 0;
    // ReSharper disable CppDFAConstantConditions
    if (iDS < static_cast<int>(sizeof(Ep_Request)))
        iDS = sizeof(Ep_Request);
    if (iDS < static_cast<int>(sizeof(Ep_Ack)))
        iDS = sizeof(Ep_Ack);
    if (iDS < static_cast<int>(sizeof(Ep_Status)))
        iDS = sizeof(Ep_Status);
    if (iDS < static_cast<int>(sizeof(Ep_Raw_GyroAccMag)))
        iDS = sizeof(Ep_Raw_GyroAccMag);
    if (iDS < static_cast<int>(sizeof(Ep_Q_s1_s)))
        iDS = sizeof(Ep_Q_s1_s);
    if (iDS < static_cast<int>(sizeof(Ep_Q_s1_e)))
        iDS = sizeof(Ep_Q_s1_e);
    if (iDS < static_cast<int>(sizeof(Ep_Euler_s1_s)))
        iDS = sizeof(Ep_Euler_s1_s);
    if (iDS < static_cast<int>(sizeof(Ep_Euler_s1_e)))
        iDS = sizeof(Ep_Euler_s1_e);
    if (iDS < static_cast<int>(sizeof(Ep_RPY)))
        iDS = sizeof(Ep_RPY);
    if (iDS < static_cast<int>(sizeof(Ep_Gravity)))
        iDS = sizeof(Ep_Gravity);
    if (iDS < static_cast<int>(sizeof(Ep_Combo)))
        iDS = sizeof(Ep_Combo);
    // ReSharper restore CppDFAConstantConditions

    oDS = iDS;
    maxSize = oDS;

    // QT Meta-Type Registration:
#ifdef EP_PLATFORM_QT5_
    qRegisterMetaType<Ep_Header>("Ep_Header");
    qRegisterMetaType<Ep_Request>("Ep_Request");
    qRegisterMetaType<Ep_Ack>("Ep_Ack");
    qRegisterMetaType<Ep_Status>("Ep_Status");
    qRegisterMetaType<Ep_Raw_GyroAccMag>("Ep_Raw_GyroAccMag");
    qRegisterMetaType<Ep_Q_s1_s>("Ep_Q_s1_s");
    qRegisterMetaType<Ep_Q_s1_e>("Ep_Q_s1_e");
    qRegisterMetaType<Ep_Euler_s1_s>("Ep_Euler_s1_s");
    qRegisterMetaType<Ep_Euler_s1_e>("Ep_Euler_s1_e");
    qRegisterMetaType<Ep_RPY>("Ep_RPY");
    qRegisterMetaType<Ep_Gravity>("Ep_Gravity");
    qRegisterMetaType<Ep_Combo>("Ep_Combo");
#endif

    // Value Reset:
    ep_Raw_GyroAccMag.timeStamp = 0;
    ep_Raw_GyroAccMag.acc[0] = 0;
    ep_Raw_GyroAccMag.acc[1] = 0;
    ep_Raw_GyroAccMag.acc[2] = 0;
    ep_Raw_GyroAccMag.gyro[0] = 0;
    ep_Raw_GyroAccMag.gyro[1] = 0;
    ep_Raw_GyroAccMag.gyro[2] = 0;
    ep_Raw_GyroAccMag.mag[0] = 0;
    ep_Raw_GyroAccMag.mag[1] = 0;
    ep_Raw_GyroAccMag.mag[2] = 0;

    // Initialize Dynamic Database of Object Items:
    {
        // ReSharper disable once CppLocalVariableMayBeConst
        EOD_DB_Dynamic arr[EOD_DB_SIZE_] = EOD_DB_DYNAMIC_INIT;
        for (int i = 0; i < static_cast<signed>(EOD_DB_SIZE_ * sizeof(EOD_DB_Dynamic)); i++) {
            *(reinterpret_cast<char*>(eOD_DB_Dynamic) + i) = *(reinterpret_cast<const char*>(arr) + i);
        }
    }
}


EasyObjectDictionary::~EasyObjectDictionary() {}


int EasyObjectDictionary::Get_MaxSize() const {
    return maxSize;
}


//------------------------------------------------------------------------
//                 Object Specific Read & Write Operations
//------------------------------------------------------------------------
/** @note Use the following method to read/write are intrinsically "safe" since they
  *       use the Read() and Write() method for low-level operation, which uses
  *       mutex to ensure the integrality of the data, and protect from unwanted changes
  *       of Object data once it is WriteProtected.
  *
  * @return EP_SUCC_            Means Operation done successfully
  *         EP_FAIL_            lengthIn doesn't match the type size, or the cmd does not exist in the object dictionary
  *         EP_MUTEX_LOCKED_    Permission denied while trying to access the Object.
 */

int EasyObjectDictionary::Write_Ep_Ack(
    const EP_ID_TYPE_ toId, ///< [INPUT] Destination ID of the package when it is to be sent
    const EP_CMD_TYPE_ cmdAck ///< [INPUT] The cmd it acknowledges to
) {
    const auto ep_Ack = Ep_Ack{
        .header = Ep_Header{
            .cmd = EP_CMD_Raw_GYRO_ACC_MAG_,
            .qos = global_SysQoS & EP_QOS_MASK_,
            .fromId = global_SysShortId & EP_ID_MASK_,
            .toId = toId & EP_ID_MASK_
        },
        .cmdAck = cmdAck
    };
    Ep_Header headerOut;
    return Write(reinterpret_cast<const char*>(&ep_Ack), sizeof(Ep_Ack), &headerOut);
}

int EasyObjectDictionary::Write_Ep_Status(
    const EP_ID_TYPE_ toId, ///< [INPUT]
    const uint32 timeStamp, ///< [INPUT]
    const float32 temperature, ///< [INPUT]
    const uint16 updateRate ///< [INPUT]
) {
    const Ep_Status ep_Status = {
        .header = Ep_Header{
            .cmd = EP_CMD_STATUS_,
            .qos = global_SysQoS & EP_QOS_MASK_,
            .fromId = global_SysShortId & EP_ID_MASK_,
            .toId = toId & EP_ID_MASK_
        },
        .timeStamp = timeStamp,
        .temperature = temperature,
        .updateRate = updateRate
    };
    Ep_Header headerOut;
    return Write(reinterpret_cast<const char*>(&ep_Status), sizeof(Ep_Status), &headerOut);
}

int EasyObjectDictionary::Write_Ep_Q_s1_s(
    const EP_ID_TYPE_ toId, ///< [INPUT]
    const uint32 timeStamp, ///< [INPUT]
    const float q1, const float q2, const float q3, const float q4 ///< [INPUT]
) {
    const auto ep_Q_s1_s = Ep_Q_s1_s{
        .header = Ep_Header{
            .cmd = EP_CMD_Q_S1_S_,
            .qos = global_SysQoS & EP_QOS_MASK_,
            .fromId = global_SysShortId & EP_ID_MASK_,
            .toId = toId & EP_ID_MASK_
        },
        .timeStamp = timeStamp,
        .q = {q1, q2, q3, q4}
    };
    Ep_Header headerOut;
    return Write(reinterpret_cast<const char*>(&ep_Q_s1_s), sizeof(Ep_Q_s1_s), &headerOut);
}

int EasyObjectDictionary::Write_Ep_Raw_GyroAccMag(
    const EP_ID_TYPE_ toId, ///< [INPUT]
    const uint32 timeStamp, ///< [INPUT]
    const float wx, const float wy, const float wz, ///< [INPUT]
    const float ax, const float ay, const float az, ///< [INPUT]
    const float mx, const float my, const float mz ///< [INPUT]
) {
    const auto ep_Raw_GyroAccMag = Ep_Raw_GyroAccMag{
        .header = Ep_Header{
            .cmd = EP_CMD_Raw_GYRO_ACC_MAG_,
            .qos = global_SysQoS & EP_QOS_MASK_,
            .fromId = global_SysShortId & EP_ID_MASK_,
            .toId = toId & EP_ID_MASK_
        },
        .timeStamp = timeStamp,
        .gyro = {wx, wy, wz},
        .acc = {ax, ay, az},
        .mag = {mx, my, mz}
    };
    Ep_Header headerOut;
    return Write(reinterpret_cast<const char*>(&ep_Raw_GyroAccMag), sizeof(Ep_Raw_GyroAccMag), &headerOut);
}

int EasyObjectDictionary::Write_Ep_Euler_s1_s(
    const EP_ID_TYPE_ toId, ///< [INPUT]
    const uint32 timeStamp, ///< [INPUT]
    const float psi, const float theta, const float phi ///< [INPUT]
) {
    const auto ep_Euler_s1_s = Ep_Euler_s1_s{
        .header = Ep_Header{
            .cmd = EP_CMD_EULER_S1_S_,
            .qos = global_SysQoS & EP_QOS_MASK_,
            .fromId = global_SysShortId & EP_ID_MASK_,
            .toId = toId & EP_ID_MASK_
        },
        .timeStamp = timeStamp,
        .psi = psi,
        .theta = theta,
        .phi = phi
    };
    Ep_Header headerOut;
    return Write(reinterpret_cast<const char*>(&ep_Euler_s1_s), sizeof(Ep_Euler_s1_s), &headerOut);
}

int EasyObjectDictionary::Write_Ep_Q_s1_e(
    const EP_ID_TYPE_ toId, ///< [INPUT]
    const uint32 timeStamp, ///< [INPUT]
    const float q1, const float q2, const float q3, const float q4 ///< [INPUT]
) {
    const auto ep_Q_s1_e = Ep_Q_s1_e{
        .header = Ep_Header{
            .cmd = EP_CMD_Q_S1_E_,
            .qos = global_SysQoS & EP_QOS_MASK_,
            .fromId = global_SysShortId & EP_ID_MASK_,
            .toId = toId & EP_ID_MASK_
        },
        .timeStamp = timeStamp,
        .q = {q1, q2, q3, q4}
    };
    Ep_Header headerOut;
    return Write(reinterpret_cast<const char*>(&ep_Q_s1_e), sizeof(Ep_Q_s1_e), &headerOut);
}

int EasyObjectDictionary::Write_Ep_Euler_s1_e(
    const EP_ID_TYPE_ toId, ///< [INPUT]
    const uint32 timeStamp, ///< [INPUT]
    const float psi, const float theta, const float phi ///< [INPUT]
) {
    const auto ep_Euler_s1_e = Ep_Euler_s1_e{
        .header = Ep_Header{
            .cmd = EP_CMD_EULER_S1_E_,
            .qos = global_SysQoS & EP_QOS_MASK_,
            .fromId = global_SysShortId & EP_ID_MASK_,
            .toId = toId & EP_ID_MASK_
        },
        .timeStamp = timeStamp,
        .psi = psi,
        .theta = theta,
        .phi = phi
    };
    Ep_Header headerOut;
    return Write(reinterpret_cast<const char*>(&ep_Euler_s1_e), sizeof(Ep_Euler_s1_e), &headerOut);
}

int EasyObjectDictionary::Write_Ep_RPY(
    const EP_ID_TYPE_ toId, ///< [INPUT]
    const uint32 timeStamp, ///< [INPUT]
    const float roll, const float pitch, const float yaw ///< [INPUT]
) {
    const auto ep_RPY = Ep_RPY{
        .header = Ep_Header{
            .cmd = EP_CMD_RPY_,
            .qos = global_SysQoS & EP_QOS_MASK_,
            .fromId = global_SysShortId & EP_ID_MASK_,
            .toId = toId & EP_ID_MASK_
        },
        .timeStamp = timeStamp,
        .roll = roll,
        .pitch = pitch,
        .yaw = yaw
    };
    Ep_Header headerOut;
    return Write(reinterpret_cast<const char*>(&ep_RPY), sizeof(Ep_RPY), &headerOut);
}

int EasyObjectDictionary::Write_Ep_Gravity(
    const EP_ID_TYPE_ toId, ///< [INPUT]
    const uint32 timeStamp, ///< [INPUT]
    const float gravityX, const float gravityY, const float gravityZ ///< [INPUT]
) {
    const auto ep_Gravity = Ep_Gravity{
        .header = Ep_Header{
            .cmd = EP_CMD_GRAVITY_,
            .qos = global_SysQoS & EP_QOS_MASK_,
            .fromId = global_SysShortId & EP_ID_MASK_,
            .toId = toId & EP_ID_MASK_
        },
        .timeStamp = timeStamp,
        .g = {gravityX, gravityY, gravityZ}
    };
    Ep_Header headerOut;
    return Write(reinterpret_cast<const char*>(&ep_Gravity), sizeof(Ep_Gravity), &headerOut);
}

int EasyObjectDictionary::Write_Ep_Combo(
    const EP_ID_TYPE_ toId, ///< [INPUT]
    const uint32 timeStamp, ///< [INPUT]
    const Ep_Status_SysState sys, ///< [INPUT]
    const int16 roll, const int16 pitch, const uint16 yaw, ///< [INPUT]
    const int32 q1, const int32 q2, const int32 q3, const int32 q4, ///< [INPUT]
    const int32 wx, const int32 wy, const int32 wz, ///< [INPUT]
    const int32 ax, const int32 ay, const int32 az, ///< [INPUT]
    const int16 mx, const int16 my, const int16 mz, ///< [INPUT]
    const int8 temperature, const uint8 updateRate, const uint16 reserved1, const uint16 simpleChecksum ///< [INPUT]
) {
    const auto ep_Combo = Ep_Combo{
        .header = Ep_Header{
            .cmd = EP_CMD_COMBO_,
            .qos = global_SysQoS & EP_QOS_MASK_,
            .fromId = global_SysShortId & EP_ID_MASK_,
            .toId = toId & EP_ID_MASK_
        },
        .timeStamp = timeStamp,
        .sysState = sys,
        .roll = roll,
        .pitch = pitch,
        .yaw = yaw,
        .q1 = q1,
        .q2 = q2,
        .q3 = q3,
        .q4 = q4,
        .wx = wx,
        .wy = wy,
        .wz = wz,
        .ax = ax,
        .ay = ay,
        .az = az,
        .mx = mx,
        .my = my,
        .mz = mz,
        .temperature = temperature,
        .updateRate = updateRate,
        .reserved1 = reserved1,
        .simpleChecksum = simpleChecksum
    };
    Ep_Header headerOut;

    return Write(reinterpret_cast<const char*>(&ep_Combo), sizeof(Ep_Combo), &headerOut);
}

/**
 * @section Write_Ep_*() Example
   @code
        if(EP_SUCC_ == Write_Ep_Request( 123, EP_CMD_RPY_ )){
            // write operation successfuly done, do something ...
        }
   @endcode
 */
int EasyObjectDictionary::Write_Ep_Request(
    const EP_ID_TYPE_ toId, ///< [INPUT]
    const EP_CMD_TYPE_ cmdRequest ///< [INPUT]
) {
    const auto ep_Request = Ep_Request{
        .header = Ep_Header{
            .cmd = EP_CMD_REQUEST_,
            .qos = global_SysQoS & EP_QOS_MASK_,
            .fromId = global_SysShortId & EP_ID_MASK_,
            .toId = toId & EP_ID_MASK_
        },
        .cmdRequest = cmdRequest
    };
    Ep_Header headerOut;
    return Write(reinterpret_cast<const char*>(&ep_Request), sizeof(Ep_Request), &headerOut);
}


/**
 * @section Read_Ep_*() Example
 * @code
          Ep_Request ep_Request;
          if(EP_SUCC_ == eOD.Read_Ep_Request(&ep_Request)){
                // Use the data saved in ep_Request
          }
 * @endcode
 */
int EasyObjectDictionary::Read_Ep_Request(
    Ep_Request* dataOut
) {
    constexpr EP_CMD_TYPE_ cmd = EP_CMD_REQUEST_;
    int odLengthOut;
    char* odDataOut;
    int retVal = EP_FAIL_;
    if (dataOut == nullptr) return EP_FAIL_;

    retVal = Read(cmd, &odDataOut, &odLengthOut);
    if (retVal == EP_SUCC_) {
        if (odLengthOut == sizeof(Ep_Request)) {
            for (int i = 0; i < odLengthOut; i++) {
                *(reinterpret_cast<char*>(dataOut) + i) = odDataOut[i];
            }
        }
    }
    EOD_DB_SetReadProtect(cmd, false);
    return retVal;
}

int EasyObjectDictionary::Read_Ep_Ack(
    Ep_Ack* dataOut
) {
    constexpr EP_CMD_TYPE_ cmd = EP_CMD_ACK_;
    int odLengthOut;
    char* odDataOut;
    int retVal = EP_FAIL_;
    if (dataOut == nullptr) return EP_FAIL_;

    retVal = Read(cmd, &odDataOut, &odLengthOut);
    if (retVal == EP_SUCC_) {
        if (odLengthOut == sizeof(Ep_Ack)) {
            for (int i = 0; i < odLengthOut; i++) {
                *(reinterpret_cast<char*>(dataOut) + i) = odDataOut[i];
            }
        }
    }
    EOD_DB_SetReadProtect(cmd, false);
    return retVal;
}

int EasyObjectDictionary::Read_Ep_Status(
    Ep_Status* dataOut
) {
    constexpr EP_CMD_TYPE_ cmd = EP_CMD_STATUS_;
    int odLengthOut;
    char* odDataOut;
    int retVal = EP_FAIL_;
    if (dataOut == nullptr) return EP_FAIL_;

    retVal = Read(cmd, &odDataOut, &odLengthOut);
    if (retVal == EP_SUCC_) {
        if (odLengthOut == sizeof(Ep_Status)) {
            for (int i = 0; i < odLengthOut; i++) {
                *(reinterpret_cast<char*>(dataOut) + i) = odDataOut[i];
            }
        }
    }
    EOD_DB_SetReadProtect(cmd, false);
    return retVal;
}

int EasyObjectDictionary::Read_Ep_Raw_GyroAccMag(
    Ep_Raw_GyroAccMag* dataOut
) {
    constexpr EP_CMD_TYPE_ cmd = EP_CMD_Raw_GYRO_ACC_MAG_;
    int odLengthOut;
    char* odDataOut;
    int retVal = EP_FAIL_;
    if (dataOut == nullptr) return EP_FAIL_;

    retVal = Read(cmd, &odDataOut, &odLengthOut);
    if (retVal == EP_SUCC_) {
        if (odLengthOut == sizeof(Ep_Raw_GyroAccMag)) {
            for (int i = 0; i < odLengthOut; i++) {
                *(reinterpret_cast<char*>(dataOut) + i) = odDataOut[i];
            }
        }
    }
    EOD_DB_SetReadProtect(cmd, false);
    return retVal;
}

int EasyObjectDictionary::Read_Ep_Q_s1_s(
    Ep_Q_s1_s* dataOut
) {
    constexpr EP_CMD_TYPE_ cmd = EP_CMD_Q_S1_S_;
    int odLengthOut;
    char* odDataOut;
    int retVal = EP_FAIL_;
    if (dataOut == nullptr) return EP_FAIL_;

    retVal = Read(cmd, &odDataOut, &odLengthOut);
    if (retVal == EP_SUCC_) {
        if (odLengthOut == sizeof(Ep_Q_s1_s)) {
            for (int i = 0; i < odLengthOut; i++) {
                *(reinterpret_cast<char*>(dataOut) + i) = odDataOut[i];
            }
        }
    }
    EOD_DB_SetReadProtect(cmd, false);
    return retVal;
}

int EasyObjectDictionary::Read_Ep_Euler_s1_s(
    Ep_Euler_s1_s* dataOut
) {
    constexpr EP_CMD_TYPE_ cmd = EP_CMD_EULER_S1_S_;
    int odLengthOut;
    char* odDataOut;
    int retVal = EP_FAIL_;
    if (dataOut == nullptr) return EP_FAIL_;

    retVal = Read(cmd, &odDataOut, &odLengthOut);
    if (retVal == EP_SUCC_) {
        if (odLengthOut == sizeof(Ep_Euler_s1_s)) {
            for (int i = 0; i < odLengthOut; i++) {
                *(reinterpret_cast<char*>(dataOut) + i) = odDataOut[i];
            }
        }
    }
    EOD_DB_SetReadProtect(cmd, false);
    return retVal;
}

int EasyObjectDictionary::Read_Ep_Q_s1_e(
    Ep_Q_s1_e* dataOut
) {
    constexpr EP_CMD_TYPE_ cmd = EP_CMD_Q_S1_E_;
    int odLengthOut;
    char* odDataOut;
    int retVal = EP_FAIL_;
    if (dataOut == nullptr) return EP_FAIL_;

    retVal = Read(cmd, &odDataOut, &odLengthOut);
    if (retVal == EP_SUCC_) {
        if (odLengthOut == sizeof(Ep_Q_s1_e)) {
            for (int i = 0; i < odLengthOut; i++) {
                *(reinterpret_cast<char*>(dataOut) + i) = odDataOut[i];
            }
        }
    }
    EOD_DB_SetReadProtect(cmd, false);
    return retVal;
}

int EasyObjectDictionary::Read_Ep_Euler_s1_e(
    Ep_Euler_s1_e* dataOut
) {
    constexpr EP_CMD_TYPE_ cmd = EP_CMD_EULER_S1_E_;
    int odLengthOut;
    char* odDataOut;
    int retVal = EP_FAIL_;
    if (dataOut == nullptr) return EP_FAIL_;

    retVal = Read(cmd, &odDataOut, &odLengthOut);
    if (retVal == EP_SUCC_) {
        if (odLengthOut == sizeof(Ep_Euler_s1_e)) {
            for (int i = 0; i < odLengthOut; i++) {
                *(reinterpret_cast<char*>(dataOut) + i) = odDataOut[i];
            }
        }
    }
    EOD_DB_SetReadProtect(cmd, false);
    return retVal;
}

int EasyObjectDictionary::Read_Ep_RPY(
    Ep_RPY* dataOut
) {
    constexpr EP_CMD_TYPE_ cmd = EP_CMD_RPY_;
    int odLengthOut;
    char* odDataOut;
    int retVal = EP_FAIL_;
    if (dataOut == nullptr) return EP_FAIL_;

    retVal = Read(cmd, &odDataOut, &odLengthOut);
    if (retVal == EP_SUCC_) {
        if (odLengthOut == sizeof(Ep_RPY)) {
            for (int i = 0; i < odLengthOut; i++) {
                *(reinterpret_cast<char*>(dataOut) + i) = odDataOut[i];
            }
        }
    }
    EOD_DB_SetReadProtect(cmd, false);
    return retVal;
}

int EasyObjectDictionary::Read_Ep_Gravity(
    Ep_Gravity* dataOut
) {
    constexpr EP_CMD_TYPE_ cmd = EP_CMD_GRAVITY_;
    int odLengthOut;
    char* odDataOut;
    int retVal = EP_FAIL_;
    if (dataOut == nullptr) return EP_FAIL_;

    retVal = Read(cmd, &odDataOut, &odLengthOut);
    if (retVal == EP_SUCC_) {
        if (odLengthOut == sizeof(Ep_Gravity)) {
            for (int i = 0; i < odLengthOut; i++) {
                *(reinterpret_cast<char*>(dataOut) + i) = odDataOut[i];
            }
        }
    }
    EOD_DB_SetReadProtect(cmd, false);
    return retVal;
}


int EasyObjectDictionary::Read_Ep_Combo(
    Ep_Combo* dataOut
) {
    constexpr EP_CMD_TYPE_ cmd = EP_CMD_COMBO_;
    int odLengthOut;
    char* odDataOut;
    int retVal = EP_FAIL_;
    if (dataOut == nullptr) return EP_FAIL_;

    retVal = Read(cmd, &odDataOut, &odLengthOut);
    if (retVal == EP_SUCC_) {
        if (odLengthOut == sizeof(Ep_Combo)) {
            for (int i = 0; i < odLengthOut; i++) {
                *(reinterpret_cast<char*>(dataOut) + i) = odDataOut[i];
            }
        }
    }
    EOD_DB_SetReadProtect(cmd, false);
    return retVal;
}

//------------------------------------------------------------------------
//              Object Specific Read & Write Operations
//------------------------------------------------------------------------


/**
 * EOD_DB_FindKey
 *
 * @return key of the item in DataBase.
 *          Range [0, (EOD_DB_SIZE_-1)]
 *          return -1 means not found
 */
int EasyObjectDictionary::EOD_DB_FindKey(
    const EP_CMD_TYPE_ cmdIn ///< [INPUT] the search keyword cmd
) {
    for (int i = 0; i < EOD_DB_SIZE_; i++) {
        if (eOD_DB_Static[i].cmd == cmdIn) {
            return i;
        }
    }
    return -1;
}


/**
  * Set & Reset write protection flag
  * @note   When WriteProtect is enabled to a specific Object:
  *            (1) Write() permission denied: no changes can be made to the Object;
  *            (2) Read() permission is still avilable.
  * @return EP_SUCC_            Setup Successful
  *         EP_MUTEX_LOCKED_    The cmd is already write protected
  *         EP_FAIL_            cmd does not exist in the object dictionary,
  */
int EasyObjectDictionary::EOD_DB_SetWriteProtect(
    const EP_CMD_TYPE_ cmdIn, ///< [INPUT] the search keyword cmd
    const bool enable ///< [INPUT] true: enable write protection; false: disable.
) {
    if (const int key = EOD_DB_FindKey(cmdIn); key == -1) {
        return EP_FAIL_;
    } else {
        if (enable) {
            if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_WRITE_PROTECT_)
                return EP_MUTEX_LOCKED_;
            // Mutex: set EOD_MUTEX_WRITE_PROTECT_ bit:
            eOD_DB_Dynamic[key].mutex |= EOD_MUTEX_WRITE_PROTECT_;
        } else {
            // Mutex: clear EOD_MUTEX_WRITE_PROTECT_ bit:
            eOD_DB_Dynamic[key].mutex &= ~EOD_MUTEX_WRITE_PROTECT_;
        }
    }
    return EP_SUCC_;
}


/**
  * Set & Reset read protection flag
  * @note   When ReadProtect is enabled to a specific Object:
  *            (1) Write() permission is denied: no changes can be made to the Object;
  *            (2) Read()  permission is also denied (to avoid another Read() from initiating
  *                           during the protection period, since the end of protection of the
  *                           previous Read() also means the end of protection of the latter,
  *                           which may lead to unexpected system behavior).
  * @return EP_SUCC_            Setup Successful
  *         EP_MUTEX_LOCKED_    The cmd is already read protected
  *         EP_FAIL_            cmd does not exist in the object dictionary
  */
int EasyObjectDictionary::EOD_DB_SetReadProtect(
    const EP_CMD_TYPE_ cmdIn, ///< [INPUT] the search keyword cmd
    const bool enable ///< [INPUT] true: enable read protection; false: disable.
) {
    if (const int key = EOD_DB_FindKey(cmdIn); key == -1) {
        return EP_FAIL_;
    } else {
        if (enable) {
            if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_READ_PROTECT_)
                return EP_MUTEX_LOCKED_;

            // Mutex: set EOD_MUTEX_READ_PROTECT_ bit :
            eOD_DB_Dynamic[key].mutex |= EOD_MUTEX_READ_PROTECT_;
        } else {
            // Mutex: clear EOD_MUTEX_READ_PROTECT_ bit:
            eOD_DB_Dynamic[key].mutex &= ~EOD_MUTEX_READ_PROTECT_;
        }
    }
    return EP_SUCC_;
}


//------------------------------------------------------------------------
//                                 Read
//------------------------------------------------------------------------
/**
  * Fetch the pointer to the Object from the dictinoary specificed the identifier cmdIn
  *
  * @return EP_SUCC_            Read Successful
  *         EP_FAIL_            cmd does not exist in the object dictionary
  *         EP_MUTEX_LOCKED_    Read permission denied.
  */
int EasyObjectDictionary::Read(
    const EP_CMD_TYPE_ cmdIn, ///< [INPUT]  Identifier to specify which object in the dictionary
    char** dataOut, ///< [OUTPUT] Pointer to the object
    int* lengthOut ///< [OUTPUT] Length (size) of the object
) {
    // Mutex Protection:
    const int key = EOD_DB_FindKey(cmdIn);
    if (key == -1) {
        return EP_FAIL_;
    } else {
        if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_READ_PROTECT_)
            return EP_MUTEX_LOCKED_;
        if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_LOCKED_)
            return EP_MUTEX_LOCKED_;
    }

    // Enable ReadProtection:
    eOD_DB_Dynamic[key].mutex |= EOD_MUTEX_READ_PROTECT_;

    // Read data:
    *dataOut = static_cast<char*>(eOD_DB_Dynamic[key].data);
    *lengthOut = eOD_DB_Static[key].size;
    return EP_SUCC_;
}


/**
  *  Read Only the header
  * @return EP_SUCC_            Read Successful
  *         EP_FAIL_            cmd does not exist in the object dictionary
  *         EP_MUTEX_LOCKED_    Read permission denied.
 */
int EasyObjectDictionary::Read_Header(
    const EP_CMD_TYPE_ cmdIn, ///< [INPUT]   Specify the Object where to read the header
    Ep_Header* headerOut ///< [OUTPUT]  Output the header
) {
    // Mutex Protection:
    const int key = EOD_DB_FindKey(cmdIn);
    if (key == -1) {
        return EP_FAIL_;
    } else {
        if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_READ_PROTECT_)
            return EP_MUTEX_LOCKED_;
        if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_LOCKED_)
            return EP_MUTEX_LOCKED_;
    }

    // Enable ReadProtection:
    // EOD_DB_SetReadProtect(cmdIn, true); // Same effect as below, but has lower efficiency in execution
    eOD_DB_Dynamic[key].mutex |= EOD_MUTEX_READ_PROTECT_;

    // Read data:
    const char* dataTmp = static_cast<char*>(eOD_DB_Dynamic[key].data);
    for (int i = 0; i < static_cast<signed>(sizeof(Ep_Header)); i++) {
        *(reinterpret_cast<char*>(headerOut) + i) = *(dataTmp + i);
    }

    // EOD_DB_SetReadProtect(cmdIn, false); // Same effect as below, but has lower efficiency in execution
    eOD_DB_Dynamic[key].mutex &= ~EOD_MUTEX_READ_PROTECT_;
    return EP_SUCC_;
}

/**
  * Analysis the input byte-array and fetch the "cmd" and "fromId" from the stream.
  * @return EP_SUCC_            Read successfully
  *         EP_FAIL_            lengthIn doesn't match the type size, or the cmd does not exist in the object dictionary
  */
int EasyObjectDictionary::Read_Header(
    const char* dataIn, ///< [INPUT] The data array
    const int lengthIn, ///< [INPUT] Size of the input data array
    Ep_Header* headerOut ///< [OUTPUT] interpreted header from the data array.
) {
    if (lengthIn < static_cast<signed>(sizeof(Ep_Header))) return EP_FAIL_;
    for (int i = 0; i < static_cast<signed>(sizeof(Ep_Header)); i++) {
        *(reinterpret_cast<char*>(headerOut) + i) = dataIn[i];
    }
    return EP_SUCC_;
}

//------------------------------------------------------------------------
//                                 Read
//------------------------------------------------------------------------


//------------------------------------------------------------------------
//                                 Write
//------------------------------------------------------------------------
/**
  * Copy data in a byte array into the relevant structure specified by the cmd at the beginning of the byte array.
  * @return EP_SUCC_            Write successfully
  *         EP_FAIL_            lengthIn doesn't match the type size, or the cmd does not exist in the object dictionary
  *         EP_MUTEX_LOCKED_    Write permission denied.
  */
int EasyObjectDictionary::Write(
    const char* dataIn, ///< [INPUT] The data array
    const int lengthIn, ///< [INPUT] Size of the input data array
    Ep_Header* headerOut ///< [OUTPUT] interpreted cmd from the data array.
) {
    int retVal = EP_SUCC_;

    // Fetch header form the data array:
    if (EP_FAIL_ == Read_Header(dataIn, lengthIn, headerOut)) {
        return EP_FAIL_;
    }

    // Mutex Protection:
    const int key = EOD_DB_FindKey(headerOut->cmd);
    if (key == -1) {
        return EP_FAIL_;
    } else {
        if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_LOCKED_)
            return EP_MUTEX_LOCKED_;
        if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_WRITE_PROTECT_)
            return EP_MUTEX_LOCKED_;
        if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_READ_PROTECT_)
            return EP_MUTEX_LOCKED_;
    }
    eOD_DB_Dynamic[key].mutex |= EOD_MUTEX_LOCKED_;

    // Write Operation:
    if (lengthIn == eOD_DB_Static[key].size) {
        const auto dataPtr = static_cast<char*>(eOD_DB_Dynamic[key].data);
        for (int i = 0; i < lengthIn; i++) {
            *(dataPtr + i) = *(dataIn + i);
        }
    } else {
        retVal = EP_FAIL_;
    }

    // Mutex: clear EOD_MUTEX_LOCKED_ bit:
    eOD_DB_Dynamic[key].mutex &= ~EOD_MUTEX_LOCKED_;
    return retVal;
}

/** Overwrite new header into Object specified by cmd
  * @return EP_SUCC_            Write successfully
  *         EP_FAIL_            lengthIn doesn't match the type size, or the cmd does not exist in the object dictionary
  *         EP_MUTEX_LOCKED_    Write permission denied.
  */
int EasyObjectDictionary::Write_Header(
    const EP_CMD_TYPE_ cmdIn, ///< [INPUT]  Specify which Object to overwrite the new header
    Ep_Header headerIn ///< [INPUT]  The new header
) {
    constexpr int retVal = EP_SUCC_;

    // Mutex Protection:
    const int key = EOD_DB_FindKey(cmdIn);
    if (key == -1) {
        return EP_FAIL_;
    } else {
        if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_LOCKED_)
            return EP_MUTEX_LOCKED_;
        if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_WRITE_PROTECT_)
            return EP_MUTEX_LOCKED_;
        if (eOD_DB_Dynamic[key].mutex & EOD_MUTEX_READ_PROTECT_)
            return EP_MUTEX_LOCKED_;
    }
    eOD_DB_Dynamic[key].mutex |= EOD_MUTEX_LOCKED_;

    // Write Operation :
    const auto dataPtr = static_cast<char*>(eOD_DB_Dynamic[key].data);
    for (int i = 0; i < static_cast<signed>(sizeof(Ep_Header)); i++) {
        *(dataPtr + i) = *(reinterpret_cast<char*>(&headerIn) + i);
    }

    // Mutex: clear EOD_MUTEX_LOCKED_ bit:
    eOD_DB_Dynamic[key].mutex &= ~EOD_MUTEX_LOCKED_;
    return retVal;
}


/**
 * Set the toId of an Object specified by the cmd without changing other contents
 * @return EP_SUCC_            Write successfully
 *         EP_FAIL_            fail
 *         EP_MUTEX_LOCKED_    Permission denied.
 */
int EasyObjectDictionary::Write_Header_toId(
    const EP_CMD_TYPE_ cmdIn, ///< [INPUT]
    const EP_ID_TYPE_ toIdIn ///< [INPUT]
) {
    Ep_Header header;
    int retVal = Read_Header(cmdIn, &header);
    if (retVal == EP_SUCC_) {
        header.toId = toIdIn;
        retVal = Write_Header(cmdIn, header);
    }
    return retVal;
}

//------------------------------------------------------------------------
//                                 Write
//------------------------------------------------------------------------
