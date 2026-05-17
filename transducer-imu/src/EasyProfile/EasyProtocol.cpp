/**
 * @file   EasyProtocol.cpp
 * @author COPYRIGHT(c) 2017 SYD Dynamics ApS
 * @see    EasyProtocol.h for more comments
 */
#include "EasyProfile/EasyProtocol.h"


EasyProtocol::EasyProtocol() {
    totalMem = 0;

    //------------------------------------------------------------------------------
    // Configure Parameters 2/2

    //Default Setting:
#ifdef EP_PLATFORM_QT5_
#define EP_QUEUE_SIZE_ (iDS*10)
#else
#define EP_QUEUE_SIZE_ (iDS*10)  // Smaller queue size for embedded syst.
#endif
    HEAD_1_ = static_cast<char>(0xaa);
    totalMem += sizeof(HEAD_1_);
    HEAD_2_ = static_cast<char>(0x55);
    totalMem += sizeof(HEAD_2_);
    HEAD_LENGTH_ = 2;
    totalMem += sizeof(HEAD_LENGTH_);
    SIZE_LENGTH_ = 1;
    totalMem += sizeof(SIZE_LENGTH_);
    MAX_PAYLOAD_SIZE_ = 120;
    totalMem += sizeof(MAX_PAYLOAD_SIZE_);

    CHECKSUM_OPTION_ = EP_CHECKSUM_2_BYTES_CRC_;
    totalMem += sizeof(CHECKSUM_OPTION_);
    totalMem += sizeof(CHECKSUM_LENGTH_);
    SetChecksumOption(CHECKSUM_OPTION_);
    HEAD_LENGTH_MSB_AHEAD_ = 0;
    totalMem += sizeof(HEAD_LENGTH_MSB_AHEAD_);
    ENABLE_ROUND_UP_ = 1;
    totalMem += sizeof(ENABLE_ROUND_UP_);
    ROUND_UP_NUM_ = 4;
    totalMem += sizeof(ROUND_UP_NUM_);

    totalMem += inQueue.Size() + 1; //new

    // Configure Parameters 2/2
    //------------------------------------------------------------------------------

    inBuf = outBuf = nullptr;
    iDS = oDS = 0;

#ifdef EP_TURN_ON_STATISTICS_
    sRecvByte_Total = 0;
    sRecvByte_BadHead = 0;
    sRecvByte_GoodHead = 0;
    sRecvPkg_Total = 0;
    sRecvPkg_Omit = 0;
    sRecvPkg_Good = 0;
    sRecvPkg_Bad = 0;
    sSendPkg = 0;
#endif
}


EasyProtocol::~EasyProtocol() {
    if (inBuf) {
        delete inBuf;
        inBuf = nullptr;
    }
    if (outBuf) {
        delete outBuf;
        outBuf = nullptr;
    }
}


/**
 * @return  SUCC_
 *          FAIL_   iDataSize and/or oDataSize too large.
 */
int EasyProtocol::Init(
    const int iDataMaxSize, ///< [INPUT]
    const int oDataMaxSize ///< [INPUT]
) {
    //totalMem = 0;                                                  //new
    if (iDS > MAX_PAYLOAD_SIZE_ || oDS > MAX_PAYLOAD_SIZE_) return EP_FAIL_;
    totalMem -= inQueue.Size() + 1; //new
    if (inBuf) {
        delete inBuf;
        inBuf = nullptr;
        totalMem -= iDS + EP_PKG_MODIFIER_SIZE_;
    }
    if (outBuf) {
        delete outBuf;
        outBuf = nullptr;
        totalMem -= oDS + EP_PKG_MODIFIER_SIZE_;
    }
    iDS = iDataMaxSize;
    oDS = oDataMaxSize;
    p = inBuf = new char[iDS + EP_PKG_MODIFIER_SIZE_];
    totalMem += iDS + EP_PKG_MODIFIER_SIZE_;
    outBuf = new char[oDS + EP_PKG_MODIFIER_SIZE_];
    totalMem += oDS + EP_PKG_MODIFIER_SIZE_;
    inQueue.resize(EP_QUEUE_SIZE_);
    totalMem += EP_QUEUE_SIZE_;

#ifdef EP_TURN_ON_STATISTICS_
    totalMem += 8 * sizeof(sRecvByte_Total);
#endif
    // ReSharper disable once CppDFAConstantConditions
    if (inBuf && outBuf)
        return EP_SUCC_; // new
    else return EP_FAIL_; // new
}


/**
 * @param option can be one of the following:
 *                 EP_CHECKSUM_1_BYTE_SUM_
 *                 EP_CHECKSUM_2_BYTES_SUM_
 *                 EP_CHECKSUM_2_BYTES_CRC_
 */
void EasyProtocol::SetChecksumOption(const char option) {
    CHECKSUM_OPTION_ = EP_CHECKSUM_1_BYTE_SUM_;
    CHECKSUM_LENGTH_ = 1;
    if (option == EP_CHECKSUM_2_BYTES_SUM_ ||
        option == EP_CHECKSUM_2_BYTES_CRC_) {
        CHECKSUM_OPTION_ = option;
        CHECKSUM_LENGTH_ = 2;
    }
}


/**
  * Debug Print
  */
#ifdef EP_TURN_ON_DEBUG_
void EasyProtocol::DebugPrint(QString title, char* data, int length) {
    qDebug() << " --- " << title << " --- length=" << length;
    QDebug* theQDebug = new QDebug(QtDebugMsg);
    for (int i = 0; i < length; i++) {
        QString t = QString("%1 ").arg((uint8)data[i], 0, 16);
        (*theQDebug) << t;
    }
    delete theQDebug;
    qDebug() << " --- --- ---";
}
#endif


/**
  * Get Statistics
  */
#ifdef EP_TURN_ON_STATISTICS_
int EasyProtocol::TotalBytesOfMemoryUsed() {
    return totalMem;
}

int EasyProtocol::GetStatistic_Recv_Byte_Total() {
    return sRecvByte_Total;
}

float EasyProtocol::GetStatistic_Recv_Byte_BadHeadRate() {
    return ((float)sRecvByte_BadHead / sRecvByte_Total);
}

int EasyProtocol::GetStatistic_Recv_Pkg_Total() {
    return sRecvPkg_Total;
}

float EasyProtocol::GetStatistic_Recv_Pkg_OmitRate() {
    return ((float)sRecvPkg_Omit / sRecvPkg_Total);
}

float EasyProtocol::GetStatistic_Recv_Pkg_BadRate() {
    return ((float)sRecvPkg_Bad / sRecvPkg_Total);
}

float EasyProtocol::GetStatistic_Recv_Pkg_GoodRate() {
    return ((float)sRecvPkg_Good / sRecvPkg_Total);
}

int EasyProtocol::GetStatistic_Send_Pkg_Total() {
    return sSendPkg;
}
#endif


int EasyProtocol::GetInDataMaxSize() const {
    return iDS;
}

int EasyProtocol::GetOutDataMaxSize() const {
    return oDS;
}

int EasyProtocol::GetRoundUp() const {
    return roundUpTmp;
}


/**
  * @brief   Construct package out of the payload data array
  * @return  EP_FAIL_    payloadSize too large or easyprotocol setting error.
  *          EP_SUCC_    Creation of a new package is successful.
  */
int EasyProtocol::CreateOutputPackage(
    char* payloadData, ///< [INPUT]  Payload data pointer
    const int payloadSize, ///< [INPUT]  Payload size
    char** packageData, ///< [OUTPUT] Package data pointer
    int* packageSize ///< [OUTPUT] Package size after construction
) {
    // Check validity:
    if (payloadSize > MAX_PAYLOAD_SIZE_) return EP_FAIL_;
    if (HEAD_LENGTH_ != 1 && HEAD_LENGTH_ != 2) return EP_FAIL_;
    if (SIZE_LENGTH_ != 1 && SIZE_LENGTH_ != 2) return EP_FAIL_;
    if (CHECKSUM_LENGTH_ != 1 && CHECKSUM_LENGTH_ != 2) return EP_FAIL_;
    if (nullptr == outBuf) return EP_FAIL_; //new

    // Head:
    outBuf[0] = HEAD_1_;
    if (HEAD_LENGTH_ == 2) {
        outBuf[1] = HEAD_2_;
    }

    // Data Length:
    if (SIZE_LENGTH_ == 1) {
        outBuf[static_cast<int16>(HEAD_LENGTH_)] = static_cast<int8>(payloadSize);
    } else if (/* ReSharper disable once CppDFAConstantConditions */ SIZE_LENGTH_ == 2) {
        if (!HEAD_LENGTH_MSB_AHEAD_) {
            *reinterpret_cast<int16*>(outBuf + HEAD_LENGTH_) = static_cast<unsigned int>(payloadSize);
        } else {
            outBuf[static_cast<int16>(HEAD_LENGTH_)] = static_cast<unsigned int>(payloadSize) >> 8; // MSB
            outBuf[static_cast<int16>(HEAD_LENGTH_) + 1] = static_cast<unsigned int>(payloadSize); // LSB
        }
    }

    // Payload:
    char* outBufTmp = outBuf + HEAD_LENGTH_ + SIZE_LENGTH_;
    for (int t = 0; t < payloadSize; t++) {
        outBufTmp[t] = payloadData[t];
    }

    // Results:
    *packageData = outBuf;
    *packageSize = payloadSize + EP_PKG_MODIFIER_SIZE_;

    // CheckSum:
    const uint16 checkSum = Checksum_Generate(outBuf + HEAD_LENGTH_, payloadSize + SIZE_LENGTH_);
    if (CHECKSUM_LENGTH_ == 1) {
        outBuf[*packageSize - CHECKSUM_LENGTH_] = checkSum;
    } else if (CHECKSUM_LENGTH_ == 2) {
        *reinterpret_cast<uint16*>(outBuf + *packageSize - CHECKSUM_LENGTH_) = checkSum;
    }

#ifdef EP_TURN_ON_DEBUG_
    DebugPrint("CreateOutputPackage()", *packageData, *packageSize);
#endif

#ifdef EP_TURN_ON_STATISTICS_
    sSendPkg++;
#endif
    return EP_SUCC_;
}


/**
 * @return Generate checksum.
 */
uint16 EasyProtocol::Checksum_Generate(
    char* data, ///< [INPUT] Data relevant for generating checksum.
    const int dataLength ///< [INPUT]
) {
    uint16 checkSum = 0;

    if (CHECKSUM_OPTION_ == EP_CHECKSUM_1_BYTE_SUM_ ||
        CHECKSUM_OPTION_ == EP_CHECKSUM_2_BYTES_SUM_) {
        for (int i = 0; i < dataLength; i++) {
            checkSum += static_cast<unsigned>(*reinterpret_cast<uint8*>(data + i));
        }
    } else {
        const unsigned char* d = reinterpret_cast<unsigned char*>(data);
        checkSum = 0xffff;
        for (int i = 0; i < dataLength; i++) {
            checkSum ^= static_cast<unsigned int>(*d++);
            for (int j = 0; j < 8; j++) {
                const unsigned char c = checkSum & 0x0001;
                checkSum >>= 1;
                if (c) checkSum ^= 0xa001;
            }
        }
    }
    return checkSum;
}


/**
 * @return   EP_SUCC_    check passed
 *           EP_FAIL_    check failed
 */
int EasyProtocol::Checksum_Verify(
    char* data, ///< [INPUT] Data relevant for verifying the checksum.
    const int dataLength, ///< [INPUT]
    const uint16 checksumToBeVerified ///< [INPUT] Depending on context, use the lower 8-bit or the entire 16-bit
) {
    uint16 checkSum = Checksum_Generate(data, dataLength);

    if (CHECKSUM_LENGTH_ == 1) {
        checkSum = static_cast<uint8>(checkSum);
    }
#ifdef EP_TURN_ON_DEBUG_
    qDebug() << "EasyProtocol: calcChecksum" << checkSum;
#endif
    if (checksumToBeVerified == checkSum) return EP_SUCC_;
    else return EP_FAIL_;
}


/**
 * @return EP_FAIL_
 *         EP_SUCC_
 */
int EasyProtocol::UnWrapInData(
    char* packageData, ///< [INPUT]
    char** payloadData, ///< [OUTPUT]
    int payloadSize ///< [INPUT]
) {
#ifdef EP_TURN_ON_DEBUG_
    DebugPrint("UnWrapInData() ", packageData + HEAD_LENGTH_ + SIZE_LENGTH_, payloadSize);
#endif

    uint16 recvChecksum = 0;

    if (CHECKSUM_LENGTH_ == 1) {
        recvChecksum = packageData[payloadSize + EP_PKG_MODIFIER_SIZE_ - CHECKSUM_LENGTH_];
    } else if (CHECKSUM_LENGTH_ == 2) {
        recvChecksum = *reinterpret_cast<uint16*>(packageData + payloadSize + EP_PKG_MODIFIER_SIZE_ - CHECKSUM_LENGTH_);
    }

#ifdef EP_TURN_ON_DEBUG_
    qDebug() << "EasyProtocol: recvChecksum" << recvChecksum;
#endif

    if (EP_SUCC_ != Checksum_Verify(packageData + HEAD_LENGTH_, payloadSize + SIZE_LENGTH_, recvChecksum)) {
#ifdef EP_TURN_ON_STATISTICS_
        sRecvPkg_Bad++;
#endif
        *payloadData = nullptr;
        return EP_FAIL_;
    } else {
        *payloadData = packageData + HEAD_LENGTH_ + SIZE_LENGTH_;
#ifdef EP_TURN_ON_STATISTICS_
        sRecvPkg_Good++;
#endif
        return EP_SUCC_;
    }
}


/**
 * @brief  Assemble received raw data into package and then fetch the payload wihtin it.
 * @return EP_NORMAL_EXIT_   No complete package is received.
 *         EP_FAIL_          Package received, but the checksum/parameter is wrong.
 *         EP_SUCC_          New package successfully received. Payload within it is returned
 *                               via **payloadData and *payloadSize.
 *         EP_QUEUE_EMPTY_   Cache queue is already empty.
 */
int EasyProtocol::AssembleInputPackage(
    char* rawData, ///< [INPUT]  Received raw data
    int rawDataLenth, ///< [INPUT]  Size (bytes) of the raw data
    char** payloadData, ///< [OUTPUT] Return a pointer to the payload of the successfuly received package.
    ///              The memory is allocated whitn this EasyProtocol object.
    ///              If no package is received successfully, it is set to 0.
    int* payloadSize ///< [OUTPUT] Return the size of the payload of the successfuly received package.
    ///              If no package is received successfully, it is set to 0.
) {
    char newDataReceived = 0;
    if (HEAD_LENGTH_ != 1 && HEAD_LENGTH_ != 2) return EP_FAIL_;
    if (SIZE_LENGTH_ != 1 && SIZE_LENGTH_ != 2) return EP_FAIL_;
    if (CHECKSUM_LENGTH_ != 1 && CHECKSUM_LENGTH_ != 2) return EP_FAIL_;
    if (nullptr == inBuf) return EP_FAIL_; // [v2.2]

#ifdef EP_TURN_ON_STATISTICS_
    sRecvByte_Total += rawDataLenth;
#endif

    for (int i = 0; i < rawDataLenth; i++) {
        inQueue.push(*(rawData + i));
    }

    while (!inQueue.empty()) {
        const char c = inQueue.front();
        inQueue.pop();
        if (p == inBuf) {
            if (c == HEAD_1_) {
                *p++ = c;
                omitStream = 'N';
#ifdef EP_TURN_ON_STATISTICS_
                sRecvByte_GoodHead++;
#endif
            }
#ifdef EP_TURN_ON_STATISTICS_
            else{
                sRecvByte_BadHead++;
            }
#endif
        } else if (HEAD_LENGTH_ == 2 && p == inBuf + 1) {
            if (c == HEAD_2_)
                *p++ = c;
            else
                p = inBuf;
        } else if (p >= inBuf + HEAD_LENGTH_ &&
            p < inBuf + HEAD_LENGTH_ + SIZE_LENGTH_) {
            *p++ = c;
            if (p >= inBuf + HEAD_LENGTH_ + SIZE_LENGTH_) {
                if (SIZE_LENGTH_ == 1) {
                    declaredPayloadSize = *reinterpret_cast<int8*>(p - SIZE_LENGTH_);
                } else if (SIZE_LENGTH_ == 2) {
                    if (!HEAD_LENGTH_MSB_AHEAD_) {
                        declaredPayloadSize = *reinterpret_cast<int16*>(p - SIZE_LENGTH_);
                    } else {
                        declaredPayloadSize =
                            (static_cast<uint16>(*reinterpret_cast<uint8*>(p - SIZE_LENGTH_)) << 8) +
                            static_cast<uint16>(*reinterpret_cast<uint8*>(p - SIZE_LENGTH_ + 1));
                    }
                }
                if (declaredPayloadSize > MAX_PAYLOAD_SIZE_ || declaredPayloadSize <= 0) {
                    p = inBuf;
                    continue;
                }

#ifdef EP_TURN_ON_DEBUG_
                qDebug() << "declaredPayloadSize=" << declaredPayloadSize;
                qDebug() << "iDS=" << iDS;
                qDebug() << "MAX_PAYLOAD_SIZE_=" << MAX_PAYLOAD_SIZE_;
#endif

                roundUpTmp = 0;
                if (ENABLE_ROUND_UP_) {
                    if (declaredPayloadSize % (int32)ROUND_UP_NUM_) {
                        roundUpTmp = static_cast<int32>(ROUND_UP_NUM_) - declaredPayloadSize % static_cast<int32>(ROUND_UP_NUM_);
                    }
                }
#ifdef EP_TURN_ON_DEBUG_
                qDebug() << "roundUpTmp=" << (int16)roundUpTmp;
#endif

                if (declaredPayloadSize <= iDS) {
                    omitStream = 'N';
#ifdef EP_TURN_ON_DEBUG_
                    qDebug() << "omitStream = 'N'";
#endif
                } else {
                    omitStream = 'Y';
#ifdef EP_TURN_ON_DEBUG_
                    qDebug() << "omitStream = 'Y'";
#endif
                }
            }
        } else {
            if (omitStream == 'N') {
                *p++ = c;
                if (p >= inBuf + declaredPayloadSize + EP_PKG_MODIFIER_SIZE_) {
#ifdef EP_TURN_ON_STATISTICS_
                    sRecvPkg_Total++;
#endif
                    p = inBuf;
                    newDataReceived = 1;
                    break;
                }
            } else {
                p++;
                if (p >= inBuf + declaredPayloadSize + EP_PKG_MODIFIER_SIZE_) {
#ifdef EP_TURN_ON_STATISTICS_
                    sRecvPkg_Total++;
                    sRecvPkg_Omit++;
#endif
                    p = inBuf;
                    break;
                }
            }
        }
    }

    if (newDataReceived) {
        if (EP_SUCC_ == UnWrapInData(inBuf, payloadData, declaredPayloadSize)) {
            *payloadSize = declaredPayloadSize;
            return EP_SUCC_;
        }
        return EP_FAIL_;
    }
    if (inQueue.empty()) {
        *payloadData = nullptr;
        *payloadSize = 0;
        return EP_QUEUE_EMPTY_;
    } else {
        *payloadData = nullptr;
        *payloadSize = 0;
        return EP_NORMAL_EXIT_;
    }
}
