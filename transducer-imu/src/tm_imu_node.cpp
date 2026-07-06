#include "tm_imu/tm_imu_node.hpp"

#include "EasyProfile/EasyObjectDictionary.h" // TransducerM Data
#include "EasyProfile/EasyProfile.h"          // TransducerM communication protocol

EasyObjectDictionary eOD;
EasyProfile eP(&eOD);

#ifdef DEBUG_MODE
#define DEBUG_MODE_PRINT_TIMER_MS_  (10000)  // 10 seconds
#endif

TMSerial::TMSerial()
    : Node("tm_imu"), logger(get_logger()) {
    // Declare node's parameters default value
    // [NOT NECESSARY TO CHANGE THE FOLLOWING, USE ../config/params.yaml INSTEAD !]
    this->declare_parameter("imu_baudrate", 115200);
    this->declare_parameter("imu_port", "/dev/ttyUSB0");
    this->declare_parameter("imu_frame_id", "imu");
    this->declare_parameter("parent_frame_id", "base_link");
    this->declare_parameter("timer_period", 50); // Unit: ms
    this->declare_parameter("transform", std::vector{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    // Declare a serial object
    serialib1 = new serialib;
    SerialportOpen();
    // Performance monitor
    #ifdef DEBUG_MODE
    count = 0;
    count2 = 0;
    timer_10 = this->create_wall_timer(std::chrono::milliseconds(DEBUG_MODE_PRINT_TIMER_MS_), std::bind(&TMSerial::TimerCallback2, this));
    #endif
    // Set frame id
    imu_data_msg.header.frame_id = this->get_parameter("imu_frame_id").as_string();
    imu_data_rpy_msg.header.frame_id = this->get_parameter("imu_frame_id").as_string();
    imu_data_mag_msg.header.frame_id = this->get_parameter("imu_frame_id").as_string();
    // Create publisher
    publisher_IMU = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    publisher_IMU_RPY = this->create_publisher<sensor_msgs::msg::MagneticField>("imu_data_rpy", 10);
    publisher_IMU_MAG = this->create_publisher<sensor_msgs::msg::MagneticField>("imu_data_mag", 10);
    // Create timer
    const auto period = std::chrono::milliseconds(this->get_parameter("timer_period").as_int());
    timer_ = this->create_wall_timer(period, [this] {
        TimerCallback();
    });
    // Create tf_broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}


TMSerial::~TMSerial() {
    if (serialib1) {
        serialib1->closeDevice();
        delete serialib1;
        serialib1 = nullptr;
    }
}


void TMSerial::TimerCallback() {
    // Read the values from the Imu sensor
    if (const bool res = OnSerialRX(); !res)
        return;
    // Update the header stamp
    // TODO 2026-03-02 (Will Free): use timestamp from the serial data
    imu_data_msg.header.stamp = this->get_clock()->now();
    imu_data_rpy_msg.header.stamp = this->get_clock()->now();
    imu_data_mag_msg.header.stamp = this->get_clock()->now();
    FillCovarianceMatrices();
    // Publish msg
    PublishTransform();
    publisher_IMU->publish(imu_data_msg);
    publisher_IMU_RPY->publish(imu_data_rpy_msg);
    publisher_IMU_MAG->publish(imu_data_mag_msg);
}


#ifdef DEBUG_MODE
void TMSerial::TimerCallback2() {
    logger.info("Rx byte cnt={}, TrasnducerM pkg cnt = {} ({} Hz)", count2, count, 1000 * ((float)count) / (DEBUG_MODE_PRINT_TIMER_MS_));
    count = 0;
    count2 = 0;
}
#endif


char TMSerial::SerialportOpen() const {
    const unsigned int baudrate = this->get_parameter("imu_baudrate").as_int();
    const auto Ret = serialib1->openDevice(this->get_parameter("imu_port").as_string().c_str(), baudrate,
                                           SERIAL_DATABITS_8, SERIAL_PARITY_NONE, SERIAL_STOPBITS_1); // Open serial link at the specified baudrate
    if (Ret != 1) {
        // If an error occured...
        logger.info("Error while opening port. Permission problem ?"); // ... display a message ...
        logger.info("imu_port:{} imu_baudrate:{}", this->get_parameter("imu_port").as_string(), baudrate);
        return Ret; // ... quit the application
    }
    logger.info("Serial port opened successfully !");
    logger.info("imu_port:{} imu_baudrate:{}", this->get_parameter("imu_port").as_string(), baudrate);
    return 1;
}


// TODO 2026-03-02 (Will Free): the datasheet says one number, and the code comment here says another number...
// constexpr auto GRAVITY = 9.794;
constexpr auto GRAVITY = 9.8158;

//! the magnetic field in Odense, Denmark (as the company is headquartered in Odense)
//!
//! from the documentation:<br>
//! (m1, m2, m3) represents the magnetic strength measured by TransducerM in its sensor frame.
//! If norm(m1, m2, m3) equals to 1, this means the magnetic strength is equal to the magnetic
//! strength measured during its factory calibration, which is the earth magnetic field
//! itself in Denmark. As such, the absolute value of the magnetic strength is not accurate when
//! TransducerM is moved to a different location, whereas the direction (m1, m2, m3) represents
//! and the relative strengtheners counts for measuring the attitude or as a digital compass
constexpr auto MAGNETIC_FACTORY_CALIBRATION = 50.5662 / std::nano::den;


bool TMSerial::OnSerialRX() {
    char serialBuffer[1024];
    int ret = serialib1->readBytes(serialBuffer, sizeof(serialBuffer), 1, 100);
    #ifdef DEBUG_MODE
    //logger.info("rxsize = {}", ret);
    count2 += ret;
    #endif
    if (ret <= 0)
        return false;
    // Step 1: read the received data buffer of the Serial Port
    char* rxData = serialBuffer; //         and then convert it to data types acceptable by the
    int rxSize = ret; //         Communication Abstraction Layer (CAL).
    Ep_Header header;
    while (EP_SUCC_ == eP.On_RecvPkg(rxData, rxSize, &header)) {
        // Step 2: Tell the CAL that new data has arrived.
        //         It does not matter if the new data only contains a fraction
        //         of a complete package, nor does it matter if the data is broken
        //         during the transmission. On_RecvPkg() will only return EP_SUCC_
        //         when a complete and correct package has arrived.
        rxData = nullptr; //         The while loop ensures no package is omitted.
        rxSize = 0;

        // Example Reading of the Short ID of the device who send the data:
        uint32 fromId = header.fromId; // Step 3.1:  Now we are able to read the received payload data.
        //            header.fromId tells us from which Motion Module the data comes.

        //Supress "parameter unused" complier warning:
        (void)fromId;

        switch (header.cmd) {
        // Step 3.2: header.cmd tells what kind of data is inside the payload.
        case EP_CMD_ACK_: {
            //           We can use a switch() as demonstrated here to do different
            Ep_Ack ep_Ack; //           tasks for different types of data.
            if (EP_SUCC_ == eOD.Read_Ep_Ack(&ep_Ack)) {}
        }
        break;
        case EP_CMD_STATUS_: {
            Ep_Status ep_Status;
            if (EP_SUCC_ == eOD.Read_Ep_Status(&ep_Status)) {}
        }
        break;
        case EP_CMD_Raw_GYRO_ACC_MAG_: {
            Ep_Raw_GyroAccMag ep_Raw_GyroAccMag;
            if (EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&ep_Raw_GyroAccMag)) {
                const auto gyro_x = ep_Raw_GyroAccMag.gyro[0];
                const auto gyro_y = ep_Raw_GyroAccMag.gyro[1];
                const auto gyro_z = ep_Raw_GyroAccMag.gyro[2];

                const auto acc_x = ep_Raw_GyroAccMag.acc[0] * GRAVITY;
                const auto acc_y = ep_Raw_GyroAccMag.acc[1] * GRAVITY;
                const auto acc_z = ep_Raw_GyroAccMag.acc[2] * GRAVITY;

                const auto mag_x = ep_Raw_GyroAccMag.mag[0] * MAGNETIC_FACTORY_CALIBRATION;
                const auto mag_y = ep_Raw_GyroAccMag.mag[1] * MAGNETIC_FACTORY_CALIBRATION;
                const auto mag_z = ep_Raw_GyroAccMag.mag[2] * MAGNETIC_FACTORY_CALIBRATION;

                imu_data_msg.angular_velocity.x = gyro_x;
                imu_data_msg.angular_velocity.y = gyro_y;
                imu_data_msg.angular_velocity.z = gyro_z;

                imu_data_msg.linear_acceleration.x = acc_x;
                imu_data_msg.linear_acceleration.y = acc_y;
                imu_data_msg.linear_acceleration.z = acc_z;

                imu_data_mag_msg.magnetic_field.x = mag_x;
                imu_data_mag_msg.magnetic_field.y = mag_y;
                imu_data_mag_msg.magnetic_field.z = mag_z;
                #ifdef DEBUG_MODE
                //logger.info("RAW pkg");
                #endif
            }
        }
        break;
        case EP_CMD_Q_S1_S_: {
            Ep_Q_s1_s ep_Q_s1_s;
            if (EP_SUCC_ == eOD.Read_Ep_Q_s1_s(&ep_Q_s1_s)) {
                // TODO 2026-03-02 (Will Free): why are we not using the quat here? how is Ep_Q_s1_s different from Ep_Q_s1_e?
            }
        }
        break;
        case EP_CMD_Q_S1_E_: {
            Ep_Q_s1_e ep_Q_s1_e;
            if (EP_SUCC_ == eOD.Read_Ep_Q_s1_e(&ep_Q_s1_e)) {
                // Step 3.3: If we decided that the received Quaternion should be used,
                //           Here is an example of how to access the Quaternion data.
                const auto q1 = ep_Q_s1_e.q[0];
                const auto q2 = ep_Q_s1_e.q[1];
                const auto q3 = ep_Q_s1_e.q[2];
                const auto q4 = ep_Q_s1_e.q[3];
                // uint32 timeStamp = ep_Q_s1_e.timeStamp;  //TimeStamp indicates the time point (since the Module has been powered on),
                //when this particular set of Quaternion was calculated. (Unit: uS)
                //Note that overflow will occured when the uint32 type reaches its maximum value.
                //The ID indicates the device Short ID telling which Motion Module the data comes from.
                // uint32 deviceId  = ep_Q_s1_e.header.fromId;
                imu_data_msg.orientation.w = q1;
                imu_data_msg.orientation.x = q2;
                imu_data_msg.orientation.y = q3;
                imu_data_msg.orientation.z = q4;
                #ifdef DEBUG_MODE
                //logger.info("QS1 pkg");
                #endif
            }
        }
        break;
        case EP_CMD_EULER_S1_S_: {
            Ep_Euler_s1_s ep_Euler_s1_s;
            if (EP_SUCC_ == eOD.Read_Ep_Euler_s1_s(&ep_Euler_s1_s)) {
                // TODO 2026-03-02 (Will Free): consider using/publishing these euler angles?
            }
        }
        break;
        case EP_CMD_EULER_S1_E_: {
            Ep_Euler_s1_e ep_Euler_s1_e;
            if (EP_SUCC_ == eOD.Read_Ep_Euler_s1_e(&ep_Euler_s1_e)) {
                // TODO 2026-03-02 (Will Free): consider using/publishing these euler angles?
            }
        }
        break;
        case EP_CMD_RPY_: {
            Ep_RPY ep_RPY;
            if (EP_SUCC_ == eOD.Read_Ep_RPY(&ep_RPY)) {
                //Another Example reading of the received Roll Pitch and Yaw
                float roll = ep_RPY.roll;
                float pitch = ep_RPY.pitch;
                float yaw = ep_RPY.yaw;
                //uint32 timeStamp = ep_RPY.timeStamp;
                //uint32 deviceId  = ep_RPY.header.fromId;
                imu_data_rpy_msg.magnetic_field.x = roll;
                imu_data_rpy_msg.magnetic_field.y = pitch;
                imu_data_rpy_msg.magnetic_field.z = yaw;
                #ifdef DEBUG_MODE
                //logger.info("RPY pkg");
                #endif
            }
        }
        break;
        case EP_CMD_GRAVITY_: {
            Ep_Gravity ep_Gravity;
            if (EP_SUCC_ == eOD.Read_Ep_Gravity(&ep_Gravity)) {
                // TODO 2026-03-02 (Will Free): why is this just not being used?
            }
        }
        break;
        case EP_CMD_COMBO_: {
            Ep_Combo ep_Combo;
            if (EP_SUCC_ == eOD.Read_Ep_Combo(&ep_Combo)) {
                // Accelerometer:
                float ax = ep_Combo.ax * 1e-5f * GRAVITY; // Unit: 1g, 1g = 9.794m/(s^2)
                float ay = ep_Combo.ay * 1e-5f * GRAVITY;
                float az = ep_Combo.az * 1e-5f * GRAVITY;

                // Gyroscope:
                float wx = ep_Combo.wx * 1e-5f; // Unit: rad/s
                float wy = ep_Combo.wy * 1e-5f;
                float wz = ep_Combo.wz * 1e-5f;

                // Magnetometer:
                float mx = ep_Combo.mx * 1e-3f * MAGNETIC_FACTORY_CALIBRATION; // Unit: one earth magnetic field
                float my = ep_Combo.my * 1e-3f * MAGNETIC_FACTORY_CALIBRATION; // vector (mx, my, mz) is used as direction reference of the local magnetic field
                float mz = ep_Combo.mz * 1e-3f * MAGNETIC_FACTORY_CALIBRATION;

                // Quaternion in (q1,q2,q3,q4)=(w,x,y,z) format
                float q1 = ep_Combo.q1 * 1e-7f;
                float q2 = ep_Combo.q2 * 1e-7f;
                float q3 = ep_Combo.q3 * 1e-7f;
                float q4 = ep_Combo.q4 * 1e-7f;
                // RPY:
                float roll = ep_Combo.roll * 1e-2f; // Unit: degree
                float pitch = ep_Combo.pitch * 1e-2f; // Unit: degree
                float yaw = ep_Combo.yaw * 1e-2f; // Unit: degree

                imu_data_msg.angular_velocity.x = wx;
                imu_data_msg.angular_velocity.y = wy;
                imu_data_msg.angular_velocity.z = wz;

                imu_data_msg.linear_acceleration.x = ax;
                imu_data_msg.linear_acceleration.y = ay;
                imu_data_msg.linear_acceleration.z = az;

                imu_data_msg.orientation.x = q1;
                imu_data_msg.orientation.y = q2;
                imu_data_msg.orientation.z = q3;
                imu_data_msg.orientation.w = q4;

                imu_data_rpy_msg.magnetic_field.x = roll;
                imu_data_rpy_msg.magnetic_field.y = pitch;
                imu_data_rpy_msg.magnetic_field.z = yaw;

                imu_data_mag_msg.magnetic_field.x = mx;
                imu_data_mag_msg.magnetic_field.y = my;
                imu_data_mag_msg.magnetic_field.z = mz;

                #ifdef DEBUG_MODE
                //logger.info("Combo pkg");
                #endif
            }
        }
        break;
        }
        #ifdef DEBUG_MODE
        count++;
        #endif
    } // while()
    return true;
}


void TMSerial::FillCovarianceMatrices() {
    for (int i = 0; i < 9; i++) {
        imu_data_msg.orientation_covariance[i] = 0.1;
        imu_data_msg.angular_velocity_covariance[i] = 0.1;
        imu_data_msg.linear_acceleration_covariance[i] = 0.1;
        imu_data_rpy_msg.magnetic_field_covariance[i] = 0.1;
        imu_data_mag_msg.magnetic_field_covariance[i] = 0.1;
    }
}


void TMSerial::PublishTransform() {
    geometry_msgs::msg::TransformStamped transform_;
    transform_.header.stamp = this->get_clock()->now();
    transform_.header.frame_id = "world";
    transform_.child_frame_id = this->get_parameter("imu_frame_id").as_string();
    transform_.transform.translation.x = this->get_parameter("transform").as_double_array()[0];
    transform_.transform.translation.y = this->get_parameter("transform").as_double_array()[1];
    transform_.transform.translation.z = this->get_parameter("transform").as_double_array()[2];

    transform_.transform.rotation.x = imu_data_msg.orientation.x;
    transform_.transform.rotation.y = imu_data_msg.orientation.y;
    transform_.transform.rotation.z = imu_data_msg.orientation.z;
    transform_.transform.rotation.w = imu_data_msg.orientation.w;
    tf_broadcaster_->sendTransform(transform_);
}


int main(const int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TMSerial>());
    rclcpp::shutdown();
    return 0;
}
