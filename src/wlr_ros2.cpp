// ROS2 node for Vanjee WLR-722Z 16 lines 3D LiDAR
// By TinLethax at Robot Club KMITL (RB26)

#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <thread>
#include <stdexcept>
#include <iomanip>
#include <sstream>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <asm/termbits.h>
#include <sys/ioctl.h> 

// ROS2 library
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/imu.hpp>

// PCL library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#define FSM_TIME_MIL	1000 // around 1Hz

#define SERIAL_BAUD		3125000 // 3.125MBaud RS485

#define POINTS_PER_PACKET	16		// 16 points per one RS485 packet
#define PACKETS_PER_SCAN	600		// 600 RS485 packets per one scan revolution
#define POINTS_PER_SCAN		(POINTS_PER_PACKET * PACKETS_PER_SCAN) // 9600 points per one scan
#define SCAN_RATE			5		// 5Hz (revolution/s) scan rate

#define PACKET_HEADER_LENGTH	6	// Header + Data type] 
#define PACKET_STAMP_LENGTH	10	// Date + Timestamp
#define PACKET_PCL_LENGTH	60	// Pointcloud data
#define PACKET_IMU_LENGTH	14	// IMU data
#define PACKET_CRC_LENGTH	4	// CRC32 

#define PACKET_PCL_FULL_LEN	(PACKET_HEADER_LENGTH + PACKET_STAMP_LENGTH + PACKET_PCL_LENGTH + PACKET_CRC_LENGTH)
#define PACKET_PCL_NOH_LEN	(PACKET_STAMP_LENGTH + PACKET_PCL_LENGTH + PACKET_CRC_LENGTH)
#define PACKET_PCL_DATA_LEN (PACKET_HEADER_LENGTH + PACKET_STAMP_LENGTH + PACKET_PCL_LENGTH)

#define PACKET_IMU_FULL_LEN	(PACKET_HEADER_LENGTH + PACKET_STAMP_LENGTH + PACKET_IMU_LENGTH + PACKET_CRC_LENGTH)
#define PACKET_IMU_NOH_LEN	(PACKET_STAMP_LENGTH + PACKET_IMU_LENGTH + PACKET_CRC_LENGTH)
#define PACKET_IMU_DATA_LEN	(PACKET_HEADER_LENGTH + PACKET_STAMP_LENGTH + PACKET_IMU_LENGTH)

// Math constant
#define DEG_TO_RADS_COSNT		0.0174533
#define DEG_TO_RADS(x)			(x * DEG_TO_RADS_COSNT)


struct VelodynePointXYZIR {
	PCL_ADD_POINT4D

	PCL_ADD_INTENSITY;
	uint16_t ring;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIR,
                                   (float, x, x)(float, y, y)
                                           (float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)
)

class wlr722z_if : public rclcpp::Node{
	
	public:
	
	// LiDAR publisher
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr 	pubCloudScan;
	// LiDAR message buffer
	sensor_msgs::msg::PointCloud2			msgCloud;
	// Pointcloud accumulator
	pcl::PointCloud<VelodynePointXYZIR>		accumuCloud;
	uint16_t u16PointPktCount = 0;
	
	// IMU publisher
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr				pubImu;
	// IMU message buffer
	sensor_msgs::msg::Imu					msgImu;
	
	// Used in wall timer callback
	rclcpp::TimerBase::SharedPtr	timerFsmLoop;
	
	// Serial port name '/dev/blablabla'
	std::string strSerialPort;
	struct termios2 tty;// tty instant
	int i32SerialFd;
	
	// Laser frame id
	std::string strLaserFrameId;
	
	// Scan topic name
	std::string strScanTopic;
	
	// IMU topic name
	std::string strImuTopic;
	
	// Tx buffer
	uint8_t u8TcpTxBuffer[50];
	// Rx buffer
	uint8_t u8SerialRxBuffer[128];
	uint8_t u8SerialBufL2[2048];
	uint8_t u8LaserBuffer[100];
	int i32RxByteCount;
	
	enum eDATA_TYPE{
		EDATA_PCL = 0,
		EDATA_IMU = 1,
	};
	
	typedef struct __attribute__((packed)){
		uint8_t 	u8Header[2];
		uint8_t 	u8Version[2];
		uint8_t 	res1;
		uint8_t 	u8DataType;
	}tLaserPkt_header;
	
	typedef struct __attribute__((packed)){
		uint8_t		u8Year;
		uint8_t 	u8Month;
		uint8_t 	u8Day;
		uint8_t		u8Hour;
		uint8_t 	u8Minute;
		uint8_t 	u8Second;
		uint32_t	u32Stamp;
	}tLaserPkt_timestamp;
	
	typedef struct __attribute__((packed)){
		uint16_t	u16Azimuth;
		
		struct __attribute__((packed)){
			uint16_t	u16Distance;
			uint8_t		u8Intensity;
		}sLaserData[16];
		
		union __attribute__((packed)){
			uint32_t u32AllChannel;
			struct{
				uint32_t	ch1		:2;
				uint32_t	ch2		:2;
				uint32_t	ch3		:2;
				uint32_t	ch4		:2;
				
				uint32_t	ch5		:2;
				uint32_t	ch6		:2;
				uint32_t	ch7		:2;
				uint32_t	ch8		:2;
				
				uint32_t	ch9		:2;
				uint32_t	ch10	:2;
				uint32_t	ch11	:2;
				uint32_t	ch12	:2;
				
				uint32_t	ch13	:2;
				uint32_t	ch14	:2;
				uint32_t	ch15	:2;
				uint32_t	ch16	:2;
			}bit;
		}sLaserDirtLevel;
		
		uint8_t u8LidarState;
		
		uint8_t u8InfoId;
		uint8_t	u8InfoData[2];
		
		uint16_t u16SequenceNum;
		
	}tLaserPkt_pointcloud;
	
	typedef struct __attribute__((packed)){
		int16_t		i16Accel_x;
		int16_t		i16Accel_y;
		int16_t 	i16Accel_z;
		int16_t 	i16Gyro_x;
		int16_t		i16Gyro_y;
		int16_t		i16Gyro_z;
		uint16_t	u16SequenceNum;
	}tLaserPkt_imu;
	
	typedef struct __attribute__((packed)){
		tLaserPkt_header		tHeader;
		tLaserPkt_timestamp		tStamp;
		tLaserPkt_pointcloud	tCloudData;
		uint32_t				u32CRC;
	}tLaserDataPointCloud;
	
	typedef struct __attribute__((packed)){
		tLaserPkt_header		tHeader;
		tLaserPkt_timestamp		tStamp;
		tLaserPkt_imu			tImuData;
		uint32_t				u32CRC;
	}tLaserDataImu;
	
	// constant
	const double f64VerticalAngle[16] = {
		DEG_TO_RADS(-0.267),		// CH1
		DEG_TO_RADS(2.545),		// CH2
		DEG_TO_RADS(5.136),		// CH3
		DEG_TO_RADS(7.852),		// CH4
		DEG_TO_RADS(10.542),		// CH5
		DEG_TO_RADS(13.25),		// CH6
		DEG_TO_RADS(15.946),		// CH7
		DEG_TO_RADS(18.65),		// CH8
		DEG_TO_RADS(21.35),		// CH9
		DEG_TO_RADS(24.049),		// CH10
		DEG_TO_RADS(26.755),		// CH11
		DEG_TO_RADS(29.448),		// CH12
		DEG_TO_RADS(32.161),		// CH13
		DEG_TO_RADS(34.846),		// CH14
		DEG_TO_RADS(37.566),		// CH15
		DEG_TO_RADS(40.244)		// CH16
	};
	
	const double f64HorizontalOffset[16] = {
		DEG_TO_RADS(-2.469),
		DEG_TO_RADS(1.499),
		DEG_TO_RADS(-2.562),
		DEG_TO_RADS(1.59),
		DEG_TO_RADS(-2.565),
		DEG_TO_RADS(1.677),
		DEG_TO_RADS(-2.64),
		DEG_TO_RADS(1.664),
		DEG_TO_RADS(-2.788),
		DEG_TO_RADS(1.629),
		DEG_TO_RADS(-3.055),
		DEG_TO_RADS(1.496),
		DEG_TO_RADS(-3.462),
		DEG_TO_RADS(1.32),
		DEG_TO_RADS(-3.805),
		DEG_TO_RADS(1.251)
	};
	
	// Pointer
	uint32_t *p32RangeIntensityPairPtr;
	
	// IMU zero motion calibration
	bool bIsGyroCalibrated = false;
	double f64GyroXAccumu = 0.0;
	double f64GyroYAccumu = 0.0;
	double f64GyroZAccumu = 0.0;
	uint16_t u16GyroCalSampleCnt = 0;
	
	// FSM
	uint8_t u8LaserFSM;

	
	wlr722z_if() : Node("wlr722z_node"){
		RCLCPP_INFO(
			this->get_logger(), 
			"Robot Club KMITL : Starting WLR-722Z LiDAR node..."
			);
		
		declare_parameter("serial_port", "/dev/ttyACM0");
		get_parameter("serial_port", strSerialPort);
		
		declare_parameter("laser_frame_id", "laser");
		get_parameter("laser_frame_id", strLaserFrameId);
		
		declare_parameter("cloud_topic", "cloud");
		get_parameter("cloud_topic", strScanTopic);
		
		declare_parameter("imu_topic", "imu");
		get_parameter("imu_topic", strImuTopic);
		
		// Open the serial port
		char *cSerialPortFile = new char[strSerialPort.length() + 1];
		strcpy(cSerialPortFile, strSerialPort.c_str());
		i32SerialFd = open(cSerialPortFile, O_RDWR);
		// Can't open serial port
		if(i32SerialFd < -1){
			RCLCPP_ERROR(
				this->get_logger(), 
				"Error openning Serial %s", 
				strSerialPort.c_str()
				);
			std::raise(SIGTERM);
			return;
		}
		
		if(ioctl(i32SerialFd, TCGETS2, &tty) != 0){
			RCLCPP_ERROR(
				this->get_logger(), 
				"Error %i from ioctl: %s\n", 
				errno, 
				strerror(errno)
				);
			std::raise(SIGTERM);
			return;			
		}
		
		tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
		tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
		tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
		tty.c_cflag |= CS8; // 8 bits per byte (most common)
		tty.c_cflag |= CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
		tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

		tty.c_lflag &= ~ICANON;
		tty.c_lflag &= ~ECHO; // Disable echo
		tty.c_lflag &= ~ECHOE; // Disable erasure
		tty.c_lflag &= ~ECHONL; // Disable new-line echo
		tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
		tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
		tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

		tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
		tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

		tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
		tty.c_cc[VMIN] = 0;
		
		tty.c_cflag &= ~(CBAUD | (CBAUD << IBSHIFT));
		tty.c_cflag |= (B3000000);

		if (ioctl(i32SerialFd, TCSETS2, &tty) != 0) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Error %i from ioctl: %s\n", 
				errno, 
				strerror(errno));
			std::raise(SIGTERM);
			return;
		}

		// We will publish pointcloud on every one revolution of scan 
		// 16 points per packet
		// 600 packets per scan revolution -> 9600 points/scan, 600 serial pkts/scan.
		// 5 scan revolution per second (5Hz)
		// ROS Message rate -> 5 = 5 msgs/sec
		// RS-485 serial packet rate -> 600 * 5 = 3000pkts/sec
		// Points rate -> 16 * 600 * 5 = 48000 points/sec

		// Setup basics data in the laser message
		// Publish every one scan revolution
		msgCloud.header.frame_id 	= strLaserFrameId + "_cloud";
		msgCloud.height				= 1;// Each scan message is a single slice
		msgCloud.width				= POINTS_PER_SCAN;// Each scan contains 9600 points
		
		sensor_msgs::PointCloud2Modifier pcl2Modifier(msgCloud);
		
		pcl2Modifier.setPointCloud2Fields(
			5,
			"x", 1, sensor_msgs::msg::PointField::FLOAT32,
			"y", 1, sensor_msgs::msg::PointField::FLOAT32,
			"z", 1, sensor_msgs::msg::PointField::FLOAT32,
			"intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
			"ring", 1, sensor_msgs::msg::PointField::UINT32
		);
		
		msgCloud.is_bigendian 		= false;
		msgCloud.point_step			= 20;// Four float32 and on uint16
		msgCloud.row_step			= msgCloud.width * msgCloud.point_step;
		
		msgCloud.is_dense			= false;

		// Setup IMU message
		msgImu.header.frame_id		= strLaserFrameId + "_imu";
		// Orientation
		msgImu.orientation.x		= 0.0;
		msgImu.orientation.y		= 0.0;
		msgImu.orientation.z		= 0.0;
		msgImu.orientation.w		= 1.0;
		
		// Covariance matrices are calculated from the noise density with Output data rate of 200Hz
		// Sensor : ICM-42668-P  
		
		/* Orientation covariance */
		msgImu.orientation_covariance = {
		// 	 x		y		z
			-1.0,	0.0,	0.0, // x
			0.0,	-1.0,	0.0, // y
			0.0,	0.0,	-1.0  // z
		};
		
		/* Angular velocity covaraince */
		msgImu.angular_velocity_covariance = {
		//	gx		gy		gz
			4.78e-7,0.0,	0.0, 	// gx
			0.0,	4.78e-7,0.0, 	// gy
			0.0,	0.0,	4.78e-7 // gz
		};
		
		/* Linear Acceleration covariance */
		msgImu.linear_acceleration_covariance = {
		//	ax		ay		az
			9.42e-5,0.0,	0.0,	// ax
			0.0,	9.42e-5,0.0,	// ay
			0.0,	0.0,	9.42e-5 // az
		};
		
		// Laser Publisher 
		pubCloudScan =
			create_publisher<sensor_msgs::msg::PointCloud2>(
				strScanTopic,
				rclcpp::QoS(rclcpp::SensorDataQoS())
			);
		
		// IMU Publisher	
		pubImu	=
			create_publisher<sensor_msgs::msg::Imu>(
				strImuTopic,
				rclcpp::QoS(rclcpp::SensorDataQoS())
			);
			
		// Serial listener thread
		std::thread tRunner(
			&wlr722z_if::wlr722z_serialRunner, 
			this
		);
		tRunner.detach();	
		
	}
	
	bool wlr722z_checkCRC(uint8_t *u8Data, uint8_t u8Length, uint32_t u32MsgCrc){
		static const uint32_t u32CrcTable[0x100] = {
		0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61,
		0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD, 0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
		0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD, 0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039,
		0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5, 0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
		0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95, 0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1,
		0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D, 0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
		0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA, 0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE,
		0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02, 0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
		0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692, 0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6,
		0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A, 0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
		0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A, 0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637,
		0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB, 0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
		0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B, 0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF,
		0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623, 0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
		0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3, 0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7,
		0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B, 0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
		0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C, 0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8,
		0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24, 0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
		0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654, 0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0,
		0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C, 0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
		0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C, 0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668,
		0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4};
		
		uint32_t checksum = 0xFFFFFFFF;
		int i = 0;
		
		for (; i < u8Length; i++) {
			uint8_t top = (uint8_t)(checksum >> 24);
			top ^= u8Data[i];
			checksum = (checksum << 8) ^ u32CrcTable[top];
		}

		while (i % 4 > 0) {
			uint8_t top = (uint8_t)(checksum >> 24);
			checksum = (checksum << 8) ^ u32CrcTable[top];
			i++;
		}
		return (checksum == u32MsgCrc);
	}
	
	uint8_t wlr722z_getNextMatch(uint8_t u8LastMatch){
	
		if(u8LastMatch == 0xEE){
			return 0xFF;
		}else if(u8LastMatch == 0xFF){
			return 0x01;
		}else if(u8LastMatch == 0x01){
			return 0x01;
		}
		
		return 0x00;
	}
	
	// Serial receiver
	void wlr722z_serialRunner(){
		uint16_t 	u16LaserDataOffset 		= 0;
		uint16_t 	u16LaserDataAccumu		= 0;
		uint16_t 	u16LaserCopyOffset		= 0;
		uint16_t 	u16LaserTotalDataLength	= 0;
		uint16_t	u16LaserHeaderNextCyclOffset  = 0;
		
		uint16_t 	u16SearchStartPos = 0;
		uint8_t 	u8MatchCount = 0;
		uint8_t 	u8LastMatchByte = 0x00;
		bool 		bFullHeaderMatch = false;
		
		bool 		bHasDataType = false;
		bool 		bNeedMoreData = false;
		
		int ret;
		
		RCLCPP_INFO(
			this->get_logger(),
			"Starting RS-485 thread"
		);
		
		RCLCPP_INFO(
			this->get_logger(),
			"Please stay stil for 5 seconds for IMU offset calibration"
		);
		
		// Sync with header
		while(1){	
		lWait:		
		
			// Clear buffer before receive again
			memset(u8SerialRxBuffer, 0, 2048);

			ret = read(
				i32SerialFd, 
				u8SerialRxBuffer, 128);
				
			if(ret < 0){
				// Non-EAGAIN error will throw an error
				if(errno != EAGAIN){
					RCLCPP_ERROR(
						this->get_logger(),
						"Error reading header with code %d",
						errno
					);
				}
				goto lWait;
			}
			
			RCLCPP_DEBUG(
				this->get_logger(),
				"Received %d bytes",
				ret
			);

			lSearch:
			if(bFullHeaderMatch == false){// Only search when no match is previously found
				RCLCPP_DEBUG(
					this->get_logger(),
					"Search begin at %d",
					u16SearchStartPos
				);
				for(; u16SearchStartPos < ret; u16SearchStartPos++){
					if(u8MatchCount == 0){
						// If we don't have any match yet
						// Start looking for 0xEE
						if(u8SerialRxBuffer[u16SearchStartPos] == 0xEE){
							u8LaserBuffer[0] = 0xEE;
							u8MatchCount++;
							u8LastMatchByte = 0xEE;
						}else{
							continue;
						}
					}else{
						if(u8LaserBuffer[u8MatchCount] = wlr722z_getNextMatch(u8LastMatchByte), 
							u8SerialRxBuffer[u16SearchStartPos] == u8LaserBuffer[u8MatchCount]){
							u8MatchCount++;
							if(u8MatchCount == 4){
								u8MatchCount = 0;
								bFullHeaderMatch = true;
								
								RCLCPP_DEBUG(
									this->get_logger(),
									"Header found at %d",
									u16SearchStartPos
								);
								
								goto lParse;
							}else{
								u8LastMatchByte = wlr722z_getNextMatch(u8LastMatchByte);
								continue;
							}
						}else{
							u8MatchCount = 0;
							continue;
						}
						
					}
				}	
				
				// If the search ended before we found the header, restart the search again
				if(u8MatchCount < 1){
					RCLCPP_DEBUG(
						this->get_logger(),
						"No or no more header found in this cycle, find it on the next cycle"
					);
					
					u8MatchCount = 0;
					u8LastMatchByte = 0x00;
					bFullHeaderMatch = false;
				}else{// Else, we found some partial match. 
					// Continue the search on next read cycle
					RCLCPP_DEBUG(
						this->get_logger(),
						"Found partial match of %d bytes at %d",
						u8MatchCount, u16SearchStartPos - u8MatchCount
					);
				}
				
				u16SearchStartPos = 0;
				goto lWait;
			}
			
		lParse: // Parse here 
			if(bNeedMoreData == false){
				if(bFullHeaderMatch == true){
					// Check if the data type is near or beyond the end of the input buffer
					// By checking from the last byte of header to the end of input buffer
					// and see if it actually has a room for data type
					if((ret - u16SearchStartPos) > 2){
						((tLaserPkt_header *)&u8LaserBuffer[0])->res1 = 0x00;
						((tLaserPkt_header *)&u8LaserBuffer[0])->u8DataType = u8SerialRxBuffer[u16SearchStartPos + 2];
						u16LaserDataOffset = u16SearchStartPos + 3;
						bHasDataType = true;
					}else{
						u16LaserHeaderNextCyclOffset = 
							(u16SearchStartPos + 2) - ret;
					
						RCLCPP_DEBUG(
							this->get_logger(),
							"Data type will arrive on next read cycle with offset of %d!",
							u16LaserHeaderNextCyclOffset
						);
						bNeedMoreData	= true;
						bHasDataType	= false;
						goto lWait;
						
					}
					
					// Set the expected data length according to the data type
					if(((tLaserPkt_header *)&u8LaserBuffer[0])->u8DataType == EDATA_PCL){
						u16LaserTotalDataLength = PACKET_PCL_NOH_LEN;
						RCLCPP_DEBUG(
							this->get_logger(),
							"Data type : Points"
						);
					}else if(((tLaserPkt_header *)&u8LaserBuffer[0])->u8DataType == EDATA_IMU){
						u16LaserTotalDataLength = PACKET_IMU_NOH_LEN;
						RCLCPP_DEBUG(
							this->get_logger(),
							"Data type : IMU"
						);
					}else{
						u16LaserTotalDataLength = 0;
						
						RCLCPP_DEBUG(
							this->get_logger(),
							"Error unknow data type 0x%02X at %d, Skipping this pkt!",
							((tLaserPkt_header *)&u8LaserBuffer[0])->u8DataType, u16SearchStartPos + 2
						);
						
						std::ostringstream oss;
						oss << "RX [" << ret << " bytes]: "
							<< std::hex << std::setfill('0');

						for (size_t i = 0; i < ret; ++i)
						{
							oss << std::setw(2)
								<< static_cast<unsigned int>(u8SerialRxBuffer[i])
								<< " ";
						}

						RCLCPP_DEBUG_STREAM(this->get_logger(), oss.str());
						
						bFullHeaderMatch = false;
						u16SearchStartPos = 0;

						goto lWait;
					}
					
					// Extract the actual copy-able data length 
					if((u16LaserDataOffset + u16LaserTotalDataLength) <= ret){
						// The data is not exceed the buffer legth
						// can process now
						u16LaserDataAccumu		= u16LaserTotalDataLength;
						bFullHeaderMatch		= false;
						bNeedMoreData			= false;
						
						RCLCPP_DEBUG(
							this->get_logger(),
							"Process now!"
						);
						
						memcpy(
							&u8LaserBuffer[PACKET_HEADER_LENGTH],
							&u8SerialRxBuffer[u16LaserDataOffset],
							u16LaserDataAccumu
						);
						
						
						switch(((tLaserPkt_header *)&u8LaserBuffer[0])->u8DataType){
							case EDATA_PCL:
							{
								if(!wlr722z_checkCRC(u8LaserBuffer, PACKET_PCL_DATA_LEN, ((tLaserDataPointCloud *)&u8LaserBuffer[0])->u32CRC)){
									RCLCPP_DEBUG(
										this->get_logger(),
										"Pointcloud CRC incorrect!"
									);
								}else
									wlr722z_publishCloudMsg();	
								
							}
							break;
							
							case EDATA_IMU:
							{
								if(!wlr722z_checkCRC(u8LaserBuffer, PACKET_IMU_DATA_LEN, ((tLaserDataImu *)&u8LaserBuffer[0])->u32CRC)){
									RCLCPP_DEBUG(
										this->get_logger(),
										"IMU CRC incorrect!"
									);
								}else
									wlr722z_publishImuMsg();	
								
							}
							break;
						}

						// Continue to search if we still have some data in the input buffer
						u16SearchStartPos += u16LaserTotalDataLength - 1;
						goto lSearch;
						
					}else{
						// The data is exceed the buffer length
						// copy what we have and copy more on next read cycle
						u16LaserDataAccumu = 
							ret - u16LaserDataOffset;
							
						RCLCPP_DEBUG(
							this->get_logger(),
							"Process Later! Cpy\'d %d bytes, Remain %d bytes",
							u16LaserDataAccumu, (u16LaserTotalDataLength - u16LaserDataAccumu)
						);
						
						if(u16LaserDataAccumu != 0){
							memcpy(
								&u8LaserBuffer[PACKET_HEADER_LENGTH],
								&u8SerialRxBuffer[u16LaserDataOffset],
								u16LaserDataAccumu
							);	
						}
						
						u16LaserCopyOffset += u16LaserDataAccumu;
							
						bNeedMoreData = true;
						goto lWait;
					}
					
				}
			}else{
				RCLCPP_DEBUG(
					this->get_logger(),
					"Need more data!"
				);
				// bNeedMoreData is true, continue copy the data and publish it
				// But If we don't have data type on the previous cycle
				// Have it in this cycle and parse it before continue the data processing
				
				if(bHasDataType == false){
					u16LaserDataAccumu = 0;
					
					if(u16LaserHeaderNextCyclOffset < ret){
						((tLaserPkt_header *)&u8LaserBuffer[0])->res1 = 0x00;
						((tLaserPkt_header *)&u8LaserBuffer[0])->u8DataType = u8SerialRxBuffer[u16LaserHeaderNextCyclOffset];
						u16LaserDataOffset = u16LaserHeaderNextCyclOffset + 1;
					}else{
						RCLCPP_DEBUG(
							this->get_logger(),
							"Error getting the data type with offset of %d",
							u16LaserHeaderNextCyclOffset
						);
						bNeedMoreData = false;
						bHasDataType  = false;
						u16LaserDataOffset = 0;
						goto lWait;
					}
					
					u16LaserHeaderNextCyclOffset = 0;
					
					// Set the expected data length according to the data type
					bHasDataType = true;
					if(((tLaserPkt_header *)&u8LaserBuffer[0])->u8DataType == EDATA_PCL){
						u16LaserTotalDataLength = PACKET_PCL_NOH_LEN;
						RCLCPP_DEBUG(
							this->get_logger(),
							"Data type : Points"
						);
					}else if(((tLaserPkt_header *)&u8LaserBuffer[0])->u8DataType == EDATA_IMU){
						u16LaserTotalDataLength = PACKET_IMU_NOH_LEN;
						RCLCPP_DEBUG(
							this->get_logger(),
							"Data type : IMU"
						);
					}else{
						u16LaserTotalDataLength = 0;
						u16LaserDataOffset = 0;
						
						RCLCPP_DEBUG(
							this->get_logger(),
							"Error unknow data type 0x%02X at %d, Skipping this pkt!",
							((tLaserPkt_header *)&u8LaserBuffer[0])->u8DataType, u16LaserHeaderNextCyclOffset
						);

						goto lWait;
					}
					
					u16LaserDataAccumu = u16LaserTotalDataLength;
					
				}else{
					// If we already have the data type then can continue to copy
					u16LaserDataOffset = 0;
				}
				
				if(u16LaserDataAccumu <= u16LaserTotalDataLength){
					// Check if on this receive cycle, we have enough data to complete a packet
					if((u16LaserTotalDataLength - u16LaserDataAccumu) <= ret){
						// If yes the copy the rest of the packet data and process it
						bNeedMoreData = false;
						bFullHeaderMatch = false;
						
						memcpy(
							&u8LaserBuffer[PACKET_HEADER_LENGTH + u16LaserDataAccumu],
							&u8SerialRxBuffer[u16LaserDataOffset],
							u16LaserTotalDataLength - u16LaserDataAccumu
						);
						
						switch(((tLaserPkt_header *)&u8LaserBuffer[0])->u8DataType){
							case EDATA_PCL:
							{
								if(!wlr722z_checkCRC(u8LaserBuffer, PACKET_PCL_DATA_LEN, ((tLaserDataPointCloud *)&u8LaserBuffer[0])->u32CRC)){
									RCLCPP_DEBUG(
										this->get_logger(),
										"Pointcloud CRC incorrect!"
									);
								}else
									wlr722z_publishCloudMsg();	
								
							}
							break;
							
							case EDATA_IMU:
							{
								if(!wlr722z_checkCRC(u8LaserBuffer, PACKET_IMU_DATA_LEN, ((tLaserDataImu *)&u8LaserBuffer[0])->u32CRC)){
									RCLCPP_DEBUG(
										this->get_logger(),
										"IMU CRC incorrect!"
									);
								}else
									wlr722z_publishImuMsg();	
								
							}
							break;
						}
						
						// Continue to search if we still have some data in the input buffer
						u16SearchStartPos = u16LaserDataOffset + (u16LaserTotalDataLength - u16LaserDataAccumu) - 1;
						u16LaserDataAccumu	 	= 0;
						goto lSearch;
						
					}else{
						// In the case of not enought data, we still need it more to complete the packet
						bNeedMoreData = true;
						
						memcpy(
							&u8LaserBuffer[PACKET_HEADER_LENGTH + u16LaserDataAccumu],
							&u8SerialRxBuffer[u16LaserDataOffset],
							ret
						);
						
						u16LaserDataAccumu += ret;
						goto lWait;
					}
				}	
				
			}// if(bNeedMoreData)
			
		
		}// while(1)
	
	}

	void wlr722z_publishImuMsg(){
		RCLCPP_DEBUG(
			this->get_logger(),
			"Received IMU packet!"
		);
	
		msgImu.linear_acceleration.x = 
			((double)(((tLaserDataImu *)&u8LaserBuffer[0])->tImuData.i16Accel_x) / 8192.0) * 9.81;
		msgImu.linear_acceleration.y = 
			((double)(((tLaserDataImu *)&u8LaserBuffer[0])->tImuData.i16Accel_y) / 8192.0) * 9.81;
		msgImu.linear_acceleration.z = 
			((double)(((tLaserDataImu *)&u8LaserBuffer[0])->tImuData.i16Accel_z) / 8192.0) * 9.81;
		
		msgImu.angular_velocity.x = 
			((double)(((tLaserDataImu *)&u8LaserBuffer[0])->tImuData.i16Gyro_x) / 32.8) * DEG_TO_RADS_COSNT;
		msgImu.angular_velocity.y = 
			((double)(((tLaserDataImu *)&u8LaserBuffer[0])->tImuData.i16Gyro_y) / 32.8) * DEG_TO_RADS_COSNT;
		msgImu.angular_velocity.z = 
			((double)(((tLaserDataImu *)&u8LaserBuffer[0])->tImuData.i16Gyro_z) / 32.8) * DEG_TO_RADS_COSNT;
				
		if(bIsGyroCalibrated == false){
			f64GyroXAccumu += msgImu.angular_velocity.x;
			f64GyroYAccumu += msgImu.angular_velocity.y;
			f64GyroZAccumu += msgImu.angular_velocity.z;
			u16GyroCalSampleCnt++;
			
			if(u16GyroCalSampleCnt == 1000){
				u16GyroCalSampleCnt = 0;
				bIsGyroCalibrated = true;
				f64GyroXAccumu = f64GyroXAccumu/1000.0;
				f64GyroYAccumu = f64GyroYAccumu/1000.0;
				f64GyroZAccumu = f64GyroZAccumu/1000.0;
				RCLCPP_INFO(
					this->get_logger(),
					"IMU calibration done!"
				);
			}
		}else{	
			msgImu.angular_velocity.x -= f64GyroXAccumu;
			msgImu.angular_velocity.y -= f64GyroYAccumu;
			msgImu.angular_velocity.z -= f64GyroZAccumu;
			
			msgImu.header.stamp = this->get_clock()->now();
			pubImu->publish(msgImu);
		}
	}

	void wlr722z_publishCloudMsg(){
		float f32Range;
		double f64Azimuth;
		uint8_t u8Ring = 0;
		
		RCLCPP_DEBUG(
			this->get_logger(),
			"Received PCL packet!"
		);
		
		f64Azimuth = ((double)(((tLaserDataPointCloud *)&u8LaserBuffer[0])->tCloudData.u16Azimuth) * 0.01) * DEG_TO_RADS_COSNT;// 0.01 degree/LSB

		sensor_msgs::PointCloud2Iterator<float>	iterPCL(msgCloud, "x");

		for(u8Ring = 0; u8Ring < 16; u8Ring++){
			f32Range = ((double)(((tLaserDataPointCloud *)&u8LaserBuffer[0])->tCloudData.sLaserData[u8Ring].u16Distance) * 0.002);
			if(f32Range < 0.2){// Skipping NaN point
				continue;
			}	
			
			VelodynePointXYZIR pointAdd;
			
			pointAdd.x = -f32Range * cos(f64Azimuth + f64HorizontalOffset[u8Ring]) * cos(f64VerticalAngle[u8Ring]);// X
			pointAdd.y = f32Range * sin(f64Azimuth + f64HorizontalOffset[u8Ring]) * cos(f64VerticalAngle[u8Ring]);// Y
			pointAdd.z = f32Range * sin(f64VerticalAngle[u8Ring]);// Z
			pointAdd.intensity = (double)(((tLaserDataPointCloud *)&u8LaserBuffer[0])->tCloudData.sLaserData[u8Ring].u8Intensity) / 255.0;
			pointAdd.ring = u8Ring;
			
			accumuCloud.points.push_back(pointAdd);
		}
		
		u16PointPktCount ++;

		if(u16PointPktCount >= 600){
			u16PointPktCount = 0;
			pcl::toROSMsg(accumuCloud, msgCloud);
			msgCloud.header.stamp = this->get_clock()->now();
			msgCloud.header.frame_id 	= strLaserFrameId + "_cloud";
			pubCloudScan->publish(msgCloud);
			accumuCloud.clear();
		}
		
	}
	
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto vanjee_if {std::make_shared<wlr722z_if>()};
	rclcpp::spin(vanjee_if);
	
	rclcpp::shutdown();
}
