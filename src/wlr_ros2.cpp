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

#define FSM_TIME_MIL	1000 // around 1Hz

#define SERIAL_BAUD		3125000 // 3.125MBaud RS485

#define POINTS_PER_PACKET	16		// 16 points per one RS485 packet
#define PACKETS_PER_SCAN	1		// 600 RS485 packets per one scan revolution
#define POINTS_PER_SCAN		(POINTS_PER_PACKET * PACKETS_PER_SCAN) // 9600 points per one scan
#define SCAN_RATE			5		// 5Hz (revolution/s) scan rate

#define PACKET_HEADER_LENGTH	6	// Header + Data type] 
#define PACKET_STAMP_LENGTH	10	// Date + Timestamp
#define PACKET_PCL_LENGTH	60	// Pointcloud data
#define PACKET_IMU_LENGTH	14	// IMU data
#define PACKET_CRC_LENGTH	4	// CRC32 

#define PACKET_PCL_FULL_LEN	(PACKET_STAMP_LENGTH + PACKET_PCL_LENGTH + PACKET_CRC_LENGTH)
#define PACKET_IMU_FULL_LEN	(PACKET_STAMP_LENGTH + PACKET_IMU_LENGTH + PACKET_CRC_LENGTH)

// Math constant
#define DEG_TO_RADS_COSNT		0.0174533
#define DEG_TO_RADS(x)			(x * DEG_TO_RADS_COSNT)

class wlr722z_if : public rclcpp::Node{
	
	public:
	
	// LiDAR publisher
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr 	pubCloudScan;
	// LiDAR message buffer
	sensor_msgs::msg::PointCloud2			msgCloud;
	
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
	}tLasetPkt_header;
	
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
		//tLasetPkt_header		tHeader;
		tLaserPkt_timestamp		tStamp;
		tLaserPkt_pointcloud	tCloudData;
		uint32_t				u32CRC;
	}tLaserDataPointCloud;
	
	typedef struct __attribute__((packed)){
		//tLasetPkt_header		tHeader;
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
		
		msgCloud.is_dense			= true;
		
		
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
		uint8_t 	u8DataType = 0;
		
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
							u8MatchCount++;
							u8LastMatchByte = 0xEE;
						}else{
							continue;
						}
					}else{
						if(u8SerialRxBuffer[u16SearchStartPos] == wlr722z_getNextMatch(u8LastMatchByte)){
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
						u8DataType = u8SerialRxBuffer[u16SearchStartPos + 2];
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
					if(u8DataType == EDATA_PCL){
						u16LaserTotalDataLength = PACKET_PCL_FULL_LEN;
						RCLCPP_DEBUG(
							this->get_logger(),
							"Data type : Points"
						);
					}else if(u8DataType == EDATA_IMU){
						u16LaserTotalDataLength = PACKET_IMU_FULL_LEN;
						RCLCPP_DEBUG(
							this->get_logger(),
							"Data type : IMU"
						);
					}else{
						u16LaserTotalDataLength = 0;
						
						RCLCPP_ERROR(
							this->get_logger(),
							"Error unknow data type 0x%02X, Skipping this pkt!",
							u8DataType
						);
						
						bFullHeaderMatch = false;
						u16SearchStartPos = 0;
						
						// for(uint16_t j=0; j<2; j++){
							// RCLCPP_INFO(
								// this->get_logger(),
								// "0x%02X 0x%02X 0x%02X 0x%02X",
								// u8SerialRxBuffer[i+0+j*4], 
								// u8SerialRxBuffer[i+1+j*4], 
								// u8SerialRxBuffer[i+2+j*4], 
								// u8SerialRxBuffer[i+3+j*4]
							// );
						// }
						//continue;
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
							&u8LaserBuffer[0],
							&u8SerialRxBuffer[u16LaserDataOffset],
							u16LaserDataAccumu
						);
						
						if(u8DataType == EDATA_PCL){
							wlr722z_publishCloudMsg();						
						}else if(u8DataType == EDATA_IMU){
							wlr722z_publishImuMsg();
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
								&u8LaserBuffer[0],
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
						u8DataType = u8SerialRxBuffer[u16LaserHeaderNextCyclOffset];
						u16LaserDataOffset = u16LaserHeaderNextCyclOffset + 1;
					}else{
						RCLCPP_ERROR(
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
					if(u8DataType == EDATA_PCL){
						u16LaserTotalDataLength = PACKET_PCL_FULL_LEN;
						RCLCPP_DEBUG(
							this->get_logger(),
							"Data type : Points"
						);
					}else if(u8DataType == EDATA_IMU){
						u16LaserTotalDataLength = PACKET_IMU_FULL_LEN;
						RCLCPP_DEBUG(
							this->get_logger(),
							"Data type : IMU"
						);
					}else{
						u16LaserTotalDataLength = 0;
						u16LaserDataOffset = 0;
						
						RCLCPP_ERROR(
							this->get_logger(),
							"Error unknow data type 0x%02X, Skipping this pkt!",
							u8DataType
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
							&u8LaserBuffer[u16LaserDataAccumu],
							&u8SerialRxBuffer[u16LaserDataOffset],
							u16LaserTotalDataLength - u16LaserDataAccumu
						);
						
						if(u8DataType == EDATA_PCL){
							wlr722z_publishCloudMsg();				
						}else if(u8DataType == EDATA_IMU){
							wlr722z_publishImuMsg();
						}
						
						// Continue to search if we still have some data in the input buffer
						u16SearchStartPos = u16LaserDataOffset + (u16LaserTotalDataLength - u16LaserDataAccumu) - 1;
						u16LaserDataAccumu	 	= 0;
						goto lSearch;
						
					}else{
						// In the case of not enought data, we still need it more to complete the packet
						bNeedMoreData = true;
						
						memcpy(
							&u8LaserBuffer[u16LaserDataAccumu],
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

		for(; iterPCL != iterPCL.end(); ++iterPCL){			
			f32Range = ((double)(((tLaserDataPointCloud *)&u8LaserBuffer[0])->tCloudData.sLaserData[u8Ring].u16Distance) * 0.002);
					
			iterPCL[0] = -f32Range * cos(f64Azimuth + f64HorizontalOffset[u8Ring]) * cos(f64VerticalAngle[u8Ring]);// X
			iterPCL[1] = f32Range * sin(f64Azimuth + f64HorizontalOffset[u8Ring]) * cos(f64VerticalAngle[u8Ring]);// Y
			iterPCL[2] = f32Range * sin(f64VerticalAngle[u8Ring]);// Z
			iterPCL[3] = (double)(((tLaserDataPointCloud *)&u8LaserBuffer[0])->tCloudData.sLaserData[u8Ring].u8Intensity) / 255.0;
			iterPCL[4] = u8Ring;

			u8Ring++;
		}


		msgCloud.header.stamp = this->get_clock()->now();
		pubCloudScan->publish(msgCloud);
	}
	
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto vanjee_if {std::make_shared<wlr722z_if>()};
	rclcpp::spin(vanjee_if);
	
	rclcpp::shutdown();
}
