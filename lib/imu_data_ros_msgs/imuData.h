#ifndef _ROS_imu_data_ros_msgs_imuData_h
#define _ROS_imu_data_ros_msgs_imuData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

namespace imu_data_ros_msgs
{

  class imuData : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _gyro_type;
      _gyro_type gyro;
      typedef geometry_msgs::Vector3 _acc_type;
      _acc_type acc;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type orientation;

    imuData():
      gyro(),
      acc(),
      orientation()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->gyro.serialize(outbuffer + offset);
      offset += this->acc.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->gyro.deserialize(inbuffer + offset);
      offset += this->acc.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "imu_data_ros_msgs/imuData"; };
    virtual const char * getMD5() override { return "5ad595673ffd7016b3a6ed13a2cd7c74"; };

  };

}
#endif
