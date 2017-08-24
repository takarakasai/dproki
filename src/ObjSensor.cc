#include "dproki/ObjSensor.h"

#include <iostream>
#include <fstream>
#include <memory>

#include "dpcommon/dp_type.h"
#include "dpcommon/dp_err.h"

namespace Dp {

  namespace obj {
  
    const std::string kSensorCameraName  = "Camera";
    const std::string kSensorUnknownName = "Unknown";
  
    const std::string & SensorType2Str (Dp::SENSOR_TYPE type) {
      switch (type) {
        case Dp::kSensorCamera : return kSensorCameraName;
        default:                 return kSensorUnknownName;
      }
    }
  
    Dp::SENSOR_TYPE Str2SensorType (const std::string &str) {
      if (str == "Camera") { return Dp::kSensorCamera;}
      else                 { return Dp::kSensorUnknown;}
    }
  
    std::shared_ptr<Dp::Sensor> CreateSensor (Dp::SENSOR_TYPE type) {
      switch (type) {
          case Dp::kSensorCamera : return std::make_shared<obj::CameraSensor>();
        default:                 return nullptr;
      }
    }
  
    static errno_t parseAttribute (Dp::Sensor &sens, std::ifstream &ifs) {
   
      ifs >> sens.GetOffsetXyz()(0);
      ifs >> sens.GetOffsetXyz()(1);
      ifs >> sens.GetOffsetXyz()(2);
      ifs >> sens.GetOffsetRpy()(0);
      ifs >> sens.GetOffsetRpy()(1);
      ifs >> sens.GetOffsetRpy()(2);
    
      return 0;
    }
  
    static errno_t composeAttribute (Dp::Sensor &sens, std::ofstream &ofs) {
    
      ofs << "    ";
      ofs << sens.GetOffsetXyz()(0) << " ";
      ofs << sens.GetOffsetXyz()(1) << " ";
      ofs << sens.GetOffsetXyz()(2) << " ";
      ofs << sens.GetOffsetRpy()(0) << " ";
      ofs << sens.GetOffsetRpy()(1) << " ";
      ofs << sens.GetOffsetRpy()(2) << std::endl;
     
      return 0;
    }
   
    /* Camera */
    errno_t CameraSensor::parse (std::ifstream &ifs) {
      std::cout << "***: Sensor Camera\n";
    
      ifs >> width_;
      ifs >> height_;
      ifs >> pitch_;
      ifs >> yaw_;
      ifs >> near_;
      ifs >> far_;
    
      ECALL(parseAttribute(*this, ifs));
    
      return 0;
    }
    
    errno_t CameraSensor::compose (std::ofstream &ofs) {
    
      ofs << "    ";
      ofs << width_  << " ";
      ofs << height_ << " ";
      ofs << pitch_  << " ";
      ofs << yaw_    << " ";
      ofs << near_   << " ";
      ofs << far_    << std::endl;
    
      ECALL(composeAttribute(*this, ofs));
    
      return 0;
    }
  }

}

