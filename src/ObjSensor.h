
#ifndef OBJ_SENSOR_H
#define OBJ_SENSOR_H

#include <memory>

#include "dp_type.h"

#include "Sensor.h"

namespace obj {

  extern const std::string kSensorCameraName;
  extern const std::string kSensorUnknownName;

  const std::string & SensorType2Str (Dp::SENSOR_TYPE type);
  Dp::SENSOR_TYPE Str2SensorType (const std::string &str);
  std::shared_ptr<Dp::Sensor> CreateSensor (Dp::SENSOR_TYPE type);

  /* format */
  /* Sensor Camera Name <width height hang vang near far> x y z r p y */
  class CameraSensor : public Dp::CameraSensor {
  private: 
  
  protected:

  public:
    CameraSensor() :
      Dp::CameraSensor() {};
    ~CameraSensor() {};

    errno_t parse (std::ifstream &ifs);
    errno_t compose (std::ofstream &ofs);
  };

}

#endif

