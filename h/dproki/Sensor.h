
#ifndef SENSOR_H
#define SENSOR_H

#include "dpcommon/dp_type.h"

#include <memory>

namespace Dp {

  typedef enum {
    kSensorCamera,
    kSensorUnknown,
  } SENSOR_TYPE;

  /* Sensor <SensorType> <sensor specific * x> x y z r p y */
  class Sensor { 
  private:
    SENSOR_TYPE type_;

    std::string name_;

    Eigen::Vector3d xyz_;   /* offset */
    Eigen::Vector3d rpy_;   /* offset [rad] */
   
  public:
    Sensor(SENSOR_TYPE type) : type_(type), name_(""), xyz_(0.,0.,0.), rpy_(0.,0.,0.) {};
    ~Sensor() {};
  
    SENSOR_TYPE Type() { return type_;};

    void SetName (std::string &name) { name_ = name;}
    std::string &GetName() { return name_;}

    void SetOffsetXyz(Eigen::Vector3d xyz) { xyz_   = xyz;}
    Eigen::Vector3d& GetOffsetXyz()        { return xyz_;}
    void SetOffsetRpy(Eigen::Vector3d rpy) { rpy_   = rpy;}
    Eigen::Vector3d& GetOffsetRpy()        { return rpy_;}

    virtual errno_t parse (std::ifstream &ifs) = 0;
    virtual errno_t compose (std::ofstream &ofs) = 0;;
    //virtual errno_t parse (std::ifstream &ifs) {
    //
    //  ifs >> xyz_(0);
    //  ifs >> xyz_(1);
    //  ifs >> xyz_(2);
    //  ifs >> rpy_(0);
    //  ifs >> rpy_(1);
    //  ifs >> rpy_(2);
    //
    //  return EOK;
    //}
    //virtual errno_t compose (std::ofstream &ofs) {
    //
    //  ifs << "    ";
    //  ifs << xyz_(0) << " ";
    //  ifs << xyz_(1) << " ";
    //  ifs << xyz_(2) << " ";
    //  ifs << rpy_(0) << " ";
    //  ifs << rpy_(1) << " ";
    //  ifs << rpy_(2) << std::endl;
    // 
    //  return EOK;
    //}
  };

  /* <width height hang vang near far> */
  class CameraSensor : public Sensor {
  private: 

  protected:
    size_t width_;
    size_t height_;
    Dp::Math::real pitch_;
    Dp::Math::real yaw_;
    Dp::Math::real near_;
    Dp::Math::real far_;

  public:
    CameraSensor() :
      Sensor(kSensorCamera),
      width_(0), height_(0), pitch_(0), yaw_(0), near_(0), far_(0) {};
    ~CameraSensor() {};

    void SetParam (size_t w, size_t h, Dp::Math::real pitch, Dp::Math::real yaw, Dp::Math::real near, Dp::Math::real far) {
      width_ = w;
      height_ = h;
      pitch_ = pitch;
      yaw_ = yaw;
      near_ = near;
      far_ = far;
    }
  };
}

#endif

