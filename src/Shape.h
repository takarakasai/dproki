
#ifndef SHAPE_H
#define SHAPE_H

#include "dp_type.h"

#include <memory>

namespace Dp {
  
  typedef enum {
    kShapeInvalid,
    kShapeStl,
    kShapeCompound,
    kShapeSphere,
    kShapeCylinder,
    kShapeRectangular,
    kShapeCone
  } SHAPE_TYPE;
  
  class Shape { 
  private:
    SHAPE_TYPE type_;

    Eigen::Vector4d color_;
  
    Eigen::Vector3d xyz_;   /* offset */
    Eigen::Vector3d rpy_;   /* offset [rad] */
    Eigen::Vector3d scale_;
   
  public:
    Shape(SHAPE_TYPE type) : type_(type), color_(0.,0.,0.,1.), xyz_(0.,0.,0.), rpy_(0.,0.,0.), scale_(1.,1.,1.) {};
    ~Shape() {};
  
    SHAPE_TYPE Type() { return type_;};

    void SetColor(Eigen::Vector4d color)   { color_ = color;};
    Eigen::Vector4d& GetColor() { return color_;};
  
    void SetOffsetXyz(Eigen::Vector3d xyz) { xyz_   = xyz;}
    Eigen::Vector3d& GetOffsetXyz()        { return xyz_;}
    void SetOffsetRpy(Eigen::Vector3d rpy) { rpy_   = rpy;}
    Eigen::Vector3d& GetOffsetRpy()        { return rpy_;}
    void SetScale(Eigen::Vector3d scale)   { scale_ = scale;}
    Eigen::Vector3d& GetScale()            { return scale_;}

    virtual errno_t parse   (std::ifstream &ifs) = 0;
    virtual errno_t compose (std::ofstream &ofs) = 0;
    //virtual errno_t parse   (std::ifstream &ifs) { return -1;};
    //virtual errno_t compose (std::ofstream &ofs) { return -1;};
  };
  
  class Stl : public Shape {
  private: 
  
  protected:
    std::string filename_;

  public:
    Stl() :
      Shape(kShapeStl), filename_("") {};
    ~Stl() {};
  };
  
  class Compound : public Shape {
    typedef struct {
      std::string filepath;
      std::shared_ptr<Shape> shape;
    } CompoundData;
  private: 
  
  protected:
    std::list<CompoundData> compounds_;
    //std::list<std::string> cfilepath;
    //std::list<std::shared_ptr<Shape>> cshapes;

  public:
    Compound() :
      Shape(kShapeCompound) {};
    ~Compound() {};
  
    void Add (std::string &filepath, std::shared_ptr<Shape> shape) {
      CompoundData data = {filepath, shape};
      compounds_.push_back(data);
      //cfilepath.push_back(filepath);
      //cshapes.push_back(shape);
    };

    std::list<CompoundData> &GetChilds() {
      return compounds_;
    }

  };
  
  class Sphere : public Shape {
  private: 
  
  protected:
    Dp::Math::real radius_;

  public:
    Sphere() :
      Shape(kShapeSphere), radius_(0.) {};
    ~Sphere() {};
  };
  
  class Cylinder : public Shape {
  private: 
 
  protected:
    Dp::Math::real radius_;
    Dp::Math::real height_;
 
  public:
    Cylinder() :
      Shape(kShapeCylinder), radius_(0.), height_(0.) {};
    ~Cylinder() {};
  };
  
  class Rectangular : public Shape {
  private: 

  protected:
    Dp::Math::real width_;  /* x */
    Dp::Math::real length_; /* y */
    Dp::Math::real height_; /* z */

  public:
    Rectangular() :
      Shape(kShapeRectangular), width_(0.), length_(0.), height_(0.) {};
    ~Rectangular() {};

    Dp::Math::real &Width()  { return width_;}
    Dp::Math::real &Length() { return length_;}
    Dp::Math::real &Height() { return height_;}
  };
  
  class Cone : public Shape {
  private: 
 
  protected:
    Dp::Math::real radius_;
    Dp::Math::real height_;
 
  public:
    Cone() :
      Shape(kShapeCone), radius_(0.), height_(0.) {};
    ~Cone() {};
  };
}

#endif

