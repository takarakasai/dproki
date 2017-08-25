#ifndef DRAWABLELINK_H
#define DRAWABLELINK_H

#include <list>

#include "dpcommon/dp_type.h"

#include "Link.h"
#include "InterfaceSceneObject.h"

using namespace Eigen;

/* ssg : Simple Scene Graph */
namespace Dp {

  class DrawableLink : public InterfaceSceneObject/*Implement*/ {
  private:
    std::string name_;
    
    std::shared_ptr<Link> link;

    //std::shared_ptr<InterfaceSceneObject> obj_ = NULL;
    std::list<std::shared_ptr<InterfaceSceneObject>> objs_;
  
  public:
  
    DrawableLink(const char* name) :
         name_(name) {
    };

    virtual ~DrawableLink() {};

    static std::shared_ptr<DrawableLink> Create(const char* name) {
      return std::make_shared<DrawableLink>(name);
    }

    void AddShape (std::shared_ptr<InterfaceSceneObject> obj) {
      objs_.push_back(obj);
    }

    void AddShape (std::list<std::shared_ptr<InterfaceSceneObject>> objs) {
      objs_.splice(objs_.end(), objs);
    }

    std::list<std::shared_ptr<InterfaceSceneObject>> GetShapes() {
      return objs_;
    }

  public: /* InterfaceSceneObject */
    errno_t Exec(void) {
      for (auto &obj : objs_) {
        obj->Draw(CasCoords::WRot(), CasCoords::WPos());
      }
      return 0;
    }
    errno_t Draw(void) {
      ECALL(ExecAll());
      return 0;
    }

    errno_t Draw(Eigen::Matrix3d& rot, Eigen::Vector3d& pos) {
      return -1;
    }

    errno_t SetTransformMatrixLocId (int32_t id) {
      for (auto &obj : objs_) {
        obj->SetTransformMatrixLocId(id);
      }
      for (auto &link : clinks_) {
        link->SetTransformMatrixLocId(id);
      }
      return 0;
    }

    errno_t SetMaterialColorLocId (int32_t id) {
      for (auto &obj : objs_) {
        obj->SetMaterialColorLocId(id);
      }
      for (auto &link : clinks_) {
        link->SetMaterialColorLocId(id);
      }
      return 0;
    }

    errno_t SetTextureLocId (int32_t id) {
      for (auto &obj : objs_) {
        obj->SetTextureLocId(id);
      }
      for (auto &link : clinks_) {
        link->SetTextureLocId(id);
      }
      return 0;
    }

    errno_t SetScale(const Eigen::Vector3d& scale) {
      return -1;
    }

    errno_t SetOffset(const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot) {
      return -1;
    }

    errno_t SetColor(Eigen::Vector4d& color) {
      return -1;
    }

    /* TODO : 
     * should be removed, but InterfaceSceneObject required this currently for dGeomTriMeshDataBuildSingle1 */ 
    Dp::Vertices& GetVertices() {
      return (*objs_.begin())->GetVertices();
    }
    /* TODO : 
     * should be removed, but InterfaceSceneObject required this currently for dGeomTriMeshDataBuildSingle1 */ 
    std::vector<GLuint>& GetIndices() {
      return (*objs_.begin())->GetIndices();
    }

    errno_t SetDrawMode (InterfaceSceneObject::DrawMode mode) {
      for (auto &obj : objs_) {
        obj->SetDrawMode(mode);
      }
      for (auto &link : clinks_) {
        link->SetDrawMode(mode);
      }

      return 0;
    }

    Coordinates& GetCoordinates() {
      return CasCoords::World();
    }

  };
}

#endif

