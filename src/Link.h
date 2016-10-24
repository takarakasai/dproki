
#ifndef LINK_H
#define LINK_H

#include <list>
#include <string>

#include <memory>

#include "dp_type.h"

#include "CasCoords.h"

#include "Joint.h"
#include "Sensor.h"
#include "Shape.h"

class Link : public CasCoords {
private:
  //const char* name_;
  std::string name_;
  std::string filepath_;

  std::shared_ptr<Joint> joint_;
  std::list<std::shared_ptr<Dp::Sensor>> sensors_;

  Eigen::Vector3d l_tippos_;
  Eigen::Vector3d l_tiprpy_; /* affect to l_tiprot_ */
  Eigen::Matrix3d l_tiprot_;
  Dp::Math::real     mass_;
  Eigen::Vector3d centroid_; /* center of gravity at local coordinates */
  Eigen::Vector3d wcentroid_; /* center of gravity at world coordinates */
  Eigen::Matrix3d cinertia_; /* inertia tensor at centroid at local coordinates */
  Eigen::Matrix3d ginertia_; /* inertia tensor at gravity centroid at local coordinates */

  /* for shape/hull */
  std::string shape_path_;
  std::string hull_path_;
  std::shared_ptr<Dp::Shape> shape_;
  std::shared_ptr<Dp::Shape> hull_;

  std::shared_ptr<Joint>& getJoint() {
    return joint_;
  }

protected:
  /* TODO */
  //std::shared_ptr<Link> parent_ = NULL;
  Link *parent_ = NULL;
  std::list<std::shared_ptr<Link>> clinks_;

public:
  //Link(const char* name) : name_(name) {};
  //Link(const char* name, std::shared_ptr<Joint> joint) : name_(name), joint_(joint) {};
  Link(
    const char* name, std::shared_ptr<Joint> joint, Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
    Matrix3d cinertia) :
    name_(name), joint_(joint), l_tippos_(lpos), l_tiprpy_(Vector3d::Zero()), l_tiprot_(Matrix3d::Identity()), mass_(mass), centroid_(centroid),
    wcentroid_(Eigen::Vector3d::Zero()), cinertia_(cinertia),
    shape_path_(""), hull_path_(""), shape_(nullptr), hull_(nullptr) {
    //ginertia_ = cinertia_ - mass_ * (centroid_.dot(centroid_) * Matrix3d::Identity() - centroid_ * centroid_.transpose());
    ginertia_ = cinertia_ - Dp::Phyx::pInertia(mass_, centroid_);
  };

  virtual ~Link() {};

  void SetShape(std::shared_ptr<Dp::Shape> shape) {
    shape_ = shape;
  }
  std::shared_ptr<Dp::Shape>  GetShape() {
    return shape_;
  }
  void SetHull(std::shared_ptr<Dp::Shape> hull) {
    hull_ = hull;
  }
  std::shared_ptr<Dp::Shape> GetHull() {
    return hull_;
  }

  void SetShapePath(std::string &path) {
    shape_path_ = path;
  }
  std::string &GetShapePath() {
    return shape_path_;
  }
  void SetHullPath(std::string &path) {
    hull_path_ = path;
  }
  std::string &GetHullPath() {
    return hull_path_;
  }

  void SetMass(Dp::Math::real mass) {
    mass_ = mass;
  }
  Dp::Math::real& GetMass() {
    return mass_;
  }

  void SetCentroid(Vector3d centroid) {
    centroid_ = centroid;
  }
  void ModCentroid(Vector3d centroid) {
    centroid_ = centroid;
    // TODO:
    //   current:   I   + pInertia
    //   to be  :Rt*I*R + pInertia
    cinertia_ = ginertia_ + Dp::Phyx::pInertia(mass_, centroid_);
  }
  Vector3d& GetCentroid() {
    return centroid_;
  }
  Vector3d& GetWCentroid() {
    /* TODO: update at UpdateCasCoords */
    wcentroid_ = WPos() + WRot() * centroid_;
    return wcentroid_;
  }

  /* affect all child's tippos w/o modifiying tippos of this link */
  void AddClinkOffset (Vector3d offset) {
    for (auto &link : clinks_) {
      link->LTipPos() += offset;
    }
  }

  void SetInertia(Matrix3d inertia) {
    cinertia_ = inertia;
  }
  Matrix3d& GetInertia() {
    return cinertia_;
  }

  static std::shared_ptr<Link> Create (
    const char* name, std::shared_ptr<Joint> joint, Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
    Matrix3d cinertia) {
    return std::make_shared<Link>(name, joint, lpos, mass, centroid, cinertia);
  }

  void AddSensor (std::shared_ptr<Dp::Sensor> sens) {
    sensors_.push_back(sens);
  }

  std::list<std::shared_ptr<Dp::Sensor>> &GetSensors() {
    return sensors_;
  }

  std::shared_ptr<Joint> GetPJoint() {
    return joint_;
  }

  Joint& GetJoint() {
    return *(joint_);
  }

  std::shared_ptr<Joint> FindJoint(const std::string& str) {
    /* TODO: duplicate */
    if (joint_->GetName() == str) {
      return joint_;
    }
    for (auto &link : clinks_) {
      auto joint = link->FindJoint(str);
      if (joint != NULL) {
        return joint;
      }
    }
    return NULL;
  }

  /* for returning std::shared_ptr<Link>, this function require 1st argument. */
  std::shared_ptr<Link> FindLink(std::shared_ptr<Link> this_, const std::string& str) {
  /* TODO: not Link* */
  //Link* FindLink(const std::string& str) {
    /* TODO: duplicate */
    if (GetName() == str) {
      return this_;
    }
    for (auto &link : clinks_) {
      auto lnk = link->FindLink(link, str);
      if (lnk != NULL) {
        return lnk;
      }
    }
    return NULL;
  }

  // TODO: --> CasCoords 
  Eigen::Vector3d& LTipPos() {
    return l_tippos_;
  }
  void SetTipRpy (const Eigen::Vector3d &rpy) {
    l_tiprpy_ = rpy;
    l_tiprot_ = Dp::Math::rpy2mat3(rpy);
  }
  Eigen::Vector3d& LTipRpy() {
    return l_tiprpy_;
  }
  Eigen::Matrix3d& LTipRot() {
    return l_tiprot_;
  }

  errno_t AssignJoint (std::shared_ptr<Joint> joint) {
    joint_ = std::move(joint);
    return 0;
  }

  //std::shared_ptr<Link> GetParent () {
  Link* GetParent () {
    return parent_;
  }

  errno_t SetParent (Link *link) {
    parent_ = link;
    return 0;
  }

  std::list<std::shared_ptr<Link>> GetChilds () {
    return clinks_;
  }

  errno_t AddChild (std::shared_ptr<Link> clink) {
    CasCoords::AddChild(clink);

    clink->SetParent(this);
    clinks_.push_back(clink);

    return 0;
  }

  // AddChild (std::shared_ptr<CasCoords> node) {
  //  node->SetParent(this->shared_from_this());
  //  cnodes.push_back(node);
  //}

  errno_t RemoveChild (std::shared_ptr<Link> clink) {
    CasCoords::RemoveChild(clink);

    clinks_.remove(clink);
    clink->SetParent(clink.get());

    return 0;
  }

  /*
   * before : this ------------> link
   * after  : this --> ilink --> link
   */
  errno_t InsertChild (std::shared_ptr<Link> ilink, std::shared_ptr<Link> link) {

    RemoveChild(link);
    ilink->AddChild(link);

    ilink->LTipPos() = link->LTipPos();
    link->LTipPos() = Eigen::Vector3d::Zero();

    ilink->SetTipRpy(link->LTipRpy());
    link->SetTipRpy(Eigen::Vector3d::Zero());
 
    AddChild(ilink);

    return 0;
  }

  errno_t ApplyLocalCoords() {
    /* pos & lot root of this link */
    //CasCoords::LRot() = joint_->Rot() * l_tiprot_;
    //std::cout << name_ << std::endl;
    CasCoords::LRot() = l_tiprot_ * joint_->Rot();
    CasCoords::LPos() = joint_->Pos() + l_tippos_;
    //std::cout << name_ << std::endl;
    //std::cout << "  POS:" << l_tippos_ << std::endl;
    //std::cout << "  POS:" << CasCoords::LPos() << std::endl;
    return 0;
  }

  errno_t UpdateLocals() {
    ECALL(ApplyLocalCoords());
    for (auto &link : clinks_) {
      link->UpdateLocals();
    }
    return 0;
  }

  errno_t UpdateCasCoords() {
    ECALL(UpdateLocals());      /* ローカル位置・姿勢更新 */
    ECALL(CasCoords::Update()); /* ワールド位置・姿勢更新 */
    return 0;
  }

  /* TODO: for joints */
  void Setup (std::list<std::shared_ptr<Link>>& links) {
    for (auto &link : clinks_) {
      links.push_back(link);
      link->Setup(links);
    }
  }

  std::string& GetName() {
    return name_;
  }

  void SetFilePath (std::string &file) {
    filepath_ = file;
  }
  std::string& GetFilePath () {
    return filepath_;
  }

};

class Object {
private:
  std::string name_;
  std::string filepath_;

  std::shared_ptr<Link> rlink_;       /* root link */
  std::list<std::shared_ptr<Link>> llinks_;
  std::vector<std::shared_ptr<Link>> vlinks_;

protected:

public:
  Object(const char* name) : name_(name) {}
  virtual ~Object() {
    vlinks_.reserve(0);
  }

  static std::shared_ptr<Object> Create (
    const char* name) {
    return std::make_shared<Object>(name);
  }

  void SetRootLink (std::shared_ptr<Link> rlink) {
    rlink_ = rlink;
  }
  std::shared_ptr<Link> RootLink() {
    return rlink_;
  }

  std::shared_ptr<Joint> FindJoint(const std::string& str) {
    return rlink_->FindJoint(str);
  }

  std::shared_ptr<Link> FindLink(const std::string& str) {
    return rlink_->FindLink(rlink_, str);
  }

  void UpdateCasCoords() {
    rlink_->UpdateCasCoords();
    return;
  }

  /* this API can be enabled after calling Setup() */
  size_t NumOfLinks () {
    //return vlinks_.capacity();
    return vlinks_.size();
    // return llinks_.size();
  }

  void Setup () {
    llinks_.clear();
    llinks_.push_back(rlink_);

    /* scan for all of descendants. */
    rlink_->Setup(llinks_);

    //vlinks_.reserve(llinks_.size());
    vlinks_.resize(llinks_.size());
    size_t idx = 0;
    for (auto &link : llinks_) {
      vlinks_[idx] = link;
      idx++;
    }
  }

  std::shared_ptr<Link> GetPLink(size_t idx) {
    return vlinks_[idx];
  }

  Link& GetLink(size_t idx) {
    return *(vlinks_[idx]);
  }

  void SetName (std::string &name) { name_ = name;}
  std::string& GetName() {
    return name_;
  }

  void SetFilePath (std::string &file) {
    filepath_ = file;
  }
  std::string& GetFilePath () {
    return filepath_;
  }

};

#endif

