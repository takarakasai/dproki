
#include "p4model.h"

//#include <string>

#include "mf_errcheck.h"

#include "dp_type.h"
#include "Link.h"

#include "ObjFileReader.h"

typedef struct {
  Dp::Math::real length;
  Eigen::Vector3d centroid; /* center of gravity */  // centor of gravity at local coords (x,y,z) [m]
  Dp::Math::real Ixx, Iyy, Izz;                      // inertia (Ixx, Iyy, Izz) at center of gravity at local coords [Nm^2]
  Dp::Math::real Iyx, Izy, Izx;                      // inertia (Iyx, Izy, Izx) at center of gravity at local coords [Nm^2]
} p4_slide_param;

static const std::string kSlideLinkParentName = "ELBOW_PITCH";
static const std::string kSlideLinkName       = "ELBOW_SLIDE";
static const std::string kRotateLinkName      = "WRIST_PITCH";

p4_slide_param g_p4_camera[] = {
                 /* length      gpos(x)       gpos(y)      gpos(y)        Ixx           Iyy           Izz            Iyx           Izy            Izx       */
  /*P4ARM_SHORT */ {0.000, {0.00000000000, -0.000000000, 0.000000000}, 0.0000000000, 0.0000000000, 0.0000000000, 0.00000000000, 0.00000000000, 0.00000000000},
  /*P4ARM_MIDDLE*/ {0.000, {0.00000000000, -0.000000000, 0.000000000}, 0.0000000000, 0.0000000000, 0.0000000000, 0.00000000000, 0.00000000000, 0.00000000000},
  /*P4ARM_LONG  */ {0.000, {0.00000000000, -0.000000000, 0.000000000}, 0.0000000000, 0.0000000000, 0.0000000000, 0.00000000000, 0.00000000000, 0.00000000000}
};

p4_slide_param g_p4_lp_direct[] = {
                 /* length      gpos(x)       gpos(y)      gpos(y)        Ixx           Iyy           Izz            Iyx           Izy            Izx       */
  /*P4ARM_SHORT */ {0.055, {0.00077072751, -0.024674919, 0.075536834}, 0.0078643351, 0.0076461731, 0.0073809509, 6.7478269e-05, 7.5681968e-04, 3.5340506e-05},
  /*P4ARM_MIDDLE*/ {0.140, {0.00065794818, -0.024624080, 0.076597185}, 0.0079787319, 0.0077628818, 0.0073612165, 6.5143374e-05, 7.7882172e-04, 3.4944057e-05},
  /*P4ARM_LONG  */ {0.230, {0.00052450299, -0.024487268, 0.076916548}, 0.0081277523, 0.0079128889, 0.0073182862, 6.1692265e-05, 7.8900183e-04, 4.0942656e-05}
};

p4_slide_param g_p4_lp_oblique[] = {
                 /* length      gpos(x)       gpos(y)      gpos(y)        Ixx           Iyy           Izz            Iyx           Izy            Izx       */
  /*P4ARM_SHORT */ {0.000, {0.00000000000, -0.000000000, 0.000000000}, 0.0000000000, 0.0000000000, 0.0000000000, 0.00000000000, 0.00000000000, 0.00000000000},
  /*P4ARM_MIDDLE*/ {0.000, {0.00000000000, -0.000000000, 0.000000000}, 0.0000000000, 0.0000000000, 0.0000000000, 0.00000000000, 0.00000000000, 0.00000000000},
  /*P4ARM_LONG  */ {0.000, {0.00000000000, -0.000000000, 0.000000000}, 0.0000000000, 0.0000000000, 0.0000000000, 0.00000000000, 0.00000000000, 0.00000000000}
};

static const char* kP4CameraName    = "p4_camera";
static const char* kP4LpDirectName  = "p4_endo_straight";
static const char* kP4LpObliqueName = "p4_endo_oblique";

static p4_slide_param* Name2SlideParam (std::string &name) {
  if (name == kP4CameraName) {
    return g_p4_camera;
  } else if (name == kP4LpDirectName) {
    return g_p4_lp_direct;
  } else if (name == kP4LpObliqueName) {
    return g_p4_lp_oblique;
  } else {
    return nullptr;
  }
}

// [mm], [gmm^2]
// short 
//X   Y   Z   -2.4674919e+01 -7.7072751e-01  7.5536834e+01 ->   X:-0.024674919     Y:-0.00077072751   Z: 0.075536834      -Y: 0.00077072751   X:-0.024674919    Z: 0.075536834
//Ixx Ixy Ixz  7.6461731e+06 -6.7478269e+04  7.5681968e+05 -> Ixx: 0.0076461731                                      ->  Iyy: 0.0078643351  Ixx:0.0076461731  Izz: 0.0073809509
//Iyx Iyy Iyz -6.7478269e+04  7.8643351e+06 -3.5340506e+04 -> Iyx:-6.7478269e-05 Iyy: 0.0078643351                      -Iyx: 6.7478269e-05 Izx:7.5681968e-04-Izy: 3.5340506e-05
//Izx Izy Izz  7.5681968e+05 -3.5340506e+04  7.3809509e+05 -> Izx: 7.5681968e-04 Izy:-3.5340506e-05 Izz: 0.0073809509
//
//// middle
//X   Y   Z   -2.4624080e+01 -6.5794818e-01  7.6597185e+01 ->   X:-0.024624080     Y:-0.00065794818   Z: 0.076597185      -Y: 0.00065794818   X:-0.024624080    Z: 0.076597185
//Ixx Ixy Ixz  7.7628818e+06 -6.5143374e+04  7.7882172e+05 -> Ixx: 0.0077628818                                      ->  Iyy: 0.0079787319  Ixx: 0.0077628818 Izz: 0.0073612165
//Iyx Iyy Iyz -6.5143374e+04  7.9787319e+06 -3.4944057e+04 -> Iyx:-6.5143374e-05 Iyy: 0.0079787319                      -Iyx: 6.5143374e-05 Izx:7.7882172e-04-Izy: 3.4944057e-05
//Izx Izy Izz  7.7882172e+05 -3.4944057e+04  7.3612165e+05 -> Izx: 7.7882172e-04 Izy:-3.4944057e-05 Izz: 0.0073612165
//
//// long
//X   Y   Z   -2.4487268e+01 -5.2450299e-01  7.6916548e+01 ->   X:-0.024487268     Y:-0.00052450299   Z: 0.076916548      -Y: 0.00052450299   X:-0.024487268    Z: 0.076916548 
//Ixx Ixy Ixz  7.9128889e+06 -6.1692265e+04  7.8900183e+05 -> Ixx: 0.0079128889  Ixy:-6.1692265e-05 Ixz: 7.8900183e-04-> Iyy: 0.0081277523  Ixx: 0.0079128889 Izz: 0.0073182862
//Iyx Iyy Iyz -6.1692265e+04  8.1277523e+06 -4.0942656e+04 -> Iyx:-6.1692265e-05 Iyy: 0.0081277523  Iyz:-4.0942656e-05  -Iyx: 6.1692265e-05 Izx:7.8900183e-04-Izy: 4.0942656e-05
//Izx Izy Izz  7.8900183e+05 -4.0942656e+04  7.3182862e+05 -> Izx: 7.8900183e-04 Izy:-4.0942656e-05 Izz: 0.0073182862

typedef struct {
  std::shared_ptr<Object> obj;
} objdata;

errno_t p4model_open (void** inst, const char* resdir, const char* filepath) {
  EINVAL_CHECK(NULL, inst);

  std::string dir(resdir);
  std::string file(filepath);
  std::shared_ptr<Object> obj = ObjFileReader::ImportObjFile(dir, file);
  if (obj == nullptr) return -1;
  obj->Setup();
  obj->UpdateCasCoords();

  objdata *p = new objdata();
  p->obj = obj;
  *inst = (void*)p;
  //*inst = (void*)(obj.get());

  return EOK;
}

errno_t p4model_close (void* inst) {
  inst = nullptr;
  return EOK;
}

errno_t p4model_change_model (void* inst, P4ARM_LENGTH length, double wrist_pitch_value/*[deg]*/) {
  EINVAL_CHECK(NULL, inst);
  EINVAL_GELE_CHECK(P4ARM_MIN, length, P4ARM_MAX);

  std::shared_ptr<Object> pobj = ((objdata*)inst)->obj;

  std::string name(pobj->GetName().c_str());
  p4_slide_param* param = Name2SlideParam(name);
  if (param == nullptr) {
    return -1;
  }

  /** link parameter **/
  /* SLIDE LINK centroid/inertia */
  auto link = pobj->FindLink(kSlideLinkParentName);
  if (link == nullptr) return -2;

  link->SetCentroid(param[length].centroid);

  Eigen::Matrix3d inertia(3,3);
  inertia << param[length].Ixx, param[length].Iyx, param[length].Izx,
             param[length].Iyx, param[length].Iyy, param[length].Izy,
             param[length].Izx, param[length].Izy, param[length].Izz;
  link->SetInertia(inertia);

  /* SLIDE LINK value */
  auto slide_link = pobj->FindLink(kSlideLinkName);
  if (slide_link == nullptr) return -3;

  slide_link->LTipPos()(2) = param[length].length;

  /* ROTATE LINK value */
  auto rotate_link = pobj->FindLink(kRotateLinkName);
  if (rotate_link == nullptr) return -4;

  rotate_link->LTipRpy()(1) = wrist_pitch_value;
  //rotate_link->LTipRpy()(1) = Dp::Math::rad2deg(wrist_pitch_value);

  // TODO: remove reinterpret_cast
  /** shape parameter **/
  /* SLIDE LINK */
  std::cout << "TYPP:" << slide_link->GetShape()->Type() << std::endl;
  auto shape = slide_link->GetShape();
  if (shape->Type() != Dp::kShapeCompound) {
    return -5;
  }
  auto comp = reinterpret_cast<Dp::Compound*>(shape.get());
  auto childs = comp->GetChilds();
  for (auto &child : childs) {
    //std::cout << "t: " << child.filepath;
    if (child.filepath == "/obj/p4_endo_straight/ELBOW_SLIDEBodyFace2.shp") {
      auto rect = reinterpret_cast<Dp::Rectangular*>(child.shape.get());
      rect->Height() = param[length].length;
    }
  }

  return EOK;
}

errno_t p4model_add_camera (void* inst, const char* lnk_name, camera_sensor *cam) {
  EINVAL_CHECK(NULL, inst);
  EINVAL_CHECK(NULL, cam);
  EINVAL_CHECK(NULL, cam->name);

  std::shared_ptr<Object> pobj = ((objdata*)inst)->obj;

  auto link = pobj->FindLink(lnk_name);
  if (link == nullptr) {
    return -10;
  }

  auto sens = std::make_shared<obj::CameraSensor>();
  if (sens == nullptr) {
    return -11;
  }

  std::string name(cam->name);
  sens->SetName(name);
  sens->GetOffsetXyz() = Eigen::Vector3d(cam->xyz);
  sens->GetOffsetRpy() - Eigen::Vector3d(cam->rpy);
  sens->SetParam(cam->w, cam->h, cam->pitch, cam->yaw, cam->near, cam->far);

  link->AddSensor(sens);

  return EOK;
}

errno_t p4model_saveas (void* inst, const char* filepath) {
  EINVAL_CHECK(NULL, inst);
  EINVAL_CHECK(NULL, filepath);

  std::string dir = filepath;
  std::shared_ptr<Object> pobj = ((objdata*)inst)->obj;

  EFUNC(ObjFileReader::Export(dir, *pobj));

  return EOK;
}

#if 0
int main(void) {

    std::string dirpath = "../../../resources";
    std::string filename = "/obj/p4_endo_straight/P4ARM_ENDO_STRAIGHT.obj";

    auto robot = ObjFileReader::ImportObjFile(dirpath, filename);
    if (robot == NULL) {
      fprintf(stderr, "fail to load %s : %s.\n", dirpath.c_str(), filename.c_str());
      return 1;
    }
    robot->Setup();
    robot->UpdateCasCoords();

    printf("--- LINK NUM: %zd\n", robot->NumOfLinks());

    std::string dir = "./test/";
    ObjFileReader::Export(dir, *robot);
  
    return 0;
}
#endif


