
#include "p4model.h"

//#include <string>

#include "mf_errcheck.h"

#include "dprint.h"

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
static const std::string kMINMAXLinkName      = "CAMERA_YAW";

static const char* p4arm_length2link_hooter (P4ARM_LENGTH length) {
  switch (length) {
    case P4ARM_SHORT:  return "_SHORT";
    case P4ARM_MIDDLE: return "_MIDDLE";
    case P4ARM_LONG:   return "_LONG";
    default:           return nullptr;
  }
}

const P4ARM_LENGTH p4arm_str2length (const char *str) {
  if (str == NULL || strlen(str) != 1) {
    return P4ARM_INVAL;
  }
  switch (str[0]) {
    case 'S': return P4ARM_SHORT;
    case 'M': return P4ARM_MIDDLE;
    case 'L': return P4ARM_LONG;
    default:  return P4ARM_INVAL;
  }
}

#if 0
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
#endif

#if 0
static const char* kP4CameraName    = "p4_camera";
static const char* kP4LpDirectName  = "p4_endo_straight";
static const char* kP4LpObliqueName = "p4_endo_oblique";
#endif

static const char* kSenserFooter = "_SENS";
static const char* kLinkExt      = ".lnk";
 
#if 0
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
#endif

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
  std::string dir;
  std::string root_file;
 
  std::shared_ptr<Object> obj;
} objdata;

errno_t p4model_open (void** inst, const char* resdir, const char* filepath) {
  EINVAL_CHECK(NULL, inst);

  objdata *p = new objdata();

  p->dir  = resdir;
  p->root_file = filepath;
  std::shared_ptr<Object> obj = ObjFileReader::ImportObjFile(p->dir, p->root_file);
  if (obj == nullptr) {
    delete p;
    return -1;
  }
  obj->Setup();
  obj->UpdateCasCoords();

  p->obj = obj;
  *inst = (void*)p;

  return EOK;
}

errno_t p4model_close (void* inst) {
  inst = nullptr;
  return EOK;
}

errno_t p4model_change_model (void* inst, P4ARM_LENGTH length, double wrist_pitch_value/*[deg]*/) {
  EINVAL_CHECK(NULL, inst);
  EINVAL_GELE_CHECK(P4ARM_MIN, length, P4ARM_MAX);

  objdata* odata = (objdata*)inst;
  std::shared_ptr<Object> pobj = odata->obj;

  std::string robot_name(pobj->GetName().c_str());
#if 0
  p4_slide_param* param = Name2SlideParam(robot_name);
  if (param == nullptr) {
    return -1;
  }
#endif

  /** link parameter **/
  /* SLIDE LINK centroid/inertia */
#if 1
  const char* hooter = p4arm_length2link_hooter(length);
  EINVAL_CHECK(nullptr, hooter);
  /*                  ex : /obj/p4_endo_straight/ELBOW_PITCH_LONG.lnk */
  std::string link_file = "/obj/" + robot_name + "/" + kSlideLinkParentName + hooter + ".lnk";
  std::shared_ptr<Link> link_from = ObjFileReader::ImportLinkFile(link_file, 1);
  if (link_from == nullptr) return -2;

  auto link_to = pobj->FindLink(kSlideLinkParentName);
  if (link_to == nullptr) return -3;

  link_to->SetMass(link_from->GetMass());
  link_to->SetCentroid(link_from->GetCentroid());
  link_to->SetInertia(link_from->GetInertia());

#else
  auto link = pobj->FindLink(kSlideLinkParentName);
  if (link == nullptr) return -2;

  link->SetCentroid(param[length].centroid);

  Eigen::Matrix3d inertia(3,3);
  inertia << param[length].Ixx, param[length].Iyx, param[length].Izx,
             param[length].Iyx, param[length].Iyy, param[length].Izy,
             param[length].Izx, param[length].Izy, param[length].Izz;
  link->SetInertia(inertia);
#endif

  /* SLIDE LINK value */
#if 1
  auto slide_link_from = link_from->FindLink(link_from, kSlideLinkName);
  if (slide_link_from == nullptr) return -4;

  auto slide_link_to = pobj->FindLink(kSlideLinkName);
  if (slide_link_to == nullptr) return -5;

  slide_link_to->LTipPos()(2) = slide_link_from->LTipPos()(2);
#else
  auto slide_link = pobj->FindLink(kSlideLinkName);
  if (slide_link == nullptr) return -3;

  slide_link->LTipPos()(2) = param[length].length;
#endif

  /* ROTATE LINK value */
  auto rotate_link = pobj->FindLink(kRotateLinkName);
  if (rotate_link == nullptr) return -6;

  rotate_link->LTipRpy()(1) = wrist_pitch_value;
  //rotate_link->LTipRpy()(1) = Dp::Math::rad2deg(wrist_pitch_value);

  // TODO: remove reinterpret_cast
  /** shape parameter **/
  /* SLIDE LINK */
#if 1
  std::cout << "TYPP:" << slide_link_to->GetShape()->Type() << std::endl;
  auto shape = slide_link_to->GetShape();
#else
  std::cout << "TYPP:" << slide_link->GetShape()->Type() << std::endl;
  auto shape = slide_link->GetShape();
#endif
  if (shape->Type() != Dp::kShapeCompound) {
    return -7;
  }
  auto comp = reinterpret_cast<Dp::Compound*>(shape.get());
  auto childs = comp->GetChilds();
  for (auto &child : childs) {
    //std::cout << "t: " << child.filepath;
    std::string shape_file = "/obj/" + robot_name + "/" + kSlideLinkName + "BodyFace2" + ".shp";
    if (child.filepath == shape_file) {
    //if (child.filepath == "/obj/p4_endo_straight/ELBOW_SLIDEBodyFace2.shp") {
      auto rect = reinterpret_cast<Dp::Rectangular*>(child.shape.get());
#if 1
      rect->Height() = slide_link_to->LTipPos()(2);
#else
      rect->Height() = param[length].length;
#endif
    }
  }

  return EOK;
}

errno_t p4model_change_model_ex(void* inst, P4ARM_LENGTH length, double wrist_pitch_value/*[deg]*/, double camera_yaw_minmax[2]/*[deg]*/)
{
  /* ROTATE LINK */
  /* SLIDE LINK  */
  EFUNC(p4model_change_model(inst, length, wrist_pitch_value));

  std::shared_ptr<Object> pobj = ((objdata*)inst)->obj;

  /* LINK min/max value */
  auto minmax_link = pobj->FindLink(kMINMAXLinkName);
  if (minmax_link == nullptr) {
    return -10;
  }

  /* can not use following code, auto is evaluated as Joint not Joint& in this case.
   * auto joint = ***
   */
  Joint& joint = minmax_link->GetJoint();
  joint.SetRange(0, Dp::Math::deg2rad(camera_yaw_minmax[0]), Dp::Math::deg2rad(camera_yaw_minmax[1]));

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

errno_t p4model_add_dummy_rotary_links (void* inst) {
  EINVAL_CHECK(NULL, inst);

  std::shared_ptr<Object> pobj = ((objdata*)inst)->obj;

  size_t nol = pobj->NumOfLinks();
  for (size_t i = 1; i < nol; i++) {

    auto lnk  = pobj->GetLink(i);
    auto plnk = pobj->GetPLink(i);

    /* handle only for rotary-joint */
    /* TODO: should not use '<=' */
    if (!(kRotaryX <= lnk.GetJoint().GetType() && lnk.GetJoint().GetType() <= kRotaryU)) {
      continue;
    }

   
    auto jname = lnk.GetJoint().GetName() + kSenserFooter;
    auto lname = lnk.GetName()            + kSenserFooter;

    auto cjoint = std::dynamic_pointer_cast<RotaryJoint>(lnk.GetPJoint());
    if (cjoint.use_count() == 0) {
      return -11;
    }

    auto joint = RotaryJoint::Create(jname.c_str(), cjoint->Axis());
    joint->SetRange(0, -Dp::Math::PI, Dp::Math::PI);
    auto inode = Link::Create(lname.c_str(), joint, (Vector3d){0.0, 0.0, 0.0}, 0, (Vector3d){0.0,0.0,0.0}, Matrix3d::Zero());
    auto filepath  = lnk.GetFilePath();
    auto filepath2 = filepath.substr(0, filepath.rfind(kLinkExt)) + kSenserFooter + kLinkExt;
    inode->SetFilePath(filepath2);

    plnk->GetParent()->InsertChild(inode, plnk);
  }

  auto name = pobj->GetName() + kSenserFooter;
  pobj->SetName(name);
  pobj->Setup();
  nol = pobj->NumOfLinks();

  return EOK;
}

errno_t p4model_saveas (void* inst, const char* respath) {
  EINVAL_CHECK(NULL, inst);
  EINVAL_CHECK(NULL, respath);

  std::string dir = respath;
  std::shared_ptr<Object> pobj = ((objdata*)inst)->obj;

  EFUNC(ObjFileReader::Export(dir, *pobj));

  return EOK;
}

errno_t p4model_saveas2 (void* inst, const char* respath, const char* fileroot) {
  EINVAL_CHECK(NULL, inst);
  EINVAL_CHECK(NULL, respath);
  EINVAL_CHECK(NULL, fileroot);

  std::string resdir = respath;
  std::string filedir = fileroot;
  std::shared_ptr<Object> pobj = ((objdata*)inst)->obj;

  EFUNC(ObjFileReader::Export(resdir, filedir, *pobj));

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

    PRINTF("--- LINK NUM: %zd\n", robot->NumOfLinks());

    std::string dir = "./test/";
    ObjFileReader::Export(dir, *robot);
  
    return 0;
}
#endif


