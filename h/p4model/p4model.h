
#ifndef P4MODEL_H
#define P4MODEL_H

#include "mf_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  P4ARM_MIN    = 0,
  P4ARM_SHORT  = 0,
  P4ARM_MIDDLE = 1,
  P4ARM_LONG   = 2,
  P4ARM_MAX    = 2
} P4ARM_LENGTH;

static inline const char* p4arm_length2str (P4ARM_LENGTH length) {
  switch (length) {
    case P4ARM_SHORT:  return "short";
    case P4ARM_MIDDLE: return "middle";
    case P4ARM_LONG:   return "long";
    default:           return "ivalid";
  }
}

errno_t p4model_open (void** inst, const char* resdir, const char* filepath);
errno_t p4model_close (void* inst);

errno_t p4model_change_model(void* inst, P4ARM_LENGTH length, double wrist_pitch_value/*[deg]*/);
errno_t p4model_saveas (void* inst, const char* respath);
errno_t p4model_saveas2 (void* inst, const char* respath, const char* fileroot);

typedef struct {
  const char* name;

  size_t w;
  size_t h;
  double pitch;
  double yaw;
  double near;
  double far;

  double xyz[3];
  double rpy[3]; /* [deg] */
} camera_sensor;

errno_t p4model_add_camera(void* inst, const char *lnk_name, camera_sensor *param);

#ifdef __cplusplus
};
#endif

#endif

