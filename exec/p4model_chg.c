
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "mf_errcheck.h"

#include "p4model.h"

int main(int argc, char *argv[]) {

    if (argc < 6) {
      goto ERROR;
    }

    //std::string dirpath = "../../../resources";
    //std::string filename = "/obj/p4arm/p4arm.obj";
    const char* dirpath = argv[1];
    const char* filename = argv[2];
    const char* savepath = argv[3];

    if (strcmp(dirpath, savepath) == 0) {
      fprintf(stderr, " -- res_dirpath and save_dirpath must not be same : save_dirpath:%s.\n", savepath);
      goto ERROR;
    }

    P4ARM_LENGTH p4arm_len;
    if (strcmp(argv[4], "S") == 0) {
      p4arm_len = P4ARM_SHORT;
    } else if (strcmp(argv[4], "M") == 0) {
      p4arm_len = P4ARM_MIDDLE;
    } else if (strcmp(argv[4], "L") == 0) {
      p4arm_len = P4ARM_LONG;
    } else {
      fprintf(stderr, " -- invalid slide value (should be S|L|M) : %s.\n", argv[4]);
      goto ERROR;
    }

    double rot_value = strtod(argv[5], NULL);
    if (rot_value < -180 || 180 < rot_value) {
      fprintf(stderr, " -- invalid rot_value : %lf[deg]\n", rot_value);
      goto ERROR;
    }

    void* hdl;
    EFUNC(p4model_open(&hdl, dirpath, filename));
    EFUNC(p4model_change_model(hdl, p4arm_len, rot_value));

    if (argc > 6) {
      EFUNC(p4model_saveas2(hdl, savepath, argv[6]));
    } else { /* argc == 6 */
      EFUNC(p4model_saveas(hdl, savepath));
    }

    EFUNC(p4model_close(hdl));

    return 0;
 
ERROR:
    fprintf(stdout, "%s <res_dirpath> <filepath> <save_dirpath> <S|M|L> <rot_value[deg] [save_filerootpath]>\n", argv[0]);

    return 1;
}


