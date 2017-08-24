
#ifndef DPRINT_H
#define DPRINT_H

#if defined(DP_DEBUG)
#define PRINTF(...) printf(__VA_ARGS__)
#define FPRINTF(...) fprintf(__VA_ARGS__)
#else
#define PRINTF(...)
#define FPRINTF(...)
#endif

#endif

