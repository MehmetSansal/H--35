// user defined variables
#include <time.h>
// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA
#if MODE_STM_ENABLED == ENABLED
time_t pastt=time(NULL);
time_t ress=time(NULL);
#endif
#endif  // USERHOOK_VARIABLES



