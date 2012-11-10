#ifndef __KBHIT_H__
#define __KBHIT_H__

#include <XnPlatform.h>
#if XN_PLATFORM == XN_PLATFORM_WIN32
#include <conio.h>
#else

int _kbhit(void);
#endif


#endif
