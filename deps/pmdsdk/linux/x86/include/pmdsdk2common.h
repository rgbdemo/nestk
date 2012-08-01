/*****************************************************************************
 * PMDSDK 2
 * 
 * Copyright (c) 2006-2010 PMD Technologies GmbH
 * All Rights Reserved.
 *
 * File: pmdsdk2common.h
 * Author: Martin Profittlich
 * Created: 20060530
 *
 *****************************************************************************/

#ifndef MSDK2COMMON_H_82635748816555424683995524346951 
#define MSDK2COMMON_H_82635748816555424683995524346951

#include <stddef.h>

#ifdef __cplusplus
# define BEGIN_EXTERN_C extern "C" {
# define END_EXTERN_C }
#else
# define BEGIN_EXTERN_C
# define END_EXTERN_C
#endif

#ifdef __cplusplus
# define BEGIN_PMD_NAMESPACE namespace pmd {
# define END_PMD_NAMESPACE }
#endif

#ifndef _WIN32
# define PMD_EXPORT
#else
# ifndef PMD_EXPORT
#  define PMD_EXPORT  __declspec(dllimport)
# endif
#endif

typedef enum { AtLeast, AtMost, CloseTo } Proximity;

#define PMD_OK 0

#define PMD_RUNTIME_ERROR 1024
#define PMD_GENERIC_ERROR 1025
#define PMD_DISCONNECTED 1026
#define PMD_INVALID_VALUE 1027
#define PMD_TIMEOUT_ERROR 1028

#define PMD_LOGIC_ERROR 2048
#define PMD_UNKNOWN_HANDLE 2049
#define PMD_NOT_IMPLEMENTED 2050
#define PMD_OUT_OF_BOUNDS 2051

#define PMD_RESOURCE_ERROR 4096
#define PMD_FILE_NOT_FOUND 4097
#define PMD_COULD_NOT_OPEN 4098
#define PMD_DATA_NOT_FOUND 4099
#define PMD_END_OF_DATA 4100

#ifndef PMD_NO_DEPRECATED
#define PMD_COULD_NOT_CONNECT 4098
#endif

#define PMD_INTERFACE_VERSION_1_0_0 0x00010000
#define PMD_INTERFACE_VERSION_1_1_0 0x00010100
#define PMD_INTERFACE_VERSION_1_1_1 0x00010101
#define PMD_INTERFACE_VERSION_1_2_0 0x00010200
#define PMD_INTERFACE_VERSION_1_2_1 0x00010201
#define PMD_INTERFACE_VERSION_1_3_0 0x00010300
#define PMD_CURRENT_INTERFACE_VERSION PMD_INTERFACE_VERSION_1_3_0

BEGIN_EXTERN_C

unsigned pmdNewGUID ();

struct PMDDataDescription;
void pmdHelperConvertDDByteOrder (struct PMDDataDescription * dd);

END_EXTERN_C

#endif 
