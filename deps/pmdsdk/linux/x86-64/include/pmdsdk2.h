/* 
 * PMDSDK 2
 *
 * File: pmdsdk2.h
 * Author: Martin Profittlich
 *
 * General header file for applications using the PMDSDK 2.0.
 * Contains all necessary definitions and prototypes.
 *
 * Copyright (c) 2006-2010 PMD Technologies GmbH.
 * All Rights Reserved.
 *
 */

#ifndef PMDMSDK2_H_INCLUDED_2503171013
#define PMDMSDK2_H_INCLUDED_2503171013

#include <pmddatadescription.h>

#ifdef _WIN32
# ifndef DLLSPEC
#  define DLLSPEC __declspec(dllimport)
# endif
#else
# ifdef DLLSPEC
#  undef DLLSPEC
# endif
# define DLLSPEC
#endif


#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned PMDHandle;

DLLSPEC int pmdOpen (PMDHandle * hnd, const char * rplugin, const char * rparam, const char * pplugin, const char * pparam);
DLLSPEC int pmdOpenSourcePlugin (PMDHandle * hnd, const char * rplugin, const char * rparam);
DLLSPEC int pmdClose (PMDHandle hnd);

DLLSPEC int pmdGetLastError (PMDHandle hnd, char * error, size_t maxLen);

DLLSPEC int pmdUpdate (PMDHandle hnd);

DLLSPEC int pmdSetIntegrationTime (PMDHandle hnd, unsigned idx, unsigned t);
DLLSPEC int pmdGetIntegrationTime (PMDHandle hnd, unsigned * t, unsigned idx);
DLLSPEC int pmdGetValidIntegrationTime (PMDHandle hnd, unsigned * result, unsigned idx, Proximity w, unsigned t);
DLLSPEC int pmdSetModulationFrequency (PMDHandle hnd, unsigned idx, unsigned f);
DLLSPEC int pmdGetModulationFrequency (PMDHandle hnd, unsigned * f, unsigned idx);
DLLSPEC int pmdGetValidModulationFrequency (PMDHandle hnd, unsigned * result, unsigned idx, Proximity w, unsigned f);

DLLSPEC int pmdGetSourceData (PMDHandle hnd, void * data, size_t maxLen);

DLLSPEC int pmdGetSourceDataSize (PMDHandle hnd, size_t * size);

DLLSPEC int pmdGetSourceDataDescription (PMDHandle hnd, struct PMDDataDescription * dd);

DLLSPEC int pmdGetDistances (PMDHandle hnd, float * data, size_t maxLen);
DLLSPEC int pmdGet3DCoordinates (PMDHandle hnd, float * data, size_t maxLen);
DLLSPEC int pmdGetAmplitudes (PMDHandle hnd, float * data, size_t maxLen);
DLLSPEC int pmdGetIntensities (PMDHandle hnd, float * data, size_t maxLen);
DLLSPEC int pmdGetFlags (PMDHandle hnd, unsigned * data, size_t maxLen);

DLLSPEC int pmdSourceCommand (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);

DLLSPEC int pmdProcessingCommand (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);

DLLSPEC int pmdOpenProcessingPlugin (PMDHandle * hnd, const char * pplugin, const char * pparam);

DLLSPEC int pmdCalcDistances (PMDHandle hnd, float * data, size_t maxLen, struct PMDDataDescription sourceDD, void * sourceData);
DLLSPEC int pmdCalcAmplitudes (PMDHandle hnd, float * data, size_t maxLen, struct PMDDataDescription sourceDD, void * sourceData);
DLLSPEC int pmdCalcIntensities (PMDHandle hnd, float * data, size_t maxLen, struct PMDDataDescription sourceDD, void * sourceData);
DLLSPEC int pmdCalc3DCoordinates (PMDHandle hnd, float * data, size_t maxLen, struct PMDDataDescription sourceDD, void * sourceData);
DLLSPEC int pmdCalcFlags (PMDHandle hnd, unsigned * data, size_t maxLen, struct PMDDataDescription sourceDD, void * sourceData);

#ifndef PMD_NO_DEPRECATED
// DEPRECATED
DLLSPEC int pmdConnect (PMDHandle * hnd, const char * rplugin, const char * rparam, const char * pplugin, const char * pparam);
DLLSPEC int pmdConnectOnlyRaw (PMDHandle * hnd, const char * rplugin, const char * rparam);
DLLSPEC int pmdDisconnect (PMDHandle hnd);
DLLSPEC int pmdOpenAccessPlugin (PMDHandle * hnd, const char * rplugin, const char * rparam);
DLLSPEC int pmdGetRawData (PMDHandle hnd, void * data, size_t maxLen);
DLLSPEC int pmdGetRawDataSize (PMDHandle hnd, size_t * size);
DLLSPEC int pmdGetRawDataDescription (PMDHandle hnd, struct PMDDataDescription * dd);
DLLSPEC int pmdPlatformCommand (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);
DLLSPEC int pmdConfigureProcess (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);
DLLSPEC int pmdGetInfo (PMDHandle hnd, char * result, size_t maxLen, const char * key);
#endif

#ifdef __cplusplus
}
#endif

#endif
