/*****************************************************************************
 * PMDSDK 2
 * 
 * Copyright (c) 2006-2010 PMD Technologies GmbH
 * All Rights Reserved.
 *
 * File: pmddatadescription.h
 * Author: Martin Profittlich
 * Created: 20060808
 *
 *****************************************************************************/

#ifndef DATADESCRIPTION_H_672861066752896024487934289650
#define DATADESCRIPTION_H_672861066752896024487934289650

/// \addtogroup core
/// @{

#include "pmdsdk2common.h"

BEGIN_EXTERN_C


#define PMD_RAW_L16_AB        0x00001006u
#define PMD_RAW_B16_AB        0x00009006u
#define PMD_RAW_L16_DAB       0x0000100Eu
#define PMD_RAW_B16_DAB    0x0000900Eu
#define PMD_RAW_L16_ABD       0x00001007u
#define PMD_RAW_B16_ABD    0x00009007u
#define PMD_RAW_P12_D         0x00000C01u
#define PMD_RAW_L16_D         0x00001001u

#define PMD_REVERSED_PHASES   0x00000001u

#define PMD_SKIP_1_FRAME      0x04000000u
#define PMD_SKIP_2_FRAMES     0x08000000u
#define PMD_SKIP_3_FRAMES     0x0C000000u

#define PMD_UNKNOWN_DATA   0x00000000u

#define PMD_DISTANCE_LF32       0x00010001u
#define PMD_AMPLITUDE_LF32      0x00010002u
#define PMD_INTENSITY_LF32      0x00010004u
#define PMD_REFLECTIVITY_LF32   0x00010008u

#define PMD_FLAGS_L16            0x00010010u
#define PMD_FLAGS_L32            0x00010400u

#define PMD_X_COORD_LF32        0x00010020u
#define PMD_Y_COORD_LF32        0x00010040u
#define PMD_Z_COORD_LF32        0x00010080u
#define PMD_XYZ_COORD_LF32      0x000100E0u
#define PMD_RGB_U8              0x00010100u
#define PMD_RGB_LF32            0x00010200u
#define PMD_PACKAGED_DATA       0x00020000u

#define PMD_DISTANCE_I16_AMPLITUDE_I16 0x00020003u
#define PMD_DISTANCE_100UM_I16_AMPLITUDE_I16 0x00020103u

#define PMD_A3_DATA_2_31      0x00040003u
#define PMD_A3_DATA_4         0x00040005u
#define PMD_A3_RAWDATA_2_31   0x00040007u
#define PMD_A3_COMBINED_2_31     0x00040009u
#define PMD_A3_COMBINED_4     0x0004000Bu
#define PMD_A3_DATA_42     0x0004000Du

#define PMD_A3_DISTANCE       0x00000001u
#define PMD_A3_AMPLITUDE      0x00000002u
#define PMD_A3_CONFIDENCE     0x00000004u
#define PMD_A3_COORDINATES    0x00000008u
#define PMD_A3_OBJECT_LIST    0x00000010u

#define PMD_O3D_DATA_1_7      0x00050003u
#define PMD_O3D_DATA_1_13      0x00050004u
#define PMD_O3_DATA_4040     0x00050005u
#define PMD_S3_DATA_4040     0x00050005u
#define PMD_O3NT_DATA_PRE      0x00050005u
#define PMD_O3D_DISTANCE       0x00000002u
#define PMD_O3D_INTENSITY     0x00000004u
#define PMD_O3D_STDDEV    0x00000010u
#define PMD_O3D_VIEW_X    0x00000020u
#define PMD_O3D_VIEW_Y    0x00000040u
#define PMD_O3D_VIEW_Z    0x00000080u
#define PMD_O3D_X    0x00000100u
#define PMD_O3D_Y    0x00000200u
#define PMD_O3D_Z    0x00000400u

#define PMD_CAMCUBE_DATA_1_0  0x00060001u
#define PMD_CAMCUBE_DATA_2_0  0x00060002u

#define PMD_USER_DEFINED_0   0xFFFF0000u
#define PMD_USER_DEFINED_1   0xFFFF0001u
#define PMD_USER_DEFINED_2   0xFFFF0002u
#define PMD_USER_DEFINED_3   0xFFFF0003u
#define PMD_USER_DEFINED_4   0xFFFF0004u
#define PMD_USER_DEFINED_5   0xFFFF0005u
#define PMD_USER_DEFINED_6   0xFFFF0006u
#define PMD_USER_DEFINED_7   0xFFFF0007u
#define PMD_USER_DEFINED_8   0xFFFF0008u
#define PMD_USER_DEFINED_9   0xFFFF0009u
#define PMD_USER_DEFINED_10   0xFFFF000au
#define PMD_USER_DEFINED_11   0xFFFF000bu
#define PMD_USER_DEFINED_12   0xFFFF000cu
#define PMD_USER_DEFINED_13   0xFFFF000du
#define PMD_USER_DEFINED_14   0xFFFF000eu
#define PMD_USER_DEFINED_15   0xFFFF000fu

#define PMD_ORIGIN_TOP_RIGHT       0x00000000u
#define PMD_ORIGIN_TOP_LEFT        0x00000001u
#define PMD_ORIGIN_BOTTOM_RIGHT    0x00000002u
#define PMD_ORIGIN_BOTTOM_LEFT     0x00000003u

#define PMD_ORIGIN_RIGHT           0x00000000u
#define PMD_ORIGIN_LEFT            0x00000001u
#define PMD_ORIGIN_TOP             0x00000000u
#define PMD_ORIGIN_BOTTOM          0x00000002u

#define PMD_DIRECTION_HORIZONTAL   0x00000000u
#define PMD_DIRECTION_VERTICAL     0x00010000u

#define PMD_GENERIC_DATA           0x00000001u
#define PMD_IMAGE_DATA             0x00000002u

#define PMD_UNKNOWN_SIZE           0x00000000u

#define PMD_FLAG_HIDE_PIXEL        0x00000001u

#define PMD_FLAG_INVALID           0x00000001u
#define PMD_FLAG_SATURATED         0x00000002u
#define PMD_FLAG_INCONSISTENT      0x00000004u
#define PMD_FLAG_LOW_SIGNAL        0x00000008u
#define PMD_FLAG_SBI_ACTIVE        0x00000010u

struct PMDGenericData
{
  unsigned subType;
  unsigned numElem;
  unsigned sizeOfElem;
};

struct PMDImageData
{
  unsigned subType;
  unsigned numColumns;
  unsigned numRows;
  unsigned numSubImages;

  int integrationTime[4];
  int modulationFrequency[4];
  int offset[4];

  int pixelAspectRatio;
  int pixelOrigin;


  unsigned timeStampHi;
  unsigned timeStampLo;

  char reserved[24];

  unsigned userData0;
};

struct PMDDataDescription
{
  unsigned PID;
  unsigned DID;
  unsigned type;
  unsigned size;

  unsigned subHeaderType;
  union
    {
      struct PMDGenericData gen;
#ifndef PMD_NO_DEPRECATED
      struct PMDImageData std;
#endif
      struct PMDImageData img;
      char fillUpToSizeOfStructure[108];
    }
#ifdef PMD_ANSI_C
 
#endif
       ;
};

#ifndef PMD_NO_DEPRECATED

typedef struct PMDDataDescription DataDescription;
typedef struct PMDImageData StandardPMDData;
typedef struct PMDGenericData GenericData;
#define GENERIC_DATA                  PMD_GENERIC_DATA
#define STANDARD_PMD_DATA             PMD_IMAGE_DATA

#define RAW_L16_AB        PMD_RAW_L16_AB        
#define RAW_L16_DAB       PMD_RAW_L16_DAB       
#define RAW_B16_DAB    PMD_RAW_B16_DAB    
#define RAW_L16_ABD       PMD_RAW_L16_ABD       
#define RAW_B16_ABD    PMD_RAW_B16_ABD    
#define RAW_P12_D         PMD_RAW_P12_D         
#define RAW_L16_D         PMD_RAW_L16_D         

#define UNKNOWN_DATA   PMD_UNKNOWN_DATA   

#define DISTANCE_LF32       PMD_DISTANCE_LF32       
#define AMPLITUDE_LF32      PMD_AMPLITUDE_LF32      
#define INTENSITY_LF32      PMD_INTENSITY_LF32      
#define REFLECTIVITY_LF32   PMD_REFLECTIVITY_LF32   
#define PMD_FLAGS_16        PMD_FLAGS_L16            
#define FLAGS_16            PMD_FLAGS_16            

#define X_COORD_LF32        PMD_X_COORD_LF32        
#define Y_COORD_LF32        PMD_Y_COORD_LF32        
#define Z_COORD_LF32        PMD_Z_COORD_LF32        

#define DISTANCE_I16_AMPLITUDE_I16 PMD_DISTANCE_I16_AMPLITUDE_I16 
#define DISTANCE_100UM_I16_AMPLITUDE_I16 PMD_DISTANCE_100UM_I16_AMPLITUDE_I16 

#define A3_DATA_2_31      PMD_A3_DATA_2_31      
#define A3_DATA_4         PMD_A3_DATA_4         
#define A3_RAWDATA_2_31   PMD_A3_RAWDATA_2_31   
#define A3_DISTANCE       PMD_A3_DISTANCE       
#define A3_AMPLITUDE      PMD_A3_AMPLITUDE      
#define A3_CONFIDENCE     PMD_A3_CONFIDENCE     
#define A3_COORDINATES    PMD_A3_COORDINATES    
#define A3_OBJECT_LIST    PMD_A3_OBJECT_LIST    

#define O3D_DATA_1_7      PMD_O3D_DATA_1_7      
#define O3D_DISTANCE       PMD_O3D_DISTANCE       
#define O3D_INTENSITY     PMD_O3D_INTENSITY     
#define O3D_STDDEV    PMD_O3D_STDDEV    
#define O3D_VIEW_X    PMD_O3D_VIEW_X    
#define O3D_VIEW_Y    PMD_O3D_VIEW_Y    
#define O3D_VIEW_Z    PMD_O3D_VIEW_Z    
#define O3D_X    PMD_O3D_X    
#define O3D_Y    PMD_O3D_Y    
#define O3D_Z    PMD_O3D_Z    

#define USER_DEFINED_0   PMD_USER_DEFINED_0   
#define USER_DEFINED_1   PMD_USER_DEFINED_1   
#define USER_DEFINED_2   PMD_USER_DEFINED_2   
#define USER_DEFINED_3   PMD_USER_DEFINED_3   
#define USER_DEFINED_4   PMD_USER_DEFINED_4   
#define USER_DEFINED_5   PMD_USER_DEFINED_5   
#define USER_DEFINED_6   PMD_USER_DEFINED_6   
#define USER_DEFINED_7   PMD_USER_DEFINED_7   
#define USER_DEFINED_8   PMD_USER_DEFINED_8   
#define USER_DEFINED_9   PMD_USER_DEFINED_9   
#define USER_DEFINED_10   PMD_USER_DEFINED_10   
#define USER_DEFINED_11   PMD_USER_DEFINED_11   
#define USER_DEFINED_12   PMD_USER_DEFINED_12   
#define USER_DEFINED_13   PMD_USER_DEFINED_13   
#define USER_DEFINED_14   PMD_USER_DEFINED_14   
#define USER_DEFINED_15   PMD_USER_DEFINED_15   

#define ORIGIN_TOP_RIGHT       PMD_ORIGIN_TOP_RIGHT       
#define ORIGIN_TOP_LEFT        PMD_ORIGIN_TOP_LEFT        
#define ORIGIN_BOTTOM_RIGHT    PMD_ORIGIN_BOTTOM_RIGHT    
#define ORIGIN_BOTTOM_LEFT     PMD_ORIGIN_BOTTOM_LEFT     

#define ORIGIN_RIGHT           PMD_ORIGIN_RIGHT           
#define ORIGIN_LEFT            PMD_ORIGIN_LEFT            
#define ORIGIN_TOP             PMD_ORIGIN_TOP             
#define ORIGIN_BOTTOM          PMD_ORIGIN_BOTTOM          

#define DIRECTION_HORIZONTAL   PMD_DIRECTION_HORIZONTAL   
#define DIRECTION_VERTICAL     PMD_DIRECTION_VERTICAL     

#define PMD_FLAG_SBI           PMD_FLAG_SBI_ACTIVE

#endif

END_EXTERN_C

/// @}

#endif 
