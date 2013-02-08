ADD_EXECUTABLE(apply-transform apply_transform.cpp)
TARGET_LINK_LIBRARIES(apply-transform nestk)

ADD_EXECUTABLE(sse sse.cpp)
TARGET_LINK_LIBRARIES(sse nestk)

ADD_EXECUTABLE(sse-normals sse_normals.cpp)
TARGET_LINK_LIBRARIES(sse-normals nestk)

IF(NESTK_USE_KIN4WIN)
   ADD_EXECUTABLE(kin4win-backend kin4win_backend.cpp)
   TARGET_LINK_LIBRARIES(kin4win-backend nestk)
ENDIF()

IF (NESTK_USE_OPENNI)
   IF (NESTK_USE_NITE)
       ADD_EXECUTABLE(skeleton-tracking skeleton_tracking.cpp)
       TARGET_LINK_LIBRARIES(skeleton-tracking nestk)

       ADD_EXECUTABLE(handpoint-tracking handpoint_tracking.cpp)
       TARGET_LINK_LIBRARIES(handpoint-tracking nestk)
   ENDIF()

   ADD_EXECUTABLE(grab-mesh grab_mesh.cpp)
   TARGET_LINK_LIBRARIES(grab-mesh nestk)

   ADD_EXECUTABLE(openni-backend openni_backend.cpp)
   TARGET_LINK_LIBRARIES(openni-backend nestk)

   ADD_EXECUTABLE(multiple-kinects multiple_kinects.cpp)
   TARGET_LINK_LIBRARIES(multiple-kinects nestk)

   ADD_EXECUTABLE(synchronized-multiple-kinects synchronized_multiple_kinects.cpp)
   TARGET_LINK_LIBRARIES(synchronized-multiple-kinects nestk)

   ADD_EXECUTABLE(markers markers.cpp)
   TARGET_LINK_LIBRARIES(markers nestk)

ENDIF(NESTK_USE_OPENNI)

IF (NESTK_USE_FREENECT)
   ADD_EXECUTABLE(libfreenect-backend libfreenect_backend.cpp)
   TARGET_LINK_LIBRARIES(libfreenect-backend nestk)
ENDIF()

IF (NESTK_USE_PMDSDK)
   ADD_EXECUTABLE(pmd-backend pmd_backend.cpp)
   TARGET_LINK_LIBRARIES(pmd-backend nestk)
ENDIF()

IF (NESTK_USE_SOFTKINETIC)
   ADD_EXECUTABLE(softkinetic-backend softkinetic_backend.cpp)
   TARGET_LINK_LIBRARIES(softkinetic-backend nestk)
ENDIF()

IF (NESTK_USE_SOFTKINETIC_IISU)
   ADD_EXECUTABLE(softkinetic-iisu-backend softkinetic_iisu_backend.cpp)
   TARGET_LINK_LIBRARIES(softkinetic-iisu-backend nestk)
ENDIF()

IF (NESTK_USE_PCL AND NESTK_USE_OPENNI)
    ADD_EXECUTABLE(pcl-compatibility pcl_compatibility.cpp)
    TARGET_LINK_LIBRARIES(pcl-compatibility nestk)

    ADD_EXECUTABLE(icp-pcl icp_pcl.cpp)
    TARGET_LINK_LIBRARIES(icp-pcl nestk)

    ADD_EXECUTABLE(align align.cpp)
    TARGET_LINK_LIBRARIES(align nestk)

    ADD_EXECUTABLE(table-top-object table_top_object.cpp)
    TARGET_LINK_LIBRARIES(table-top-object nestk)

    IF (NESTK_BUILD_OBJECT_DETECTION)
        ADD_EXECUTABLE(object-recognition object_recognition.cpp)
        TARGET_LINK_LIBRARIES(object-recognition nestk)
    ENDIF()
ENDIF()

IF (NESTK_USE_OPENNI AND NESTK_USE_PCL)
    ADD_EXECUTABLE(image-pose-estimator image_pose_estimator.cpp)
    TARGET_LINK_LIBRARIES(image-pose-estimator nestk)
ENDIF()
