IF (NESTK_USE_OPENNI)
   ADD_EXECUTABLE(skeleton-tracking skeleton_tracking.cpp)
   TARGET_LINK_LIBRARIES(skeleton-tracking nestk)

   ADD_EXECUTABLE(grab-mesh grab_mesh.cpp)
   TARGET_LINK_LIBRARIES(grab-mesh nestk)

   ADD_EXECUTABLE(handpoint-tracking handpoint_tracking.cpp)
   TARGET_LINK_LIBRARIES(handpoint-tracking nestk)

   ADD_EXECUTABLE(openni-backend openni_backend.cpp)
   TARGET_LINK_LIBRARIES(openni-backend nestk)

   ADD_EXECUTABLE(multiple-kinects multiple_kinects.cpp)
   TARGET_LINK_LIBRARIES(multiple-kinects nestk)

   ADD_EXECUTABLE(synchronized-multiple-kinects synchronized_multiple_kinects.cpp)
   TARGET_LINK_LIBRARIES(synchronized-multiple-kinects nestk)
ENDIF(NESTK_USE_OPENNI)

IF (NESTK_USE_FREENECT)
   ADD_EXECUTABLE(libfreenect-backend libfreenect_backend.cpp)
   TARGET_LINK_LIBRARIES(libfreenect-backend nestk)
ENDIF()

IF (NESTK_USE_PCL)
    ADD_EXECUTABLE(pcl-compatibility pcl_compatibility.cpp)
    TARGET_LINK_LIBRARIES(pcl-compatibility nestk)

    ADD_EXECUTABLE(icp-pcl icp_pcl.cpp)
    TARGET_LINK_LIBRARIES(icp-pcl nestk)

    ADD_EXECUTABLE(table-top-object table_top_object.cpp)
    TARGET_LINK_LIBRARIES(table-top-object nestk)
ENDIF()
