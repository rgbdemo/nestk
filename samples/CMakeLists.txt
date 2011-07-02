ADD_EXECUTABLE(skeleton-tracking skeleton_tracking.cpp)
TARGET_LINK_LIBRARIES(skeleton-tracking nestk)

ADD_EXECUTABLE(grab-mesh grab_mesh.cpp)
TARGET_LINK_LIBRARIES(grab-mesh nestk)

ADD_EXECUTABLE(handpoint-tracking handpoint_tracking.cpp)
TARGET_LINK_LIBRARIES(handpoint-tracking nestk)

ADD_EXECUTABLE(libfreenect-backend libfreenect_backend.cpp)
TARGET_LINK_LIBRARIES(libfreenect-backend nestk)

ADD_EXECUTABLE(openni-backend openni_backend.cpp)
TARGET_LINK_LIBRARIES(openni-backend nestk)

ADD_EXECUTABLE(multiple-kinects multiple_kinects.cpp)
TARGET_LINK_LIBRARIES(multiple-kinects nestk)
