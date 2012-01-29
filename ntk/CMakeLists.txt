#
# Root nestk
#
#   nicolas.burrus@ensta.fr

SET ( ntk_sources

     core.h
     ntk.h
     aruco/aruco.h
     aruco/board.cpp
     aruco/boarddetector.cpp
     aruco/boarddetector.h
     aruco/board.h
     aruco/cameraparameters.cpp
     aruco/cameraparameters.h
     aruco/cvdrawingutils.cpp
     aruco/cvdrawingutils.h
     aruco/marker.cpp
     aruco/markerdetector.cpp
     aruco/markerdetector.h
     aruco/marker.h
     camera/calibration.h
     camera/calibration.cpp
     camera/file_grabber.h
     camera/file_grabber.cpp
     camera/multiple_grabber.h
     camera/multiple_grabber.cpp
     camera/opencv_grabber.h
     camera/opencv_grabber.cpp
     camera/rgbd_frame_recorder.h
     camera/rgbd_frame_recorder.cpp
     camera/rgbd_grabber.h
     camera/rgbd_grabber.cpp
     camera/rgbd_image.h
     camera/rgbd_image.cpp
     camera/rgbd_processor.h
     camera/rgbd_processor.cpp
     projector/calibration.h
     projector/calibration.cpp
     geometry/affine_transform.h
     geometry/affine_transform.cpp
     geometry/eigen_utils.h
     geometry/incremental_pose_estimator.h
     geometry/incremental_pose_estimator.cpp
     geometry/incremental_pose_estimator_from_rgb_features.h
     geometry/incremental_pose_estimator_from_rgb_features.cpp
     geometry/incremental_pose_estimator_from_markers.h
     geometry/incremental_pose_estimator_from_markers.cpp
     geometry/plane.h
     geometry/plane.cpp
     geometry/polygon.h
     geometry/polygon.cpp
     geometry/pose_3d.h
     geometry/pose_3d.cpp
     geometry/relative_pose_estimator.h
     geometry/relative_pose_estimator.cpp
     geometry/relative_pose_estimator_from_image.h
     geometry/relative_pose_estimator_from_image.cpp
     geometry/relative_pose_estimator_markers.h
     geometry/relative_pose_estimator_markers.cpp
     geometry/similarity_transform.h
     geometry/similarity_transform.cpp
     gui/image_widget.h
     gui/image_widget.cpp
     gui/screen_grabber.h
     gui/screen_grabber.cpp
     image/bilateral_filter.h
     image/bilateral_filter.cpp
     image/color_model.h
     image/color_model.cpp
     image/feature.h
     image/feature.cpp
     image/sift.h
     image/sift.cpp
     image/sift_gpu.h
     image/sift_gpu.cpp
     image/siftgpu/SiftGPU.h
     image/siftgpu/SiftGPU.cpp
     image/siftgpu/FrameBufferObject.cpp
     image/siftgpu/FrameBufferObject.h
     image/siftgpu/GlobalUtil.cpp
     image/siftgpu/GlobalUtil.h
     image/siftgpu/GLTexImage.cpp
     image/siftgpu/GLTexImage.h
     image/siftgpu/LiteWindow.h
     image/siftgpu/ProgramGLSL.cpp
     image/siftgpu/ProgramGLSL.h
     image/siftgpu/ProgramGPU.cpp
     image/siftgpu/ProgramGPU.h
     image/siftgpu/PyramidGL.cpp
     image/siftgpu/PyramidGL.h
     image/siftgpu/ShaderMan.cpp
     image/siftgpu/ShaderMan.h
     image/siftgpu/SiftPyramid.cpp
     image/siftgpu/SiftPyramid.h
     mesh/ply.h
     mesh/ply.cpp
     mesh/mesh.h
     mesh/mesh.cpp
     mesh/mesh_generator.h
     mesh/mesh_generator.cpp
     mesh/mesh_renderer.h
     mesh/mesh_renderer.cpp
     mesh/mesh_viewer.h
     mesh/mesh_viewer.cpp
     mesh/plane_remover.h
     mesh/plane_remover.cpp
     mesh/rgbd_modeler.h
     mesh/rgbd_modeler.cpp
     mesh/surfels_rgbd_modeler.h
     mesh/surfels_rgbd_modeler.cpp
     numeric/cost_function.h
     numeric/cost_function.cpp
     numeric/levenberg_marquart_minimizer.h
     numeric/levenberg_marquart_minimizer.cpp
     numeric/cost_function.h
     numeric/utils.h
     numeric/utils.cpp
     numeric/differential_evolution_solver.h
     numeric/differential_evolution_solver.cpp
     numeric/differential_evolution_minimizer.h
     numeric/differential_evolution_minimizer.cpp
     stats/distributions.h
     stats/distributions.cpp
     stats/estimation.cpp
     stats/estimation.h
     stats/histogram.h
     stats/histogram.cpp
     stats/hypothesis_testing.cpp
     stats/hypothesis_testing.h
     stats/hypothesis_testing.hxx
     stats/moments.h
     stats/moments.cpp
     stats/moments.hxx
     thread/event.h
     thread/event.cpp
     thread/utils.h
     thread/utils.cpp
     utils/arg.h
     utils/arg.cpp
     utils/common.h
     utils/debug.h
     utils/debug.cpp
     utils/opencv_utils.h
     utils/opencv_utils.cpp
     utils/progressive.h
     utils/progressive.cpp
     utils/qt_utils.h
     utils/qt_utils.cpp
     utils/stl.h
     utils/serializable.h
     utils/time.h
     utils/xml_parser.h
     utils/xml_parser.cpp
     utils/xml_serializable.h
     utils/xml_serializable.cpp
)

IF (NESTK_BUILD_OBJECT_DETECTION)
  SET ( ntk_sources ${ntk_sources}
     detection/object/feature.cpp
     detection/object/feature.h
     detection/object/feature_indexer.cpp
     detection/object/feature_indexer.h
     detection/object/feature_indexer_kdt.cpp
     detection/object/feature_indexer_kdt.h
     detection/object/located_feature.cpp
     detection/object/located_feature.h
     detection/object/object_database.cpp
     detection/object/object_database.h
     detection/object/object_detector.cpp
     detection/object/object_detector.h
     detection/object/object_finder.cpp
     detection/object/object_finder.h
     detection/object/object_match.cpp
     detection/object/object_match.h
     detection/object/object_pose.cpp
     detection/object/object_pose.h
     detection/object/object_tracker.cpp
     detection/object/object_tracker.h
     detection/object/pose_2d.cpp
     detection/object/pose_2d.h
     detection/object/sift_hough.cpp
     detection/object/sift_hough.h
     detection/object/sift_object_detector.cpp
     detection/object/sift_object_detector.h
     detection/object/sift_object_detector_lowe.cpp
     detection/object/sift_object_detector_lowe.h
     detection/object/sift_object_match.cpp
     detection/object/sift_object_match.h
     detection/object/sift_object_match_lowe.cpp
     detection/object/sift_object_match_lowe.h
     detection/object/sift_object_pose_estimator.cpp
     detection/object/sift_object_pose_estimator.h
     detection/object/sift_parameters.cpp
     detection/object/sift_parameters.h
     detection/object/sift_point_match.cpp
     detection/object/sift_point_match.h
     detection/object/task_pool.cpp
     detection/object/task_pool.h
     detection/object/vfh_object_detector.cpp
     detection/object/vfh_object_detector.h
     detection/object/visual_object.cpp
     detection/object/visual_object.h
     detection/object/visual_object_view.cpp
     detection/object/visual_object_view.h
  )
ENDIF()

QT4_WRAP_CPP(moc_sources
  gui/image_widget.h
  mesh/mesh_viewer.h
  utils/progressive.h
  thread/event.h  
)

# Workaround a bug with i386 on Mac that cannot compile this particular
# file, error is:
# non-relocatable subtraction expression, "LC36" minus "L00000000103$pb"
IF(APPLE AND (CMAKE_OSX_ARCHITECTURES MATCHES "i386"))
  SET_SOURCE_FILES_PROPERTIES(numeric/levenberg_marquart_minimizer.cpp
    PROPERTIES COMPILE_FLAGS "-O0")
ENDIF()

IF (NESTK_USE_OPENNI)
  SET (ntk_sources ${ntk_sources}
       camera/openni_grabber.h
       camera/openni_grabber.cpp
       camera/openni_grabber_internals.hxx
       gesture/body_event.h
       gesture/body_event.cpp
       gesture/ni_utils.h
       gesture/skeleton.h
       gesture/skeleton.cpp
      )
ENDIF()

IF (NESTK_USE_FREENECT)
  SET (ntk_sources ${ntk_sources}
       camera/freenect_grabber.h
       camera/freenect_grabber.cpp)
ENDIF()

IF(NESTK_USE_PCL)
    SET (ntk_sources ${ntk_sources}
         detection/plane_estimator.h
         detection/plane_estimator.cpp
         detection/table_object_detector.h
         detection/table_object_detector.hpp
     	 geometry/relative_pose_estimator_icp.h
     	 geometry/relative_pose_estimator_icp.cpp
         geometry/relative_pose_estimator_tabletop.h
         geometry/relative_pose_estimator_tabletop.cpp
         mesh/table_object_rgbd_modeler.h
         mesh/table_object_rgbd_modeler.cpp
         mesh/pcl_utils.h
         mesh/pcl_utils.hpp
         mesh/pcl_utils.cpp
    )
ENDIF()

IF (NESTK_USE_OPENCL)
  SET (ntk_sources ${ntk_sources}
       gpu/opencl.cpp
       gpu/opencl.h
   )
ENDIF()

IF(NESTK_USE_PCL)
    SET (ntk_sources ${ntk_sources}
         mesh/pcl_utils.h
         mesh/pcl_utils.cpp
        )
ENDIF(NESTK_USE_PCL)

IF (NESTK_USE_ROS)
  SET (ntk_sources ${ntk_sources}
     ros/ros_rgbd_grabber.h
     ros/ros_rgbd_grabber.cpp)
ENDIF(NESTK_USE_ROS)

IF (IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/private")
   SET(HAVE_NESTK_PRIVATE 1)
ENDIF()

IF (WIN32 OR APPLE)
  ADD_LIBRARY(nestk STATIC ${ntk_sources} ${moc_sources})
ELSE (WIN32 OR APPLE)
  ADD_LIBRARY(nestk SHARED ${ntk_sources} ${moc_sources})
ENDIF (WIN32 OR APPLE)

IF (WIN32)
  set(WINMM_LIBRARY winmm)
ENDIF()

TARGET_LINK_LIBRARIES(nestk
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
  ${GSL_LIBRARIES}
  ${OPENCL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLUT_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${X11_LIBRARY}
  ${LIBUSB_1_LIBRARY}
  ${PMD_LIBRARIES}
  ${FREENECT_LIBRARIES}
  ${OPENNI_LIBRARIES}
  ${NITE_LIBRARY}
  ${WINMM_LIBRARY}
  ${PTHREAD_LIBRARY})

SET_TARGET_PROPERTIES(nestk PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}"
  RUNTIME_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
)

# FIXME: on Mac, the dylib rpath points to current directory.
if(APPLE AND NOT "${EXECUTABLE_OUTPUT_PATH}" STREQUAL "" AND NOT "${LIBRARY_OUTPUT_PATH}" STREQUAL "")
    add_custom_command(TARGET nestk POST_BUILD
        COMMAND cp ${LIBRARY_OUTPUT_PATH}/*.dylib ${EXECUTABLE_OUTPUT_PATH}/
    )
endif()

INSTALL(TARGETS nestk DESTINATION lib)
NESTK_INSTALL_NOBASE_HEADER_FILES(include/ntk ${ntk_sources})
