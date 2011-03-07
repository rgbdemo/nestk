set(${TINYXML_LIB_NAME}_sources tinyxml.cpp tinystr.cpp tinyxmlerror.cpp tinyxmlparser.cpp)

add_library(${TINYXML_LIB_NAME} STATIC ${${TINYXML_LIB_NAME}_sources})
