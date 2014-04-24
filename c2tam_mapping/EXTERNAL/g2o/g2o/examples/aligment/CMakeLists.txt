ADD_EXECUTABLE(aligment
  aligment.cpp
)

INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(aligment core solver_cholmod types_sba)


