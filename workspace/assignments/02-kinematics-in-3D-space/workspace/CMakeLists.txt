CMAKE_MINIMUM_REQUIRED (VERSION 3.5)

PROJECT (THREE_DIMENSIONAL_MACHINERY_WITH_EIGEN)

SET(EIGEN3_INCLUDE_DIR "Eigen" )

INCLUDE_DIRECTORIES(
    ${EIGEN3_INCLUDE_DIR}
)

# solve linear equations using Eigen
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/01-solve-linear-equations)
ADD_EXECUTABLE (solve-linear-equations 01-solve-linear-equations/solve-linear-equations.cpp)

# coordinate transform using Eigen
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/02-coordinate-transform)
ADD_EXECUTABLE (coordinate-transform 02-coordinate-transform/coordinate-transform.cpp)