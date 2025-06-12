cmake_minimum_required(VERSION 3.16)
project(FeverTime LANGUAGES CXX CUDA)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_ARCHITECTURES "native")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# VTK 설정
set(VTK_DIR "${CMAKE_SOURCE_DIR}/ext/VTK/build")
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    IOGeometry
    IOPLY
    FiltersCore
    FiltersPoints
    FiltersSources
    RenderingCore
    RenderingOpenGL2
    InteractionStyle
    RenderingFreeType
    IOInfovis
)

IF (WIN32)
message(STATUS "HDIAPI_INCLUDE_DIRS: ${HDIAPI_INCLUDE_DIRS}")
# message(STATUS "HDIAPI_LIBRARIES: ${HDIAPI_LIBRARIES}")

set(HDIAPI_DIR "${CMAKE_SOURCE_DIR}/ext/hidapi/build/export_generated")
find_package(hidapi REQUIRED)
include_directories(${HDIAPI_INCLUDE_DIRS})
link_libraries(${HDIAPI_LIBRARIES})
ENDIF()

# CUDA 설정
enable_language(CUDA)

# 소스 수집 (CUDA 포함)
file(GLOB_RECURSE SOURCES "src/*.h" "src/*.hpp" "src/*.cpp" "src/*.cxx" "src/*.cuh" "src/*.cu")

source_group(TREE "${CMAKE_SOURCE_DIR}/src" FILES ${SOURCES})

# 실행 파일 생성
add_executable(FeverTime ${SOURCES})

# CUDA 옵션
set_target_properties(FeverTime PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_STANDARD 17
)

target_include_directories(FeverTime PRIVATE
    "${CMAKE_SOURCE_DIR}/ext/nvapi510/include"
    "${CMAKE_SOURCE_DIR}/ext/eigen"
    "${CMAKE_SOURCE_DIR}/ext/libwebsockets/include"
    "${CMAKE_SOURCE_DIR}/src"
)

# NVAPI 설정
target_link_directories(FeverTime PRIVATE
    "${CMAKE_SOURCE_DIR}/ext/nvapi510/amd64"
    "${CMAKE_SOURCE_DIR}/ext/libwebsockets/lib"
)

target_link_libraries(FeverTime PRIVATE
    ${VTK_LIBRARIES}
    nvapi64
    websockets_static
    hidapi::hidapi
)

# VTK 모듈 자동 초기화
vtk_module_autoinit(
  TARGETS FeverTime
  MODULES ${VTK_LIBRARIES}
)

# 공통 출력 디렉토리 설정
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

# 디버그/릴리즈 각각의 출력 디렉토리도 설정
set_target_properties(FeverTime PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/bin/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/bin/Release"
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/$<CONFIG>"
)

target_compile_options(FeverTime PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--expt-extended-lambda>
)
