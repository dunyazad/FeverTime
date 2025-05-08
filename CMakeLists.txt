cmake_minimum_required(VERSION 3.16)
project(FeverTime LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# VTK 설정
set(VTK_DIR "${CMAKE_SOURCE_DIR}/ext/VTK/build")
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    FiltersCore
    FiltersSources
    RenderingCore
    RenderingOpenGL2
    InteractionStyle
    RenderingFreeType
    IOInfovis
)

# CUDA 설정
enable_language(CUDA)

# 소스 수집 (CUDA 포함)
file(GLOB_RECURSE SOURCES "src/*.h" "src/*.hpp" "src/*.cpp" "src/*.cxx" "src/*.cuh" "src/*.cu")

# 실행 파일 생성
add_executable(FeverTime ${SOURCES})

# CUDA 옵션
set_target_properties(FeverTime PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_STANDARD 17
)

# VTK 연결
target_link_libraries(FeverTime PRIVATE ${VTK_LIBRARIES})

# VTK 모듈 자동 초기화
vtk_module_autoinit(
  TARGETS FeverTime
  MODULES ${VTK_LIBRARIES}
)

target_include_directories(FeverTime PRIVATE
    "${CMAKE_SOURCE_DIR}/ext/nvapi510/include"
    "${CMAKE_SOURCE_DIR}/ext/eigen"
    "${CMAKE_SOURCE_DIR}/src"
)

# NVAPI 설정
target_link_directories(FeverTime PRIVATE
    "${CMAKE_SOURCE_DIR}/ext/nvapi510/amd64"
)
target_link_libraries(FeverTime PRIVATE nvapi64)

# 공통 출력 디렉토리 설정
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

# 디버그/릴리즈 각각의 출력 디렉토리도 설정
set_target_properties(FeverTime PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/bin/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/bin/Release"
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/$<CONFIG>"
)
