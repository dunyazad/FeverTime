project(FeverTimeCuda LANGUAGES CXX CUDA)
enable_language(CUDA)

# CUDA 소스
file(GLOB_RECURSE CUDA_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cu"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cuh"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

# 정적 라이브러리 생성
add_library(FeverTimeCuda OBJECT ${CUDA_SOURCES})

# 인클루드 경로
target_include_directories(FeverTimeCuda PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/ext/nvapi510/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/ext/eigen
)

# 링크 경로
target_link_directories(FeverTimeCuda PRIVATE
    "${CMAKE_SOURCE_DIR}/ext/nvapi510/amd64"
    "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.4/lib/x64"  # ❗ 필요 시 본인의 CUDA 경로로 수정
)

# CUDA 속성 설정
set_target_properties(FeverTimeCuda PROPERTIES
    CUDA_STANDARD 17
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_ARCHITECTURES 86  # 또는 "native" / 사용 중인 GPU 아키텍처
)

target_compile_options(FeverTimeCuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--expt-extended-lambda>
)

# CUDA 런타임 라이브러리 링크
target_link_libraries(FeverTimeCuda PRIVATE
    cudart
    nvapi64
)
