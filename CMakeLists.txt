get_filename_component(UWB_ROOT "${CMAKE_CURRENT_LIST_DIR}/upstream" ABSOLUTE)

# FetchContent is only allowed during project generation, not in IDF script mode
if(NOT CMAKE_SCRIPT_MODE_FILE)
    include(FetchContent)

    FetchContent_Declare(
            decadriver
            GIT_REPOSITORY https://github.com/br101/dw3000-decadriver-source.git
            GIT_TAG        master
    )
    FetchContent_MakeAvailable(decadriver)

    set(DWT_DIR "${decadriver_SOURCE_DIR}/dwt_uwb_driver")
    set(DWT_PLATFORM_DIR "${decadriver_SOURCE_DIR}/platform")

    set(UWB_SRCS
            # Core Ranging & Math
            "${UWB_ROOT}/src/ranging/AoAEstimator.cpp"
            "${UWB_ROOT}/src/ranging/DistanceEstimator.cpp"
            "${UWB_ROOT}/src/ranging/HoppingCalculator.cpp"
            "${UWB_ROOT}/src/ranging/RangeConsensusFilter.cpp"

            # Crypto & Security
            "${UWB_ROOT}/src/crypto/MbedTlsCryptoProvider.cpp"
            "${UWB_ROOT}/src/crypto/CccKeyDerivationEngine.cpp"
            "${UWB_ROOT}/src/crypto/Sp0SecurityEngine.cpp"

            # Protocol & TLV Codecs
            "${UWB_ROOT}/src/protocol/FrameCodec.cpp"
            "${UWB_ROOT}/src/protocol/SetupMessageCodec.cpp"

            # Transceiver & Session
            "${UWB_ROOT}/src/transceiver/DW3000Controller.cpp"
            "${UWB_ROOT}/src/session/RangingSession.cpp"

            # Fetched Decawave Driver Sources
            "${DWT_DIR}/deca_interface.c"
            "${DWT_PLATFORM_DIR}/deca_compat.c"
            "${DWT_DIR}/deca_rsl.c"
            "${DWT_DIR}/dw3000/dw3000_device.c"
            "${DWT_DIR}/lib/qmath/src/qmath.c"
            deca_port.c
            ddk_dw3000_channel.cpp
    )
    set(UWB_INCL
            "${DWT_DIR}"
            "${DWT_DIR}/lib/qmath/include"
    )
    if (CONFIG_DW3000_CHIP_DW3000)
        list(APPEND UWB_SRCS "${DWT_DIR}/dw3000/dw3000_device.c")
        list(APPEND UWB_INCL "${DWT_DIR}/dw3000")
    elseif(CONFIG_DW3000_CHIP_DW3720)
        list(APPEND UWB_SRCS "${DWT_DIR}/dw3720/dw3720_device.c")
        list(APPEND UWB_INCL "${DWT_DIR}/dw3720")
    endif()

endif()

idf_component_register(
        SRCS ${UWB_SRCS}
        INCLUDE_DIRS
        "${UWB_ROOT}/include"
        "${UWB_INCL}"
        "${DWT_PLATFORM_DIR}"
        include
        PRIV_INCLUDE_DIRS
        "${UWB_ROOT}/src"
        REQUIRES
        esp_timer
        driver
        freertos
        esp_rom
        mbedtls
        DigitalDoorKey
)

if(NOT CMAKE_SCRIPT_MODE_FILE)
    target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_20)
    target_compile_options(${COMPONENT_LIB} PUBLIC "-std=gnu++20")

    set_source_files_properties(
            "${DWT_DIR}/deca_interface.c"
            "${DWT_DIR}/deca_rsl.c"
            "${DWT_DIR}/dw3000/dw3000_device.c"
            "${DWT_DIR}/lib/qmath/src/qmath.c"
            PROPERTIES COMPILE_FLAGS "-w"
    )
endif()