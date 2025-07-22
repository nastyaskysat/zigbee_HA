idf_component_register(
    SRC_DIRS  "." "${PROJECT_DIR}/common/zcl_utility/src"
    INCLUDE_DIRS "." "${PROJECT_DIR}/common/zcl_utility/include"
    # REQUIRES light_driver
)
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/esp-zigbee-lib)