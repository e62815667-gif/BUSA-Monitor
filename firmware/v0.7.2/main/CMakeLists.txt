set(UI_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${UI_ROOT}/index.html" "${UI_ROOT}/lab-core.js" "${UI_ROOT}/lab-app.js" "${UI_ROOT}/tools/embed_ui.py")
execute_process(COMMAND python3 "${UI_ROOT}/tools/embed_ui.py" "${CMAKE_CURRENT_BINARY_DIR}/LabPage.h"
    COMMAND_ERROR_IS_FATAL ANY)
idf_component_register(SRCS "LabMain.cpp" INCLUDE_DIRS "." "${CMAKE_CURRENT_BINARY_DIR}"
    REQUIRES espressif__arduino-esp32 json nvs_flash)
