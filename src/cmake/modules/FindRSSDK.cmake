find_path(RSSDK_INCLUDES NAMES pxcversion.h PATHS "${RSSDK_DIR}/include")

# note that libpxcmd is used instead of libpxc, because the project is compiled as /MD
find_library(RSSDK_LIB_RELEASE NAMES libpxcmd.lib PATHS "${RSSDK_DIR}/lib/x64")
find_library(RSSDK_LIB_DEBUG NAMES libpxcmd_d.lib PATHS "${RSSDK_DIR}/lib/x64")
set(RSSDK_LIBS optimized ${RSSDK_LIB_RELEASE} debug ${RSSDK_LIB_DEBUG})
find_package_handle_standard_args(RSSDK REQUIRED_VARS RSSDK_LIBS RSSDK_INCLUDES)
