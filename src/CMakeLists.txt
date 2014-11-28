remake_find_package(
  Boost REQUIRED
  COMPONENTS system thread signals chrono program_options
  RESULT_VAR Boost_FOUND
)

remake_include(
  lib
  ${Boost_INCLUDE_DIRS}
)

remake_add_directories(lib)
remake_add_directories(bin COMPONENT utils)
remake_pkg_config_generate()
