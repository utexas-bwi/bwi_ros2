# Minimum CMake version for ROS 2
cmake_minimum_required(VERSION 3.5)
project(pr2_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Optional external tools
# find_program(CONVEX_DECOMPOSITION_CMD convex_decomposition)
# find_program(IVCON_CMD ivcon)

# Set C++ compiler flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Mesh file generation logic
file(GLOB pr2_stl_files ${CMAKE_CURRENT_SOURCE_DIR}/meshes/*/*.stl)
set(pr2_gen_files "")

# foreach(it ${pr2_stl_files})
#   get_filename_component(basepath ${it} PATH)
#   get_filename_component(basename ${it} NAME_WE)

#   if (${basename} MATCHES "_convex")
#     message("Ignoring stale _convex.stl file: ${basename}")
#   else ()
#     # Create necessary subdirectories
#     file(MAKE_DIRECTORY ${basepath}/convex)
#     file(MAKE_DIRECTORY ${basepath}/iv)

#     # Create OBJ files for convex decomposition from STL files
#     add_custom_command(
#       OUTPUT ${basepath}/convex/${basename}.obj
#       COMMAND ${IVCON_CMD} ${it} ${basepath}/convex/${basename}.obj
#       DEPENDS ${it})

#     set(pr2_gen_files ${pr2_gen_files} ${basepath}/convex/${basename}.obj)

#     # Convex decompose object files
#     add_custom_command(
#       OUTPUT ${basepath}/convex/${basename}_convex.obj
#       COMMAND ${CONVEX_DECOMPOSITION_CMD} ${basepath}/convex/${basename}.obj -v12 -p10
#       DEPENDS ${basepath}/convex/${basename}.obj)

#     set(pr2_gen_files ${pr2_gen_files} ${basepath}/convex/${basename}_convex.obj)

#     # Convert OBJ files back to binary STL (stlb)
#     add_custom_command(
#       OUTPUT ${basepath}/convex/${basename}_convex.stlb
#       COMMAND ${IVCON_CMD} ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.stlb
#       DEPENDS ${basepath}/convex/${basename}_convex.obj)

#     set(pr2_gen_files ${pr2_gen_files} ${basepath}/convex/${basename}_convex.stlb)

#     # Convert OBJ files back to ASCII STL (stla)
#     add_custom_command(
#       OUTPUT ${basepath}/convex/${basename}_convex.stla
#       COMMAND ${IVCON_CMD} ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.stla
#       DEPENDS ${basepath}/convex/${basename}_convex.obj)

#     set(pr2_gen_files ${pr2_gen_files} ${basepath}/convex/${basename}_convex.stla)

#     # Create IV files
#     add_custom_command(
#       OUTPUT ${basepath}/iv/${basename}.iv
#       COMMAND ${IVCON_CMD} ${it} ${basepath}/iv/${basename}.iv
#       DEPENDS ${it})

#     add_custom_command(
#       OUTPUT ${basepath}/convex/${basename}_convex.iv
#       COMMAND ${IVCON_CMD} ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.iv
#       DEPENDS ${basepath}/convex/${basename}_convex.obj)

#     set(pr2_gen_files ${pr2_gen_files} ${basepath}/iv/${basename}.iv ${basepath}/convex/${basename}_convex.iv)
#   endif ()
# endforeach(it)

# Add a custom target for media files
add_custom_target(media_files ALL DEPENDS ${pr2_gen_files})

# Install files
foreach(dir meshes gazebo materials robots documents urdf)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

# Export package
ament_package()
