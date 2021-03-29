set(H_PREFIX include/CentroidalMPCWalking/System)

# set target name
add_centroidal_mpc_walking_library(
  NAME                   System
  PUBLIC_HEADERS         ${H_PREFIX}/Block.h ${H_PREFIX}/SharedResource.h ${H_PREFIX}/BlockRunner.h
  PRIVATE_HEADERS        ${H_PREFIX}/impl/traits.h
  IS_INTERFACE
  PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler Eigen3::Eigen
  SUBDIRECTORIES         tests
  )
