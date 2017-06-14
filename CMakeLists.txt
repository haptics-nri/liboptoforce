cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  liboptoforce
  VERSION 0.2
  RELEASE beta
  SUMMARY "Driver library for OptoForce force sensing devices."
  AUTHOR "Ralf Kaestner"
  AUTHOR "Mark Hoepflinger"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/ethz-asl/liboptoforce"
  LICENSE "GNU Lesser General Public License (LGPL)"
  README README.md
  PREFIX optoforce-
  INSTALL /usr
  HEADER_DESTINATION include/optoforce
  EXTRA_CXX_FLAGS -std=c++0x
)

remake_doc(
  html
)

remake_pack_deb(
  DEPENDS libboost-system1.58.0 libboost-thread1.58.0
    libboost-signals1.58.0 libboost-chrono1.58.0
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  DEPENDS liboptoforce
    libboost-system1.58-dev
    libboost-thread1.58-dev
    libboost-signals1.58-dev
)
remake_pack_deb(
  COMPONENT utils
  DESCRIPTION "utilities"
  DEPENDS liboptoforce libboost-program-options1.58.0
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)

remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libboost-system1.58-dev libboost-thread1.58-dev
    libboost-signals1.58-dev libboost-chrono1.58-dev
    libboost-program-options1.58-dev remake doxygen
    pkg-config
  PASS CMAKE_BUILD_TYPE LIBOPTOFORCE_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION trusty
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libboost-system-dev libboost-thread-dev libboost-signals-dev
    libboost-chrono-dev libboost-program-options-dev remake doxygen
    pkg-config
  PASS CMAKE_BUILD_TYPE LIBOPTOFORCE_GIT_REVISION
)
