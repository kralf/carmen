cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  CARMEN
  VERSION 0.7.4
  RELEASE beta
  SUMMARY "The Carnegie Mellon Robot Navigation Toolkit."
  AUTHOR "Mike Montemerlo"
  CONTACT "mmde@cs.cmu.edu"
  HOME "http://carmen.sourceforge.net"
  LICENSE "GNU General Public License (GPL)"
  INSTALL /usr
  EXTRA_SHARED_LINKER_FLAGS -Wl,--allow-shlib-undefined
)

remake_doc(
  man INSTALL share
  html
)
remake_add_directories(
  data
  COMPONENT data
)

remake_pack_deb(
  COMPONENT core
  DESCRIPTION "core libraries"
  DEPENDS zlib1g
)
remake_pack_deb(
  COMPONENT interface
  DESCRIPTION "interface libraries"
  DEPENDS carmen-core
)
remake_pack_deb(
  DEPENDS carmen-interface
)
remake_pack_deb(
  COMPONENT cpp
  DESCRIPTION "C++ interface"
  DEPENDS carmen
)
remake_pack_deb(
  COMPONENT gui
  DESCRIPTION "GUI frontend"
  DEPENDS carmen libgtk2.0-0 libglib2.0-0 libart-2.0-2 libjpeg[0-9]*
    libmagickwand[0-9]* libqt3-mt
)
remake_pack_deb(
  COMPONENT test
  DESCRIPTION "testing binaries"
  DEPENDS carmen carmen-cpp carmen-gui
)

remake_pack_deb(
  COMPONENT core-dev
  DESCRIPTION "core development headers"
  DEPENDS carmen-core zlib1g-dev
)
remake_pack_deb(
  COMPONENT interface-dev
  DESCRIPTION "interface development headers"
  DEPENDS carmen-interface carmen-core-dev
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  DEPENDS carmen carmen-interface-dev
)
remake_pack_deb(
  COMPONENT cpp-dev
  DESCRIPTION "C++ development headers"
  DEPENDS carmen-cpp carmen-dev
)
remake_pack_deb(
  COMPONENT gui-dev
  DESCRIPTION "GUI development headers"
  DEPENDS carmen-gui carmen-dev libgtk2.0-dev libglib2.0-dev
    libart-2.0-dev libjpeg[0-9][0-9]*-dev libmagickwand-dev
    libqt3-compat-headers libqt3-mt-dev
)

remake_pack_deb(
  ARCH all
  COMPONENT data
  DESCRIPTION "map data"
)

remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)
remake_pack_deb(
  ARCH all
  COMPONENT gui-doc
  DESCRIPTION "GUI documentation"
  DEPENDS carmen-doc
)
remake_pack_deb(
  ARCH all
  COMPONENT test-doc
  DESCRIPTION "testing documentation"
  DEPENDS carmen-doc
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS remake libgtk2.0-dev libglib2.0-dev libart-2.0-dev
    libjpeg62-dev libmagickwand-dev libqt3-mt-dev
    libqt3-compat-headers pkg-config doxygen
  PASS CMAKE_BUILD_TYPE CARMEN_SUBVERSION_REVISION
)
remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS remake libgtk2.0-dev libglib2.0-dev libart-2.0-dev
    libjpeg8-dev libmagickwand-dev libqt3-mt-dev libqt3-compat-headers
    pkg-config doxygen
  PASS CMAKE_BUILD_TYPE CARMEN_SUBVERSION_REVISION
)
