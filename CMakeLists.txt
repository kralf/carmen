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
)

remake_add_directories(data)

remake_pack_deb(
  COMPONENT core
  DESCRIPTION "core libraries"
  zlib1g
)
remake_pack_deb(
  COMPONENT interface
  DESCRIPTION "interface libraries"
  carmen-core
)
remake_pack_deb(
  carmen-interface
)
remake_pack_deb(
  COMPONENT cpp
  DESCRIPTION "C++ interface"
  carmen
)
remake_pack_deb(
  COMPONENT gui
  DESCRIPTION "GUI frontend"
  carmen libgtk2.0-0 libglib2.0-0 libart-2.0-2 libjpeg62 libmagickwand2
)
remake_pack_deb(
  COMPONENT test
  DESCRIPTION "testing binaries"
  carmen carmen-cpp
)

remake_pack_deb(
  COMPONENT core-dev
  DESCRIPTION "core development headers"
  carmen-core zlib1g-dev
)
remake_pack_deb(
  COMPONENT interface-dev
  DESCRIPTION "interface development headers"
  carmen-interface
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  carmen
)
remake_pack_deb(
  COMPONENT cpp-dev
  DESCRIPTION "C++ development headers"
  carmen-cpp
)
remake_pack_deb(
  COMPONENT gui-dev
  DESCRIPTION "GUI development headers"
  carmen-gui libgtk2.0-dev libglib2.0-dev libart-2.0-dev libjpeg62-dev
    libmagickwand-dev
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
