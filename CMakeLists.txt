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

remake_pack_deb(COMPONENT core)
remake_pack_deb(COMPONENT interface)
remake_pack_deb()
remake_pack_deb(COMPONENT dev)
remake_pack_deb(COMPONENT data)
remake_pack_deb(COMPONENT doc)
