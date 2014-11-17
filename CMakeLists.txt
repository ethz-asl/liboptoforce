cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  liboptoforce
  VERSION 0.1
  RELEASE beta
  SUMMARY "Driver library for OptoForce force sensing devices."
  AUTHOR "Mark Hoepflinger"
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/ethz-asl/liboptoforce"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX optoforce-
  INSTALL /usr
  HEADER_DESTINATION include/optoforce
)

remake_doc(
  html
)
