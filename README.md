# liboptoforce

## Synopsis

This project provides a driver library and basic command-line utilities for operating OptoForce force sensing devices under Linux. It has roughly been inspired by the Linux vendor driver provided by OptoForce Ltd., but generally represents a fundamental re-write of this driver.

**Author(s):** Ralf Kaestner, Mark Hoepflinger
<br/>
**Maintainer:** Ralf Kaestner <ralf.kaestner@gmail.com>
<br/>
**Licsense:** GNU Lesser General Public License (LGPL)

**Operating system(s):** Debian-based Linux
<br/>
**Package PPA:** ppa:ethz-asl/drivers

## Building

This project is based on the CMake build system with an open-source macro extension called ReMake.

### Preparing the build system

If you already have installed ReMake on your build system, you may skip this step. Otherwise, before attempting to build this project the traditional CMake way, you must install ReMake following any of the two options described below:

* From source:
  * Clone ReMake from https://github.com/kralf/remake.git into `REMAKE_PROJECT_DIR`
  * Install the ReMake build dependencies (see `REMAKE_PROJECT_DIR/CMakeLists.txt` for details)
  * Run `cmake REMAKE_PROJECT_DIR` in the build directory to configure the build
  * Build and install the project using `make package_install` (from package on Debian-based Linux only) or `make install`
* From binary packages (Debian-based Linux only):
  * Add the ReMake PPA to your APT sources by issuing `sudo add-apt-repository ppa:ethz-asl/build-essential` on the command line
  * Run `sudo apt-get update` to re-synchronize your package index files
  * Install the ReMake package through `sudo apt-get install remake`

### Building with CMake

Once ReMake is available on your build system, you may attempt to build this project the CMake way. Assuming that you have cloned the project sources into `PROJECT_DIR`, a typical out-of-source build might look like this:

* Create a build directory using `mkdir -p PROJECT_DIR/build`
* Switch into the build directoy by `cd PROJECT_DIR/build`
* Run `cmake PROJECT_DIR` in the build directory to configure the build
* If you want to inspect or modify the build configuration, issue `ccmake PROJECT_DIR`
* Build the project using `make`
* If you intend to install the project, call `make packages_install` (from package(s) on Debian-based Linux only) or `make install`

## Installing from packages

The maintainers of this project provide binary packages for the latest Ubuntu LTS releases and commonly used system architectures. To install these packages, you may follow these instructions:

  * Add the project PPA to your APT sources by issuing `sudo add-apt-repository PROJECT_PPA` on the command line (see synopsis for the project's package PPA)
  * Run `sudo apt-get update` to re-synchronize your package index files
  * Install all project packages through `sudo apt-get install PROJECT_NAME*` or selected packages using your favorite package management tool

## API documentation

This project generates its API documentation from source. To access it, you may either inspect the build directory tree after the project has been built using `make` or install the documentation package through `sudo apt-get install PROJECT_NAME-doc`.

## Feature requests and bug reports

If you would like to propose a feature for this project, please consider contributing or send a feature request to the project authors. Bugs may be reported through the project's issue page.
