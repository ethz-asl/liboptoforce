/*
 * ForceSensorBase.cpp
 *
 *  Created on: Aug. 5, 2013
 *      Author: markho
 */

#include"ForceSensorBase.hpp"

#include<iostream>


ForceSensorBase::ForceSensorBase(){
	name_="unknown";
	std::cout << "ForceSensorBase: Constructor called" << std::endl;
}

ForceSensorBase::~ForceSensorBase(){
}

std::string ForceSensorBase::name(){
  return name_;
}

