#include "RosParamHelper.h"

RosParamHelper::RosParamHelper(ros::NodeHandle nh) {
  nh_ = nh;
}

bool RosParamHelper::getParam(const char* name, int* param) {
  return nh_.getParam(name, param);
}

bool RosParamHelper::getParam(const char* name, int* param, int defaultValue) {
  if (nh_.getParam(name, param)) {
    return true;
  }
  (*param) = defaultValue;
  return false;
}

int RosParamHelper::getParam(const char* name, int defaultValue) {
  int paramValue;
  getParam(name, &paramValue, defaultValue);
  return paramValue;
}

bool RosParamHelper::getParam(const char* name, float* param) {
  return nh_.getParam(name, param);
}

bool RosParamHelper::getParam(const char* name, float* param, float defaultValue) {
  if (nh_.getParam(name, param)) {
    return true;
  }
  (*param) = defaultValue;
  return false;
}

float RosParamHelper::getParam(const char* name, float defaultValue) {
  float paramValue;
  getParam(name, &paramValue, defaultValue);
  return paramValue;
}

bool RosParamHelper::getParam(const char* name, char** param) {
  return nh_.getParam(name, param);
}

bool RosParamHelper::getParam(const char* name, char** param, char* defaultValue) {
  if (nh_.getParam(name, param)) {
    return true;
  }
  strcpy((*param), defaultValue);
  return false;
}

