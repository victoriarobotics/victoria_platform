#ifndef ROS_PARAM_HELPER_h
#define ROS_PARAM_HELPER_h

#include <ros.h>

class RosParamHelper {

public:
  RosParamHelper(ros::NodeHandle nh);
  
  /**
   * Retrieves int parameter from ros and sets it
   * into the param.
   * 
   * Returns true if parameter was found in ros.
   */
  bool getParam(const char* name, int* param);

  /**
   * Retrieves int parameter from ros and sets it
   * into the param. If the parameter is not found, then
   * the defaultValue is set into the param.
   * 
   * Returns true if parameter was found in ros.
   */
  bool getParam(const char* name, int* param, int defaultValue);

  /**
   * Retrieves int parameter from ros and returns it.
   * If the parameter is not found, then the
   * defaultValue is returned.
   */
  int getParam(const char* name, int defaultValue);
  
  /**
   * Retrieves float parameter from ros and sets it
   * into the param.
   * 
   * Returns true if parameter was found in ros.
   */
  bool getParam(const char* name, float* param);

  /**
   * Retrieves float parameter from ros and sets it
   * into the param. If the parameter is not found, then
   * the default is set into the param.
   * 
   * Returns true if parameter was found in ros.
   */
  bool getParam(const char* name, float* param, float defaultValue);

  /**
   * Retrieves floag parameter from ros and returns it.
   * If the parameter is not found, then the
   * defaultValue is returned.
   */
  float getParam(const char* name, float defaultValue);
  
  /**
   * Retrieves char* parameter from ros and sets it
   * into the param.
   * 
   * Returns true if parameter was found in ros.
   */
  bool getParam(const char* name, char** param);

  /**
   * Retrieves char* parameter from ros and sets it
   * into the param. If the parameter is not found, then
   * the default is set into the param.
   * 
   * Returns true if parameter was found in ros.
   */
  bool getParam(const char* name, char** param, char* defaultValue);

private:
  ros::NodeHandle nh_;
};

#endif
