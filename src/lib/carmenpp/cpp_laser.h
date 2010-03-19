#ifndef CARMEN_CPP_LASER_MESSAGE_H
#define CARMEN_CPP_LASER_MESSAGE_H

#include "cpp_global.h"

#include "cpp_abstractmessage.h"
#include "cpp_point.h"

#include "laser_messages.h"

class LaserConfig {
 public:
  LaserConfig();
  LaserConfig(const LaserConfig& x);
  LaserConfig(const carmen_laser_laser_config_t& x);
  virtual ~LaserConfig();

  void setLaserConfig(const carmen_laser_laser_config_t& x);

  carmen_inline operator carmen_laser_laser_config_t() const {return m_Config;}

  DEFAULT_PARAM_SET_GET(carmen_laser_laser_config_t, Config);

  DEFAULT_STRUCT_PARAM_SET_GET(m_Config, carmen_laser_laser_type_t,       LaserType,  laser_type);
  DEFAULT_STRUCT_PARAM_SET_GET(m_Config, double,                          StartAngle, start_angle);
  DEFAULT_STRUCT_PARAM_SET_GET(m_Config, double,                          FOV, fov);
  DEFAULT_STRUCT_PARAM_SET_GET(m_Config, double,                          AngularResolution, angular_resolution);
  DEFAULT_STRUCT_PARAM_SET_GET(m_Config, double,                          MaximumRange, maximum_range);
  DEFAULT_STRUCT_PARAM_SET_GET(m_Config, double,                          Accuracy, accuracy);
  DEFAULT_STRUCT_PARAM_SET_GET(m_Config, carmen_laser_remission_type_t,   RemissionMode, remission_mode);
};

class LaserMessage : public AbstractMessage {
 public:
  LaserMessage(int num_readings=0, int num_remissions=0);
  LaserMessage(int num_readings, int num_remissions, const LaserConfig& cfg);
  LaserMessage(const LaserMessage& x);
  LaserMessage(const carmen_laser_laser_message& x);
  LaserMessage(carmen_laser_laser_message* x);
  LaserMessage(char* s);

  virtual ~LaserMessage();

  void init(int num_readings, int num_remissions);
  void initRange(int num_readings);
  void initRemission(int num_remission);
  void clone(const LaserMessage& x);
  void clone(const carmen_laser_laser_message& x);
  void setLaserMessage( carmen_laser_laser_message* x);
  void free();
  void freeRange();
  void freeRemission();

  float* getRange();
  float* getRemission();
  const float* getRange() const ;
  const float* getRemission() const ;

  int   getNumReadings() const;
  int   getNumRanges() const;
  float getRange(int i) const;
  void  setRange(int i, float val);

  int   getNumRemissions() const;
  float getRemission(int i) const;
  void  setRemission(int i, float val);

  Point endPoint(const OrientedPoint& robotPose, int beam) const;

  LaserConfig getConfig() const;
  void setConfig(const LaserConfig& x) ;
  void setConfig(const carmen_laser_laser_config_t& x) ;

  carmen_inline carmen_laser_laser_message* toCarmenMsg() {return m_msg;}
  carmen_inline operator carmen_laser_laser_message*() {return m_msg;}
  carmen_inline operator const carmen_laser_laser_message&() const {return *m_msg;}
  carmen_inline operator carmen_laser_laser_message&() {return *m_msg;}

  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, int, ID, id);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, char*, Host, host);
  STRUCT_PARAM_VIRTUAL_SET_GET(*m_msg, double, Timestamp, timestamp, public, public);

  virtual const char* getMessageID() const {
    return "RAWLASER";
  };

  virtual void save(carmen_FILE *logfile, double logger_timestamp);
  virtual char*  fromString(char* s);
  virtual AbstractMessage* clone() const;


 protected:
  carmen_laser_laser_message * m_msg;
};

#endif
