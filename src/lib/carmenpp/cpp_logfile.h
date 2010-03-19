#ifndef CARMEN_CPP_LOGFILE_H
#define CARMEN_CPP_LOGFILE_H

#include <list>
#include <vector>

#include "cpp_global.h"

#include "cpp_abstractmessage.h"
#include "cpp_laser.h"
#include "cpp_base.h"
#include "cpp_robot.h"
#include "cpp_simulator.h"
#include "cpp_imu.h"
#include "cpp_unknownmessage.h"

#include "readlog.h"

typedef  std::vector<AbstractMessage*> Carmen_Cpp_LogFile_Collection;

class LogFile : public Carmen_Cpp_LogFile_Collection {
 public:
  LogFile();
  LogFile(const LogFile& x);
  LogFile(char* filename);
  virtual ~LogFile();

  bool load(char* filename, bool verbose = true);
  bool save(char* filename, bool verbose = true) const;

 public:
  typedef Carmen_Cpp_LogFile_Collection Collection;
};

#endif
