#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <deque>

#include <assert.h>

#include "global.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "line_map.h"
#ifdef __cplusplus
}
#endif

#define CLF_MAX_LINE_LENGTH 1024

using namespace std;

/////////////////////////////////////////////////////////////////

template <class T>
struct point{
  carmen_inline point(){}
  carmen_inline point(T _x, T _y):x(_x),y(_y){}
  T x, y;
};

template <class T>
carmen_inline point<T> operator+(const point<T>& p1, const point<T>& p2){
  return point<T>(p1.x+p2.x, p1.y+p2.y);
}

template <class T>
carmen_inline point<T> operator - (const point<T> & p1, const point<T> & p2){
  return point<T>(p1.x-p2.x, p1.y-p2.y);
}

template <class T>
carmen_inline point<T> operator * (const point<T>& p, const T& v){
  return point<T>(p.x*v, p.y*v);
}

template <class T>
carmen_inline point<T> operator * (const T& v, const point<T>& p){
  return point<T>(p.x*v, p.y*v);
}

template <class T>
carmen_inline T operator * (const point<T>& p1, const point<T>& p2){
  return p1.x*p2.x+p1.y*p2.y;
}


template <class T, class A>
struct orientedpoint: public point<T>{
  carmen_inline orientedpoint(){}
  carmen_inline orientedpoint(const point<T>& p);
  carmen_inline orientedpoint(T x, T y, A _theta): point<T>(x,y),
theta(_theta){}
  A theta;
};

template <class T, class A>
orientedpoint<T,A>::orientedpoint(const point<T>& p){
  this->x=p.x;
  this->y=p.y;
  this->theta=0.;
}


template <class T, class A>
orientedpoint<T,A> operator+(const orientedpoint<T,A>& p1, const
orientedpoint<T,A>& p2){
  return orientedpoint<T,A>(p1.x+p2.x, p1.y+p2.y, p1.theta+p2.theta);
}

template <class T, class A>
orientedpoint<T,A> operator - (const orientedpoint<T,A> & p1, const
orientedpoint<T,A> & p2){
  return orientedpoint<T,A>(p1.x-p2.x, p1.y-p2.y, p1.theta-p2.theta);
}

template <class T, class A>
orientedpoint<T,A> operator * (const orientedpoint<T,A>& p, const T& v){
  return orientedpoint<T,A>(p.x*v, p.y*v, p.theta*v);
}

template <class T, class A>
orientedpoint<T,A> operator * (const T& v, const orientedpoint<T,A>& p){
  return orientedpoint<T,A>(p.x*v, p.y*v, p.theta*v);
}

template <class T>
struct pointcomparator{
  bool operator ()(const point<T>& a, const point<T>& b) const {
    return a.x<b.x || (a.x==b.x && a.y<b.y);
  }
};

typedef point<int> IntPoint;
typedef point<double> Point;
typedef orientedpoint<double, double> OrientedPoint;

/////////////////////////////////////////////////////////////////

class CLFRecord{
 public:
  CLFRecord();
  virtual ~CLFRecord();
  virtual void read(istream& is)=0;
  virtual void write(ostream& os)const =0 ;
  virtual string id() const{ return "NO-TYPE"; };

  unsigned int dim;
  double time;
};

/////////////////////////////////////////////////////////////////

class CLFCommentRecord: public CLFRecord {
 public:
  CLFCommentRecord();
  virtual void read(istream& is);
  virtual void write(ostream& os) const ;
  virtual string id() const{ return "COMMENT"; };
  string text;
};


/////////////////////////////////////////////////////////////////

class CLFTruePoseRecord: public CLFRecord{
 public:
  CLFTruePoseRecord();
  void read(istream& is);
  virtual void write(ostream& os)const ;
  virtual string id() const{ return "TRUEPOS"; };

  OrientedPoint truePose;
  OrientedPoint odomPose;
};

/////////////////////////////////////////////////////////////////

class CLFOdometryRecord: public CLFRecord{
 public:
  CLFOdometryRecord();
  virtual void read(istream& is);
  virtual void write(ostream& os)const ;
  virtual string id() const{ return "ODOM"; };

  OrientedPoint pose;
  double tv, rv, acceleration;
};

/////////////////////////////////////////////////////////////////

class CLFLaserRecord: public CLFRecord{
 public:
  CLFLaserRecord(int laserid);
  virtual void read(istream& is);
  virtual void write(ostream& os)const ;
  virtual string id() const;

  vector<double> readings;
  OrientedPoint laserPose;
  OrientedPoint odomPose;
  int laserID;
};

/////////////////////////////////////////////////////////////////

class CLFRecordList: public deque<CLFRecord*>{
 public:
  CLFRecordList(bool showDebug = false);
  virtual ~CLFRecordList() {};
  istream& read(istream& is);
  virtual CLFRecord* createRecord(string recordType);

  bool showDebug;
};

/////////////////////////////////////////////////////////////////

CLFRecord::CLFRecord(){
}

CLFRecord::~CLFRecord(){
}

/////////////////////////////////////////////////////////////////

CLFCommentRecord::CLFCommentRecord(){
}

void CLFCommentRecord::read(istream& is){
  char buf[CLF_MAX_LINE_LENGTH];
  memset(buf,0, CLF_MAX_LINE_LENGTH*sizeof(char));
  is.getline(buf, CLF_MAX_LINE_LENGTH);
  text=string(buf);
}

void CLFCommentRecord::write(ostream& os) const {
  os << "## " << text << endl;
}

/////////////////////////////////////////////////////////////////

CLFTruePoseRecord::CLFTruePoseRecord() {
};

void CLFTruePoseRecord::read(istream& is){
  is >> truePose.x >> truePose.y >> truePose.theta;
  is >> odomPose.x >> odomPose.y >> odomPose.theta;
  time = 0;
  if (is)
    is >> time;
}
void CLFTruePoseRecord::write(ostream& os) const {
  os << "TRUEPOS ";
  os << truePose.x << " " << truePose.y << " " << truePose.theta << " ";
  os << odomPose.x << " " << odomPose.y << " " << odomPose.theta << " " << time
<< " clfwrite " << time << endl;
}

/////////////////////////////////////////////////////////////////

CLFOdometryRecord:: CLFOdometryRecord() {
}

void CLFOdometryRecord::read(istream& is){
  is >> pose.x >> pose.y >> pose.theta;
  is >> tv >> rv >> acceleration;
  time = 0;
  if (is)
    is >> time;
}

void CLFOdometryRecord::write(ostream& os)const {
  os << "ODOM ";
  os << pose.x << " " << pose.y << " " << pose.theta << " ";
  os << tv << " " << rv << " " << acceleration << " " << time << " clfwrite " <<
time << endl;
}

/////////////////////////////////////////////////////////////////

CLFLaserRecord::CLFLaserRecord(int laserid) {
  laserID = laserid;
}

void CLFLaserRecord::read(istream& is){
  is >> dim;
  for (unsigned int i=0; i< dim; i++){
    double r;
    is >> r;
    readings.push_back(r);
  }
  is >> laserPose.x;
  is >> laserPose.y;
  is >> laserPose.theta;
  is >> odomPose.x;
  is >> odomPose.y;
  is >> odomPose.theta;
  time = 0;
  if (is)
    is >> time;
}

void CLFLaserRecord::write(ostream& os)const {
  if (laserID == 1) {
    os << "FLASER " <<  dim;
  }
  else  if (laserID == 2) {
    os << "RLASER " <<  dim;
  }
  else  if (laserID == 3) {
    os << "LASER3 " <<  dim;
  }
  else  if (laserID == 4) {
    os << "LASER4 " <<  dim;
  }
  else  {
    os << "FLASER " <<  dim;
  }

  for (unsigned int i=0; i< dim; i++){
    os <<" "<< readings[i] ;
  }
  os <<" "<< laserPose.x;
  os <<" "<< laserPose.y;
  os <<" "<< laserPose.theta;
  os <<" "<< odomPose.x;
  os <<" "<< odomPose.y;
  os <<" "<< odomPose.theta;
  os <<" "<< time <<  " clfwrite " << time << endl;
};

string CLFLaserRecord::id() const {
  if (laserID == 1) {
    return "FLASER";
  }
  else  if (laserID == 2) {
    return "RLASER";
  }
  else  if (laserID == 3) {
    return "LASER3";
  }
  else  if (laserID == 4) {
    return "LASER4";
  }
  else  {
    return "FLASER";
  }
};

/////////////////////////////////////////////////////////////////

CLFRecordList::CLFRecordList(bool showDebug) {
  this->showDebug = showDebug;
}

CLFRecord* CLFRecordList::createRecord(string recordType) {
  CLFRecord* rec = NULL;

  if (recordType=="FLASER"){
    rec=new CLFLaserRecord(1);
    if (showDebug)
      cerr << "l" << flush;
  }
  else if (recordType=="RLASER"){
    rec=new CLFLaserRecord(2);
    if (showDebug)
      cerr << "r" << flush;
  }
  else if (recordType=="LASER3"){
    rec=new CLFLaserRecord(3);
    if (showDebug)
      cerr << "3" << flush;
  }
  else if (recordType=="LASER4"){
    rec=new CLFLaserRecord(4);
    if (showDebug)
      cerr << "4" << flush;
  }
  else if (recordType=="ODOM"){
    rec=new CLFOdometryRecord;
    if (showDebug)
      cerr << "o" << flush;
  }
  else if (recordType=="TRUEPOS"){
    rec=new CLFTruePoseRecord;
    if (showDebug)
      cerr << "t" << flush;
  }
  else if (recordType=="COMMENT"){
    rec=new CLFCommentRecord;
    if (showDebug)
      cerr << "c" << flush;
  }

  return rec;
}

istream& CLFRecordList::read(istream& is){

  char buf[CLF_MAX_LINE_LENGTH];
  while(is){
    buf[0]='\0';
    is.getline(buf, CLF_MAX_LINE_LENGTH);

    istringstream lineStream(buf);

    string recordType;
    lineStream >> recordType;

    CLFRecord* rec = createRecord(recordType);
    if (rec){
      rec->read(lineStream);
      push_back(rec);
    }
  }
  return is;
}

//////////////////////////////////////////////////////////

int main(int argc, char** argv) {

  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <carmen-log-file>" << endl;
    exit(0);
  }

  carmen_ipc_initialize(argc, argv);
  carmen_linemapping_init(argc, argv);

  cerr << "reading clf " << argv[argc-1] << " ...";
  ifstream log(argv[argc-1]);
  if (!log.is_open()) {
    cerr << "unable to open file!" << endl;
    exit(0);
  }
  CLFRecordList rec;
  rec.read(log);
  cerr << "done!" << endl;
  
  cerr << "collecting and dumping scans ...";

  int num_entries=(int) rec.size();
  int num_lasers = 0;
  for (int i=0; i<num_entries; i++) {
    if (rec[i]->id() == "FLASER") 
      num_lasers++;
  }

  int laser_cnt=0;
  carmen_robot_laser_message* lasers = new carmen_robot_laser_message[num_lasers];
  int i=0;
  while (i<num_entries) {

    if (rec[i]->id() == "FLASER") {
      const CLFLaserRecord* laser =  
	dynamic_cast<const CLFLaserRecord*> (rec[i]);
      assert(laser);

      carmen_robot_laser_message ltmp;
      ltmp.num_readings = (int) laser->readings.size();
      ltmp.range = new float[ltmp.num_readings];
      for (int j=0; j<ltmp.num_readings; j++) 
	ltmp.range[j] = laser->readings[j];
      ltmp.laser_pose.x      = laser->laserPose.x;
      ltmp.laser_pose.y      = laser->laserPose.y;
      ltmp.laser_pose.theta  = laser->laserPose.theta;
      ltmp.robot_pose.x      = laser->odomPose.x;
      ltmp.robot_pose.y      = laser->odomPose.y;      
      ltmp.robot_pose.theta  = laser->odomPose.theta;

      lasers[laser_cnt++] = ltmp;

      laser->write(cout);

    }

    i++;
  }
  cerr << "done!" << endl;


  cerr << "building linemap...";
  carmen_linemapping_segment_set_t linemap = 
    carmen_linemapping_get_segments_from_scans(lasers, laser_cnt);
  cerr << "done" << endl;

  cerr << "writing lines to log output...";
  
  for (int j=0; j<linemap.num_segs; j++) {
    cout << "MARKER [color=red; line=" << linemap.segs[j].p1.x << ", " << linemap.segs[j].p1.y << ", "
	 << linemap.segs[j].p2.x << ", " << linemap.segs[j].p2.y << "]" << endl;
    cout << "MARKER [color=blue; circle=" << linemap.segs[j].p1.x << ", " << linemap.segs[j].p1.y << ", "
	 << 0.1 << "]" << endl;
    cout << "MARKER [color=blue; circle=" << linemap.segs[j].p2.x << ", " << linemap.segs[j].p2.y << ", "
	 << 0.1 << "]" << endl;
  }

  cerr <<  "done!" << endl;


  return 0;
}
