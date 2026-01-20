#include "config.h"
#include "filter.h"
#include <SPI.h>
#include <SD.h>
#include "states.h"

static File datafile;
static const char* LOG_FILE = "janesh.csv";

static bool logClosed = false;

static const char* modeToStr(FlightMode m) {
  switch (m) {
    case IDLE:    return "IDLE";
    case ASCENT:  return "ASCENT";
    case APOGEE:  return "APOGEE";
    case DESCENT: return "DESCENT";
    case LANDED:  return "LANDED";
    default:      return "UNKNOWN";
  }
}


bool initSD() {
  
  if (!SD.begin(SD_CS)) return false;

  File test = SD.open("sdtest.txt", FILE_WRITE);
  if (!test) return false;

  datafile = SD.open(LOG_FILE, FILE_WRITE);
  if (!datafile) return false;

  
  datafile.println("tms,mode,modeStr,acc_g,alt_m,vel_mps,maxAlt_m");
  datafile.flush();
  //Serial.println("sd fine");
  return true;
}



bool logData(const Data& d, const FlightSignals& s) {
  if (!datafile) return false;

  datafile.print(d.tms); datafile.print(',');
  datafile.print((int)d.mode); datafile.print(',');
  datafile.print(modeToStr(d.mode)); datafile.print(',');

  datafile.print(s.acc_g); datafile.print(',');
  datafile.print(s.alt_m); datafile.print(',');
  datafile.print(s.vel_mps); datafile.print(',');
  datafile.println(s.maxAlt_m);

  return true;
}
void closeLog() {
  if (datafile && !logClosed) {
    datafile.flush();
    datafile.close();
    logClosed = true;
  }
}

void flushLog() {
  if (datafile) {
    datafile.flush();
  }
}

