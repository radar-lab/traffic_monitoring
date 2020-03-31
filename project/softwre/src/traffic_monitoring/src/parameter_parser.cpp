#include "parameter_parser.h"

namespace traffic_monitoring {

PLUGINLIB_EXPORT_CLASS(traffic_monitoring::parameter_parser, nodelet::Nodelet);

parameter_parser::parameter_parser() {}

void parameter_parser::onInit() {}

void parameter_parser::params_parser(traffic_monitoring::mmWaveCLI &srv, ros::NodeHandle &nh) {

//   ROS_ERROR("%s",srv.request.comm.c_str());
//   ROS_ERROR("%s",srv.response.resp.c_str());
  std::vector <std::string> v;
  std::string s = srv.request.comm.c_str(); 
  std::istringstream ss(s);
  std::string token;
  std::string req;
  int i = 0;
  while (std::getline(ss, token, ' ')) {
    v.push_back(token);
    if (i > 0) {
      if (!req.compare("profileCfg")) {
        switch (i) {
          case 2:
            nh.setParam("/traffic_monitoring/startFreq", std::stof(token)); break;
          case 3:
            nh.setParam("/traffic_monitoring/idleTime", std::stof(token)); break;
          case 4:
            nh.setParam("/traffic_monitoring/adcStartTime", std::stof(token)); break;
          case 5:
            nh.setParam("/traffic_monitoring/rampEndTime", std::stof(token)); break;
          case 8:
            nh.setParam("/traffic_monitoring/freqSlopeConst", std::stof(token)); break;
          case 10:
            nh.setParam("/traffic_monitoring/numAdcSamples", std::stoi(token)); break;
          case 11:
            nh.setParam("/traffic_monitoring/digOutSampleRate", std::stof(token)); break;
          case 14:
            nh.setParam("/traffic_monitoring/rxGain", std::stof(token)); break;
        }
      } else if (!req.compare("frameCfg")) {
        switch (i) {
          case 1:
            nh.setParam("/traffic_monitoring/chirpStartIdx", std::stoi(token)); break;
          case 2:
            nh.setParam("/traffic_monitoring/chirpEndIdx", std::stoi(token)); break;
          case 3:
            nh.setParam("/traffic_monitoring/numLoops", std::stoi(token)); break;
          case 4:
            nh.setParam("/traffic_monitoring/numFrames", std::stoi(token)); break;
          case 5:
            nh.setParam("/traffic_monitoring/framePeriodicity", std::stof(token)); break;
        }
      }
    } else req = token;
    i++;
  }
}

void parameter_parser::cal_params(ros::NodeHandle &nh) {
  float c0 = 299792458;
  int chirpStartIdx;
  int chirpEndIdx;
  int numLoops;
  float framePeriodicity;
  float startFreq;
  float idleTime;
  float adcStartTime;
  float rampEndTime;
  float digOutSampleRate;
  float freqSlopeConst;
  float numAdcSamples;

  nh.getParam("/traffic_monitoring/startFreq", startFreq);
  nh.getParam("/traffic_monitoring/idleTime", idleTime);
  nh.getParam("/traffic_monitoring/adcStartTime", adcStartTime);
  nh.getParam("/traffic_monitoring/rampEndTime", rampEndTime);
  nh.getParam("/traffic_monitoring/digOutSampleRate", digOutSampleRate);
  nh.getParam("/traffic_monitoring/freqSlopeConst", freqSlopeConst);
  nh.getParam("/traffic_monitoring/numAdcSamples", numAdcSamples);

  nh.getParam("/traffic_monitoring/chirpStartIdx", chirpStartIdx);
  nh.getParam("/traffic_monitoring/chirpEndIdx", chirpEndIdx);
  nh.getParam("/traffic_monitoring/numLoops", numLoops);
  nh.getParam("/traffic_monitoring/framePeriodicity", framePeriodicity);

  int ntx = chirpEndIdx - chirpStartIdx + 1;
  int nd = numLoops;
  int nr = numAdcSamples;
  float tfr = framePeriodicity * 1e-3;
  float fs = digOutSampleRate * 1e3;
  float kf = freqSlopeConst * 1e12;
  float adc_duration = nr / fs;
  float BW = adc_duration * kf;
  float PRI = (idleTime + rampEndTime) * 1e-6;
  float fc = startFreq * 1e9 + kf * (adcStartTime * 1e-6 + adc_duration / 2); 
  float fc_chirp = startFreq * 1e9 + BW / 2; 

  float vrange = c0 / (2 * BW);
  float max_range = nr * vrange;
  float max_vel = c0 / (2 * fc * PRI) / ntx;
  float vvel = max_vel / nd;

  nh.setParam("/traffic_monitoring/num_TX", ntx);
  nh.setParam("/traffic_monitoring/f_s", fs);
  nh.setParam("/traffic_monitoring/f_c", fc);
  nh.setParam("/traffic_monitoring/fc_chirp", fc_chirp);
  nh.setParam("/traffic_monitoring/BW", BW);
  nh.setParam("/traffic_monitoring/PRI", PRI);
  nh.setParam("/traffic_monitoring/t_fr", tfr);
  nh.setParam("/traffic_monitoring/max_range", max_range);
  nh.setParam("/traffic_monitoring/range_resolution", vrange);
  nh.setParam("/traffic_monitoring/max_doppler_vel", max_vel);
  nh.setParam("/traffic_monitoring/doppler_vel_resolution", vvel);
}

}