
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <limits>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

double read_double(string str) {
  try {
    return std::stod(str);
  } catch (...) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

void check_arguments(int argc, char* argv[], bool& verboseMode, double& std_a, double& std_yawdd, bool& dynamicProcesNoise, bool& reportStdDev, bool& useLaser, bool& useRadar) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt [-v] [-a std_a] [-y std_yawdd] [-d] [-r] [-s [lr]]\n";
  usage_instructions += "OPTIONAL ARGUMENTS:\n";
  usage_instructions += " [-v]           - verbose mode, print predicted and updated state after each sensor measurement\n";
  usage_instructions += " [-a std_a]     - override std_a noise parameter\n";
  usage_instructions += " [-y std_yawdd] - override std_yawdd noise parameter\n";
  usage_instructions += " [-d]           - adjust noise parameters dynamically\n";
  usage_instructions += " [-r]           - report std dev of estimated acceleration and yaw_dd\n";
  usage_instructions += " [-s [lr]]      - sensor types to use: l=>lidar, r=>radar; both can be specified, which is default mode\n";

  bool has_valid_args = false, sensorsSpecified = false;
  verboseMode = dynamicProcesNoise = reportStdDev = useLaser = useRadar = false;
  std_a = std_yawdd = std::numeric_limits<double>::quiet_NaN();

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc >= 4) {
    has_valid_args = true;
    for (auto i = 3; i < argc; ++i)
    {
      if (argv[i][0] == 'v') { verboseMode = true; continue; }
      if (argv[i][0] != '-') continue;
      switch (tolower(argv[i][1])) {
      case 'v': verboseMode = true; continue;
      case 'd': dynamicProcesNoise = true; continue;
      case 'r': reportStdDev = true; continue;
      case 'a':
        if ((i + 1) < argc) { std_a = read_double(argv[i + 1]); continue; }
        cerr << "Please specify value for std_a when -a is specified" << endl;
        has_valid_args = false;
        goto done;
      case 'y':
        if ((i + 1) < argc) { std_yawdd = read_double(argv[i + 1]); continue; }
        cerr << "Please specify value for std_yawdd when -y is specified" << endl;
        has_valid_args = false;
        goto done;
      case 's':
        if ((i + 1) < argc) {
          sensorsSpecified = true;
          for (auto ci = 0; argv[i + 1][ci]; ++ci) {
            if (towlower(argv[i + 1][ci]) == 'l') useLaser = true;
            if (towlower(argv[i + 1][ci]) == 'r') useRadar = true;
          }
          continue; 
        }
        cerr << "Please specify value for sensor type(s) to use when -s is specified" << endl;
        has_valid_args = false;
        goto done;
      }
    }
  }
  if (!sensorsSpecified) useLaser = useRadar = true;
done:
  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name, ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

void read_data(std::ifstream &in_file_, std::vector<MeasurementPackage> &measurement_pack_list, std::vector<GroundTruthPackage> &gt_pack_list) {
  string line;

  while (getline(in_file_, line)) { // each line represents a measurement at a timestamp
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {      // laser measurement
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      double px, py;
      iss >> px >> py >> timestamp;
      meas_package.raw_measurements_ << px, py;
      meas_package.timestamp_ = timestamp;
    } else if (sensor_type.compare("R") == 0) {  // radar measurement
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      double ro, phi, ro_dot;
      iss >> ro >> phi >> ro_dot >> timestamp;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      meas_package.timestamp_ = timestamp;
    }
    measurement_pack_list.push_back(meas_package);

    // read ground truth data to compare later
    double x_gt, y_gt, vx_gt, vy_gt;
    iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }
}

void update_nis_stats(VectorXd& NIS_tracker, double NIS)
{
  if (NIS < 0.35) NIS_tracker(0) += 1;
  else if (NIS<=7.81) NIS_tracker(1) += 1;
  else NIS_tracker(2) += 1;
}

int main(int argc, char* argv[]) {
  auto verboseMode = false; double std_a, std_yawdd; auto dynamicProcesNoise = false; auto reportStdDev = false; 
  bool useLaser, useRadar;
  check_arguments(argc, argv, verboseMode, std_a, std_yawdd, dynamicProcesNoise, reportStdDev, useLaser, useRadar);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;
  read_data(in_file_, measurement_pack_list, gt_pack_list);

  // Create a UKF instance
  UKF ukf{ verboseMode, std_a, std_yawdd, dynamicProcesNoise, useLaser, useRadar };

  // used to compute the RMSE later, recomended std_a and std_yawdd
  vector<VectorXd> process_state;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  auto NIS_tracker = VectorXd{ 3 }; // index 0 for #<0.35, index 1 for # between 0.35 and 7.81, index 2 for #>7.81
  NIS_tracker.fill(0.0);

  // start filtering from the second frame (the speed is unknown in the first frame)
  size_t number_of_measurements = measurement_pack_list.size();

  // column names for output file
  out_file_ << "time_stamp" << "\t";  
  out_file_ << "px_state" << "\t";
  out_file_ << "py_state" << "\t";
  out_file_ << "v_state" << "\t";
  out_file_ << "yaw_angle_state" << "\t";
  out_file_ << "yaw_rate_state" << "\t";
  out_file_ << "sensor_type" << "\t";
  out_file_ << "NIS" << "\t";  
  out_file_ << "px_measured" << "\t";
  out_file_ << "py_measured" << "\t";
  out_file_ << "px_ground_truth" << "\t";
  out_file_ << "py_ground_truth" << "\t";
  out_file_ << "vx_ground_truth" << "\t";
  out_file_ << "vy_ground_truth" << "\n";

  auto p0 = VectorXd{ 2 }; p0 << 0.0, 0.0;

  for (size_t k = 0; k < number_of_measurements; ++k) {
    // Call the UKF-based fusion
    ukf.ProcessMeasurement(measurement_pack_list[k]);

    // timestamp
    out_file_ << measurement_pack_list[k].timestamp_ << "\t"; // pos1 - est

    // output the state vector
    out_file_ << ukf.x_(0) << "\t"; // pos1 - est
    out_file_ << ukf.x_(1) << "\t"; // pos2 - est
    out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
    out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
    out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

    // output lidar and radar specific data
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // sensor type
      out_file_ << "lidar" << "\t";

      // NIS value
      out_file_ << ukf.NIS_laser_ << "\t";
      if (useLaser) update_nis_stats(NIS_tracker, ukf.NIS_laser_);

      // output the lidar sensor measurement px and py
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";

    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // sensor type
      out_file_ << "radar" << "\t";

      // NIS value
      out_file_ << ukf.NIS_radar_ << "\t";
      if (useRadar) update_nis_stats(NIS_tracker, ukf.NIS_radar_);

      // output radar measurement in cartesian coordinates
      auto ro = measurement_pack_list[k].raw_measurements_(0);
      auto phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t"; // px measurement
      out_file_ << ro * sin(phi) << "\t"; // py measurement
    }

    // output the ground truth
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

    // convert ukf x vector to cartesian to compare to ground truth
    VectorXd ukf_x_cartesian_ = VectorXd(4);

    auto x_estimate_ = ukf.x_(0);
    auto y_estimate_ = ukf.x_(1);
    auto vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
    auto vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
    
    ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
    
    // get accelaration and change of yaw_rate (yawdd)
    if (k > 0) {
      auto dt = (measurement_pack_list[k].timestamp_ - measurement_pack_list[k-1].timestamp_) / 1000000.0;
      if (dt != 0.0) {
        auto accelaration = (ukf.x_(2) - p0(0)) / dt; auto yawdd = (ukf.x_(4) - p0(1)) / dt;
        auto a_yawdd = VectorXd{ 2 }; a_yawdd << accelaration, yawdd;
        process_state.push_back(a_yawdd);
      }
    }
    p0 << ukf.x_(2), ukf.x_(4); // store state to be used for acceleration and yawdd calcs
    estimations.push_back(ukf_x_cartesian_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }
  
  if (reportStdDev) {
    cout << "NIS stats: #" << NIS_tracker(0) << " < 0.35 < #" << NIS_tracker(1) << " < 7.81 < #" << NIS_tracker(2);
    cout << ": % within required range=" << 100.0*NIS_tracker(1) / NIS_tracker.sum() << "%" << endl;
    auto stdev = Tools::CalculateStdDev(process_state);
    cout << "stdev of accelaration & yaw rate change (yaw_dd): -a " << stdev(0) << " -y " << stdev(1) << endl;
  }
  // compute the accuracy (RMSE)
  VectorXd rmse{ Tools::CalculateRMSE(estimations, ground_truth) } ;
  cout << "RMSE" << endl;
  for (auto i = 0; i<rmse.size(); i++)
    cout << rmse[i] << endl;

  // close files
  if (out_file_.is_open()) out_file_.close();
  if (in_file_.is_open()) in_file_.close();

  //cout << "Done!" << endl;
  return 0;
}
