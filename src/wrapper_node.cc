#include "rclcpp/rclcpp.hpp"

#include <opencv2/calib3d/calib3d.hpp>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"
#include "mynteye/device/context.h"
#include "mynteye/device/device.h"


MYNTEYE_BEGIN_NAMESPACE

inline double compute_time(const double end, const double start) {
  return end - start;
}

class MynteyeWrapper: public rclcpp::Node {
public:
  MynteyeWrapper()
  : Node("mynteye_wrapper") {

    initDevice();

    if (api_ == nullptr) {
      RCLCPP_FATAL(this->get_logger(), "No MYNT EYE device selected :(");
      throw std::runtime_error("No MYNT EYE device selected :(");
    }

    pthread_mutex_init(&mutex_data_, nullptr);

    // node params
    std::map<Stream, std::string> stream_names{
        {Stream::LEFT, "left"},
        {Stream::RIGHT, "right"},
        {Stream::LEFT_RECTIFIED, "left_rect"},
        {Stream::RIGHT_RECTIFIED, "right_rect"},
        {Stream::DISPARITY, "disparity"},
        {Stream::DISPARITY_NORMALIZED, "disparity_norm"},
        {Stream::DEPTH, "depth"},
        {Stream::POINTS, "points"}
    };

    std::map<Stream, std::string> stream_topics;
    for (auto &&it : stream_names) {
      // Declare the parameter with default value
      this->declare_parameter<std::string>(it.second + "_topic", it.second);

      // Get the parameter value
      std::string topic_name;
      this->get_parameter(it.second + "_topic", topic_name);
      // Store in the topic map
      stream_topics[it.first] = topic_name;
      // Initialize published flag
      is_published_[it.first] = false;
    }
    // Get the parameter value

    base_frame_id_ = "camera_link";
    this->declare_parameter<std::string>("base_frame_id", base_frame_id_);
    this->get_parameter("base_frame_id", base_frame_id_);

    RCLCPP_INFO(this->get_logger(), "Base frame ID: %s", base_frame_id_.c_str());

    int tmp_disparity_type_ = 0;
    disparity_type_ = DisparityComputingMethod::BM;
    this->declare_parameter<int>("disparity_computing_method", tmp_disparity_type_);
    this->get_parameter("disparity_computing_method", tmp_disparity_type_);
    disparity_type_ = (DisparityComputingMethod)tmp_disparity_type_;
    api_->SetDisparityComputingMethodType(disparity_type_);

    RCLCPP_INFO(this->get_logger(), "Disparity computing method: %d", tmp_disparity_type_);

    // device options of standard
    if (model_ == Model::STANDARD) {
      option_names_ = {
          {Option::GAIN, "standard/gain"},
          {Option::BRIGHTNESS, "standard/brightness"},
          {Option::CONTRAST, "standard/contrast"},
          {Option::FRAME_RATE, "standard/frame_rate"},
          {Option::IMU_FREQUENCY, "standard/imu_frequency"},
          {Option::EXPOSURE_MODE, "standard/exposure_mode"},
          {Option::MAX_GAIN, "standard/max_gain"},
          {Option::MAX_EXPOSURE_TIME, "standard/max_exposure_time"},
          {Option::DESIRED_BRIGHTNESS, "standard/desired_brightness"},
          {Option::IR_CONTROL, "standard/ir_control"},
          {Option::HDR_MODE, "standard/hdr_mode"},
          {Option::ACCELEROMETER_RANGE, "standard/accel_range"},
          {Option::GYROSCOPE_RANGE, "standard/gyro_range"}};
    }

    for (auto &&it : option_names_) {
      if (!api_->Supports(it.first))
        continue;

      this->declare_parameter<int>(it.second, -1);

      // Retrieve parameter value
      int value = this->get_parameter(it.second).as_int();

      if (value != -1) {
          RCLCPP_INFO(this->get_logger(), "Set %s to %d", it.second.c_str(), value);
          api_->SetOptionValue(it.first, value);
      }

      RCLCPP_INFO(this->get_logger(), "%d: %d",
                  static_cast<int>(it.first),
                  api_->GetOptionValue(it.first));
    }

    // publishers

  }
  ~MynteyeWrapper() override {
    if (api_) {
      api_->Stop(Source::ALL);
    }
    if (time_beg_ != -1) {
      double time_end = this->now().seconds();

      LOG(INFO) << "Time elapsed: " << compute_time(time_end, time_beg_)
                << " s";
      if (left_time_beg_ != -1) {
        LOG(INFO) << "Left count: " << left_count_ << ", fps: "
                  << (left_count_ / compute_time(time_end, left_time_beg_));
      }
      if (right_time_beg_ != -1) {
        LOG(INFO) << "Right count: " << right_count_ << ", fps: "
                  << (right_count_ / compute_time(time_end, right_time_beg_));
      }
      if (imu_time_beg_ != -1) {
          if (model_ == Model::STANDARD) {
            LOG(INFO) << "Imu count: " << imu_count_ << ", hz: "
                      << (imu_count_ /
                      compute_time(time_end, imu_time_beg_));
          } else {
          if (publish_imu_by_sync_) {
            LOG(INFO) << "imu_sync_count: " << imu_sync_count_ << ", hz: "
                      << (imu_sync_count_ /
                      compute_time(time_end, imu_time_beg_));
          } else {
            LOG(INFO) << "Imu count: " << imu_count_ << ", hz: "
                      << (imu_count_ /
                      compute_time(time_end, imu_time_beg_));
          }
        }
      }

    }
    RCLCPP_INFO(this->get_logger(), "Destructor called");

  }

private:
  void initDevice() {
    std::shared_ptr<Device> device = nullptr;

    device = selectDevice();

    api_ = mynteye::API::Create(device);
    auto &&requests = device->GetStreamRequests();
    std::size_t m = requests.size();
    int request_index = 0;

    model_ = api_->GetModel();

    // # MYNTEYE-S1030, Reslution: 752x480, Format: YUYV
    // # index_s_0: 0
    // # MYNTEYE-S1030, Reslution: 376x240, Format: YUYV
    // # index_s_1: 1
    if (model_ == Model::STANDARD) {
      RCLCPP_INFO(this->get_logger(), "Model::STANDARD");
      frame_rate_ = api_->GetOptionValue(Option::FRAME_RATE);
    }

    if (m <= 0){
      RCLCPP_ERROR(this->get_logger(), "No MYNT EYE devices :(");
    }

    if (m <= 1) {
      RCLCPP_INFO(this->get_logger(), "Only one stream request, select index: 0");
      api_->ConfigStreamRequest(requests[0]);
    } else {
      if (request_index >= m) {
        RCLCPP_INFO(this->get_logger(), "Resquest_index out of range");
        api_->ConfigStreamRequest(requests[0]);
      } else {
        api_->ConfigStreamRequest(requests[request_index]);
      }
    }

    computeRectTransforms();
  }


  std::shared_ptr<Device> selectDevice() {
    // NODELET_INFO_STREAM("Detecting MYNT EYE devices");
    RCLCPP_INFO(this->get_logger(), "Detecting MYNT EYE devices");

    Context context;
    auto &&devices = context.devices();

    size_t n = devices.size();
    if (n <= 0) {
      RCLCPP_INFO(this->get_logger(), "No MYNT EYE devices :(");
    }

    RCLCPP_INFO(this->get_logger(), "MYNT EYE devices:");
    for (size_t i = 0; i < n; i++) {
      auto &&device = devices[i];
      auto &&name = device->GetInfo(Info::DEVICE_NAME);
      auto &&serial_number = device->GetInfo(Info::SERIAL_NUMBER);
      RCLCPP_INFO(this->get_logger(),
      "  index: %li, name: %s, serial number: %s",
       i, name.c_str(),
       serial_number.c_str());
    }

    if (n <= 1) {
      RCLCPP_INFO(this->get_logger(), "Only one MYNT EYE device, select index: 0");
      return devices[0];
    } else {
      while (true) {
        size_t i;
        RCLCPP_INFO(this->get_logger(),
        "There are : %li MYNT EYE devices, select index: ", n);
        std::cin >> i;
        if (i >= n) {
          RCLCPP_INFO(this->get_logger(), "Index out of range :(");
          continue;
        }
        return devices[i];
      }
    }

    return nullptr;
  }


  std::shared_ptr<IntrinsicsBase> getDefaultIntrinsics() {
    auto res = std::make_shared<IntrinsicsPinhole>();
    res->width = 640;
    res->height = 400;
    res->model = 0;
    res->fx = 3.6220059643202876e+02;
    res->fy = 3.6350065250745848e+02;
    res->cx = 4.0658699068023441e+02;
    res->cy = 2.3435161110061483e+02;
    double codffs[5] = {
      -2.5034765682756088e-01,
      5.0579399202897619e-02,
      -7.0536676161976066e-04,
      -8.5255451307033846e-03,
      0.
    };
    for (unsigned int i = 0; i < 5; i++) {
      res->coeffs[i] = codffs[i];
    }
    return res;
  }

  std::shared_ptr<Extrinsics> getDefaultExtrinsics() {
    auto res = std::make_shared<Extrinsics>();
    double rotation[9] = {
      9.9867908939669447e-01,  -6.3445566137485428e-03, 5.0988459509619687e-02,
      5.9890316389333252e-03,  9.9995670037792639e-01,  7.1224201868366971e-03,
      -5.1031440326695092e-02, -6.8076406092671274e-03, 9.9867384471984544e-01
    };
    double translation[3] = {-1.2002489764113250e+02, -1.1782637409050747e+00,
        -5.2058205159996538e+00};
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++) {
        res->rotation[i][j] = rotation[i*3 + j];
      }
    }
    for (unsigned int i = 0; i < 3; i++) {
      res->translation[i] = translation[i];
    }
    return res;
  }


  void computeRectTransforms() {
    // TODO fix for modern asser from rclcpp!
    assert(api_);
    // ROS_ASSERT(api_);
    auto in_left_base = api_->GetIntrinsicsBase(Stream::LEFT);
    auto in_right_base = api_->GetIntrinsicsBase(Stream::RIGHT);
    is_intrinsics_enable_ = in_left_base && in_right_base;
    if (is_intrinsics_enable_) {
      if (in_left_base->calib_model() != CalibrationModel::PINHOLE ||
          in_right_base->calib_model() != CalibrationModel::PINHOLE) {
        return;
      }
    } else {
      in_left_base = getDefaultIntrinsics();
      in_right_base = getDefaultIntrinsics();
    }

    auto in_left = *std::dynamic_pointer_cast<IntrinsicsPinhole>(in_left_base);
    auto in_right = *std::dynamic_pointer_cast<IntrinsicsPinhole>(
        in_right_base);
    auto ex_right_to_left = api_->GetExtrinsics(Stream::RIGHT, Stream::LEFT);
    if (!is_intrinsics_enable_) {
      ex_right_to_left = *(getDefaultExtrinsics());
    }

    cv::Size size{in_left.width, in_left.height};
    cv::Mat M1 =
        (cv::Mat_<double>(3, 3) << in_left.fx, 0, in_left.cx, 0, in_left.fy,
         in_left.cy, 0, 0, 1);
    cv::Mat M2 =
        (cv::Mat_<double>(3, 3) << in_right.fx, 0, in_right.cx, 0, in_right.fy,
         in_right.cy, 0, 0, 1);
    cv::Mat D1(1, 5, CV_64F, in_left.coeffs);
    cv::Mat D2(1, 5, CV_64F, in_right.coeffs);
    cv::Mat R =
        (cv::Mat_<double>(3, 3) << ex_right_to_left.rotation[0][0],
         ex_right_to_left.rotation[0][1], ex_right_to_left.rotation[0][2],
         ex_right_to_left.rotation[1][0], ex_right_to_left.rotation[1][1],
         ex_right_to_left.rotation[1][2], ex_right_to_left.rotation[2][0],
         ex_right_to_left.rotation[2][1], ex_right_to_left.rotation[2][2]);
    cv::Mat T(3, 1, CV_64F, ex_right_to_left.translation);

    cv::stereoRectify(
        M1, D1, M2, D2, size, R, T, left_r_, right_r_, left_p_, right_p_, q_,
        cv::CALIB_ZERO_DISPARITY, 0, size, &left_roi_, &right_roi_);

    std::stringstream ss;
    ss << "left_r_: " << left_r_
       << "\nright_r_: " << right_r_
       << "\nleft_p_: " << left_p_
       << "\nright_p_: " << right_p_
       << "\nq_: " << q_;
    RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
  }

  bool is_intrinsics_enable_;
  pthread_mutex_t mutex_data_;
  Model model_;
  std::map<Option, std::string> option_names_;
  std::map<Stream, bool> is_published_;
  bool is_motion_published_;
  bool is_started_;
  int frame_rate_;
  std::shared_ptr<API> api_;

  std::string base_frame_id_;
  DisparityComputingMethod disparity_type_;

  // rectification transforms
  cv::Mat left_r_, right_r_, left_p_, right_p_, q_;
  cv::Rect left_roi_, right_roi_;

  double time_beg_ = -1;
  double left_time_beg_ = -1;
  double right_time_beg_ = -1;
  double imu_time_beg_ = -1;
  std::size_t left_count_ = 0;
  std::size_t right_count_ = 0;
  std::size_t imu_count_ = 0;
  std::size_t imu_sync_count_ = 0;
  bool publish_imu_by_sync_ = true;
};

MYNTEYE_END_NAMESPACE

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mynteye::MynteyeWrapper>());
  rclcpp::shutdown();
  return 0;
}
