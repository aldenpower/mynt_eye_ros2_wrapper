#include "rclcpp/rclcpp.hpp"

#include <opencv2/calib3d/calib3d.hpp>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"
#include "mynteye/device/context.h"
#include "mynteye/device/device.h"


MYNTEYE_BEGIN_NAMESPACE

class MynteyeWrapper: public rclcpp::Node {
public:
  MynteyeWrapper()
  : Node("mynteye_wrapper") {

    initDevice();

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
  Model model_;
  int frame_rate_;
  std::shared_ptr<API> api_;

  // rectification transforms
  cv::Mat left_r_, right_r_, left_p_, right_p_, q_;
  cv::Rect left_roi_, right_roi_;
};

MYNTEYE_END_NAMESPACE

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mynteye::MynteyeWrapper>());
  rclcpp::shutdown();
  return 0;
}
