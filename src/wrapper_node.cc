#include "rclcpp/rclcpp.hpp"

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

  Model model_;
  int frame_rate_;
  std::shared_ptr<API> api_;
};

MYNTEYE_END_NAMESPACE

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mynteye::MynteyeWrapper>());
  rclcpp::shutdown();
  return 0;
}
