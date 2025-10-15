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
