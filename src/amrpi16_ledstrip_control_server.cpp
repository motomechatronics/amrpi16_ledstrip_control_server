#include "rclcpp/rclcpp.hpp"
#include "/home/pi16/ros2_ws/src/amrpi16_ledstrip_control_server/include/amrpi16_ledstrip_control_server/PCA9685.h"
#include "/home/pi16/ros2_ws/src/amrpi16_ledstrip_control_server/include/amrpi16_ledstrip_control_server/I2C.h"
#include <thread>
#include <map>
#include <vector>

class LedControllerNode : public rclcpp::Node {
public:
    // I have found the right i2c channel and the board address with the fallowing command
    // i2cget -y 8 0x40
    // result:
    // 0x00
  
    LedControllerNode() : Node("led_controller_node"), pca_(8, 0x40) {
        // initialization PCA9685
        pca_.setPWMFreq(60); // set PWM frequency at 60Hz
        // turn off all pins
        for (int pin = 0; pin < 13; pin++)
        {
            pca_.setPWM(pin, 0, 0);
        }
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LedControllerNode::toggleLed, this));
        RCLCPP_INFO(get_logger(), "LedControllerNode initialized.");
        rgbPinLedStrip[1] = {1, 2, 3};  // pin numbers on Adafruit 16 channel LED driver
        rgbPinLedStrip[2] = {4, 5, 6};
        rgbPinLedStrip[3] = {7, 8, 9};
        rgbPinLedStrip[4] = {10, 11, 12};
   
    }

private:
    PCA9685 pca_;
    rclcpp::TimerBase::SharedPtr timer_;   
    bool led_state_ = true;
    int ledIntensity_;
    std::map<int, std::vector<int>> rgbPinLedStrip;
       
    

void testLedColor(int rgbPinLedStrip, int time_ms)
{
     for (ledIntensity_= 0; ledIntensity_<= 4095; ledIntensity_ += 10) 
     {
        pca_.setPWM(rgbPinLedStrip, 0, ledIntensity_);
        std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
        //RCLCPP_INFO(get_logger(), "LedIntensity1: %d", ledIntensity_);
     }

    for (ledIntensity_= 4095; ledIntensity_>= 0; ledIntensity_ -= 10) 
     {
        pca_.setPWM(rgbPinLedStrip, 0, ledIntensity_);
        std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
        //RCLCPP_INFO(get_logger(), "LedIntensity2: %d", ledIntensity_);
     }

     pca_.setPWM(rgbPinLedStrip, 0, 0);
}

void testLedStrip(int ledStripNumber, int time_ms)
{
    testLedColor(rgbPinLedStrip[ledStripNumber][0], time_ms);
    testLedColor(rgbPinLedStrip[ledStripNumber][1], time_ms);
    testLedColor(rgbPinLedStrip[ledStripNumber][2], time_ms);
    testLedColor(rgbPinLedStrip[ledStripNumber][3], time_ms);
}

void toggleLed() 
{
    testLedStrip(1, 1);
    testLedStrip(2, 1);
    testLedStrip(3, 1);
    testLedStrip(4, 1);
}

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
