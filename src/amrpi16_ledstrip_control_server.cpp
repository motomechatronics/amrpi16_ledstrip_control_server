#include "rclcpp/rclcpp.hpp"
#include "/home/pi16/ros2_ws/src/amrpi16_ledstrip_control_server/include/amrpi16_ledstrip_control_server/PCA9685.h"
#include "/home/pi16/ros2_ws/src/amrpi16_ledstrip_control_server/include/amrpi16_ledstrip_control_server/I2C.h"

class LedControllerNode : public rclcpp::Node {
public:
    // I have found the right i2c channel and the board address with the fallowing command
    // i2cget -y 8 0x40
    // result:
    // 0x00
  
    LedControllerNode() : Node("led_controller_node"), pca_(8, 0x40) {
        // Inizializza PCA9685
        pca_.setPWMFreq(1000); // Imposta la frequenza PWM a 1000Hz
        pca_.setPWM(1, 0, 0); // Assicura che tutti i LED siano spenti all'avvio
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&LedControllerNode::toggleLed, this));
        RCLCPP_INFO(get_logger(), "LedControllerNode initialized.");
   
    }

private:
    PCA9685 pca_;
    rclcpp::TimerBase::SharedPtr timer_;
    int led_intensity_ = 0;
    bool led_state_ = false;

    void toggleLed() {
        
        // Inverti lo stato del LED
        led_state_ = !led_state_;      

        //Imposta l'intensità del LED in base allo stato
        if (led_state_) {
            pca_.setPWM(1, led_intensity_,0); // Accendi il LED con l'intensità corrente
        } else {
            pca_.setPWM(1, 0, 0); // Spegni il LED
        }

        // Modifica l'intensità del LED
        led_intensity_ += 100;
        if (led_intensity_ > 4095) {
            led_intensity_ = 0; // Resetta l'intensità al valore minimo quando raggiunge il massimo
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
