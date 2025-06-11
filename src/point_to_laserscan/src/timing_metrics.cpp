#include <chrono>
#include <iostream>
#include <string>
#include "rclcpp/node.hpp"  

//created using the help of LLMs
class ScopedTimer {
public:
    ScopedTimer(const std::string& name, rclcpp::Node* node, bool verbose) : name_(name), node_(node), verbose_(verbose) {
        if(verbose_){
            start_ = std::chrono::high_resolution_clock::now();
            clocking_ = true;
        }
    }
    void stopClock(){
        if(verbose_){
            clocking_ =false;
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_);
            RCLCPP_INFO(node_->get_logger(), "%s took %.3f ms", name_.c_str(), static_cast<double>(duration.count()));
        }
    }
    ~ScopedTimer() {
        if (clocking_ && verbose_){
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_);
            RCLCPP_INFO(node_->get_logger(), "%s took %.3f ms", name_.c_str(), static_cast<double>(duration.count()));
        }
        
    }

private:
    std::string name_;
    std::chrono::high_resolution_clock::time_point start_;
    bool clocking_;
    rclcpp::Node* node_;
    bool verbose_;

};