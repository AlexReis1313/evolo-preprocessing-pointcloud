#include <chrono>
#include <iostream>
#include <string>
#include "rclcpp/node.hpp"  
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <functional>

//created using the help of LLMs
class ScopedTimer {
public:
    ScopedTimer(const std::string& name, rclcpp::Node* node, bool verbose, bool savefile,std::ofstream& file ) : name_(name), node_(node), verbose_(verbose), savefile_(savefile), file_(file){
        if(verbose_ || savefile_){
            start_ = std::chrono::high_resolution_clock::now();
            clocking_ = true;
        }
    }

    void stopClock(){
        if(verbose_ || savefile_){
            clocking_ =false;
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start_; 
            if(verbose_){
                RCLCPP_INFO(node_->get_logger(), "%s took %.3f ms", name_.c_str(), static_cast<double>(duration.count()));
            }
            if(savefile_){
                file_ << name_.c_str() << ", " << std::fixed << std::setprecision(3) << duration.count() << ", ms\n";

            }
        }
        
  
    }
    
    ~ScopedTimer() {
        if(clocking_ &&(verbose_ || savefile_)){
            clocking_ =false;
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start_; 
            if(verbose_){
                RCLCPP_INFO(node_->get_logger(), "%s took %.3f ms", name_.c_str(), static_cast<double>(duration.count()));
            }
            if(savefile_){
                file_ << name_.c_str() << ", " << std::fixed << std::setprecision(3) << duration.count() << ", ms\n";

            }
        }
        
    }

private:
    std::string name_;
    std::chrono::high_resolution_clock::time_point start_;
    bool clocking_;
    rclcpp::Node* node_;
    bool verbose_;
    std::ofstream& file_;
    bool savefile_;

};