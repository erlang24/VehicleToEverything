#include <cstdio>
#include <iostream>
#include <chrono>
#include <thread>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <grpc/grpc.h>
#include <grpcpp/server_builder.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <nlohmann/json.hpp>

#include "obu_service.pb.h"
#include "obu_message.pb.h"
#include "obu_service.grpc.pb.h"
#include "obu_message.grpc.pb.h"
#include "obu_message.pb.h"
#include "obu_service.pb.h"

#include "std_msgs/msg/string.hpp"
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using grpc::obu::msg::ObuAnswerRequest;
using grpc::obu::msg::ObuAnswerResponse;
using grpc::obu::msg::ObuService;
using grpc::obu::msg::ObuBasicInfo;
using grpc::obu::msg::ObuTrafficLightInfo;
using grpc::obu::msg::TrafficLight;
using json = nlohmann::json;
// Implement the service
class ObuServiceImpl final : public grpc::obu::msg::ObuService::Service {
public:
    ObuServiceImpl() {
      // 初始化 ROS 2 节点
      rclcpp::init(0, nullptr); 
      node_ = rclcpp::Node::make_shared("grpc_bridge_gui");
      sub_scenes_ = node_->create_subscription<std_msgs::msg::String>("/scenes",rclcpp::QoS{5},
                    std::bind(&ObuServiceImpl::scenes_callback, this, std::placeholders::_1)
                        );
      sub_spat_ = node_->create_subscription<std_msgs::msg::String>("/traffic_light_json",rclcpp::QoS{5},
                    std::bind(&ObuServiceImpl::spat_callback, this, std::placeholders::_1)
                        );
      sub_gnss_ = node_->create_subscription<gps_msgs::msg::GPSFix>("/ui_datapublisher_llahs",rclcpp::QoS{5},
                    std::bind(&ObuServiceImpl::gnss_callback, this, std::placeholders::_1)
                        );
      sub_rpy_ =  node_->create_subscription<geometry_msgs::msg::Twist>("/RPYdata",rclcpp::QoS{5},
                    std::bind(&ObuServiceImpl::rpy_callback, this, std::placeholders::_1)
                        );                 
      // pub_engage_= node_->create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("/autoware/engage",10);
      // pub_goal_= node_->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal",10);
      // pub_initialpose_= node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10);
      // pub_objects_= node_->create_publisher<dummy_perception_publisher::msg::Object>("/initialpose",10);
    }

    ~ObuServiceImpl() {
      // 关闭 ROS 2
      rclcpp::shutdown();
    }




    Status answerMethod(grpc::ServerContext* context, const grpc::obu::msg::ObuAnswerRequest* request,
                        grpc::obu::msg::ObuAnswerResponse* response) override {


        std::cout << "Server: answerMethod for \"" << request->api_version() << "\"." << std::endl;

        // // Sleep for 1 second
        //std::cout << "Sleep after 1 second!" << std::endl;
        //std::this_thread::sleep_for(std::chrono::seconds(1));

        // Fill the response message
        rclcpp::spin_some(node_);
        response->set_api_version("v1.0");
        
        // basic_info = basic_info_;
        if (gnss_msg != NULL) {
            auto* basic_info = response->mutable_basic_info();
            basic_info->set_latitude(gnss_msg->latitude);
            basic_info->set_longitude(gnss_msg->longitude);
            basic_info->set_altitude(gnss_msg->altitude);
            basic_info->set_gnss_heading(gnss_msg->track);
            basic_info->set_gnss_speed(gnss_msg->speed);
            gnss_msg = NULL;
        }
        // basic_info->set_vehicle_speed(12.0);
        // basic_info->set_pitch_rad(0.1);
        // basic_info->set_roll_rad(0.1);
        // basic_info->set_yaw_rad(0.1);

        // auto* traffic_light_info = response->mutable_traffic_light_info();
        // auto* turn_left = traffic_light_info->mutable_turn_left();
        // turn_left->set_state(grpc::obu::msg::TrafficLight::YELLOW);
        // if (count_test % 30 < 10) turn_left->set_state(grpc::obu::msg::TrafficLight::RED);
        // if (count_test % 30 > 20) turn_left->set_state(grpc::obu::msg::TrafficLight::GREEN); 
        // turn_left->set_remaining_time(10 - (count_test % 30)/3);


        // auto* straight_ahead = traffic_light_info->mutable_straight_ahead();
        // straight_ahead->set_state(grpc::obu::msg::TrafficLight::YELLOW);
        // if (count_test % 30 < 10) straight_ahead->set_state(grpc::obu::msg::TrafficLight::RED);
        // if (count_test % 30 > 20) straight_ahead->set_state(grpc::obu::msg::TrafficLight::GREEN); 
        // straight_ahead->set_remaining_time(10 - (count_test % 30)/3);

        // auto* turn_right = traffic_light_info->mutable_turn_right();
        // turn_right->set_state(grpc::obu::msg::TrafficLight::YELLOW);
        // turn_right->set_remaining_time(5.0);
        if (scenses_msg != NULL) {
          //盲区预警、紧急车辆提醒、异常车辆提醒、交叉路口碰撞预警、前项碰撞预警
          auto* warning_info = response->mutable_warning_info();
          warning_info->set_fcw(scenses_msg->data=="前项碰撞预警");
          warning_info->set_avw(scenses_msg->data=="异常车辆提醒");
          warning_info->set_bsw(scenses_msg->data=="盲区预警");
          warning_info->set_evw(scenses_msg->data=="紧急车辆提醒");
          warning_info->set_icw(scenses_msg->data=="交叉路口碰撞预警");
          // std::cout<<scenses_msg->data.substr(0,18)<<std::endl;
          // std::cout<<(scenses_msg->data.substr(0,18)=="绿波车速引导")<<std::endl;
          // std::cout<<std::stof(scenses_msg->data.substr(19))<<std::endl;
          if (scenses_msg->data.substr(0,18)=="绿波车速引导")
          {
            warning_info->set_glosa(true);
            warning_info->set_glosa_mps(std::stof(scenses_msg->data.substr(19)));
            // std::cout<<scenses_msg->data.substr(19)<<std::endl;
          }
          
          
          scenses_msg = NULL;
        }
        else
        {
          auto* warning_info = response->mutable_warning_info();
          warning_info->set_fcw(false);
          warning_info->set_avw(false);
          warning_info->set_glosa(false);
          warning_info->set_evw(false);
          warning_info->set_icw(false);
          warning_info->set_bsw(false);
          // std::cout<<"false"<<std::endl;
        }
        if (spat_msg != NULL)
        {
          nlohmann::json jsonData;
          jsonData = nlohmann::json::parse(spat_msg->data);
          auto* spat = response->mutable_traffic_light_info();
          TrafficLight* traffic_light  =  spat->mutable_straight_ahead();

          if (jsonData["intersections"][0]["phases"][0]["phaseStates"][0]["light"] == 3)
          {
            traffic_light->set_state(grpc::obu::msg::TrafficLight::RED);
            traffic_light->set_remaining_time(int(jsonData["intersections"][0]["phases"][0]["phaseStates"][0]["timing"]["counting"]["likelyEndTime"])/10);
          }
          if (jsonData["intersections"][0]["phases"][0]["phaseStates"][0]["light"] == 6)
          {
            traffic_light->set_state(grpc::obu::msg::TrafficLight::GREEN);
            traffic_light->set_remaining_time(int(jsonData["intersections"][0]["phases"][0]["phaseStates"][0]["timing"]["counting"]["likelyEndTime"])/10);
          }
          if (jsonData["intersections"][0]["phases"][0]["phaseStates"][0]["light"] == 7)
          {
            traffic_light->set_state(grpc::obu::msg::TrafficLight::YELLOW);
            traffic_light->set_remaining_time(int(jsonData["intersections"][0]["phases"][0]["phaseStates"][0]["timing"]["counting"]["likelyEndTime"])/10);
          }
          std::cout<<"get_remaining_time"<<int(jsonData["intersections"][0]["phases"][0]["phaseStates"][0]["timing"]["counting"]["likelyEndTime"])/10<<std::endl;
          spat_msg = NULL;
        }

        return Status::OK;
        }
private:
    std_msgs::msg::String::SharedPtr scenses_msg;
    std_msgs::msg::String::SharedPtr spat_msg;
    gps_msgs::msg::GPSFix::SharedPtr gnss_msg;
    geometry_msgs::msg::Twist::SharedPtr rpy_msg;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_scenes_{nullptr};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_spat_{nullptr};
    
    rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr sub_gnss_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_rpy_{nullptr};
    
    int count_test  = 0;

    void scenes_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      scenses_msg = msg;

    }
    void spat_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      spat_msg = msg;

    }
    void gnss_callback(const gps_msgs::msg::GPSFix::SharedPtr msg)
    {
      gnss_msg = msg;

    }
    void rpy_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      rpy_msg = msg;

    }

};

int main(int argc, char* argv[])
{
    grpc::ServerBuilder builder;
    std::string addr_uri = "127.0.0.1:50052";
    builder.AddListeningPort(addr_uri, grpc::InsecureServerCredentials());
    std::cout << "ListeningPort : " << addr_uri << std::endl;

    ObuServiceImpl my_service;
    builder.RegisterService(&my_service);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    server->Wait();
    
    return 0;
}
