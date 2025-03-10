// 装甲板预测  标记当前装甲板与下一时刻装甲板的位置 发送装甲板坐标

#include "Kalman_predict.h"
#include "armor_parameter/msg/armor.hpp"


// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/publisher.hpp"
#include "image_transport/image_transport.hpp"
// opencv
#include "opencv2/opencv.hpp"

// 装甲板预测
class Armor_Predict_Node : public rclcpp::Node
{
public:
    Armor_Predict_Node() : Node("predict")
    {
        RCLCPP_INFO(this->get_logger(), "Armor_Predict_Node working...");
        // 创建坐标接收节点
        sub_x_y_z = this->create_subscription<armor_parameter::msg::Armor>("camera/xyz", 10,
                                                                           std::bind(&Armor_Predict_Node::get_xyz, this, std::placeholders::_1));
        // 创建图像接收节点
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_track", 10,
                                                                  std::bind(&Armor_Predict_Node::armor_predict, this, std::placeholders::_1));
        //创建图像发布节点
        image_pub_ = image_transport::create_publisher(this,"camera/result");
        // 初始化卡尔曼类
        kalman_ = std::make_shared<kalman_armor>();
        kalman_->init(3,1,0);
        //匀加速模型
        kalman_->kfinit_uniform();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;          // 接收图像
    rclcpp::Subscription<armor_parameter::msg::Armor>::SharedPtr sub_x_y_z; // 接收坐标
    image_transport::Publisher image_pub_;                                   //图像发布 使用rqt接收

    // 图像处理
    void armor_predict(const sensor_msgs::msg::Image::ConstSharedPtr &image);

    // 坐标获取
    void get_xyz(const armor_parameter::msg::Armor::ConstSharedPtr &xyz);

    // 卡尔曼类
    std::shared_ptr<kalman_armor> kalman_;
};

void Armor_Predict_Node::get_xyz(const armor_parameter::msg::Armor::ConstSharedPtr &xyz)
{
    cv::Mat measurement;
    if (kalman_->update_armor == 0)
    {
        //          初始化装甲板
        //      new_armor = target_armor;
        kalman_->new_armor.x = xyz->pix_x;
        kalman_->new_armor.y = xyz->pix_y;
        kalman_->new_armor.position_x = xyz->x;
        kalman_->new_armor.position_y = xyz->y;
        kalman_->new_armor.position_z = xyz->z;
        //      last_armor = target_armor;
        kalman_->last_armor = kalman_->new_armor;

        kalman_->kalman.statePost = (cv::Mat_<float>(3, 1) << kalman_->new_armor.y, 0, 0);
        kalman_->update_armor = 1;
    }
    else
    {
        //           更新装甲板
        //    last_armor =   new_armor;
        kalman_->last_armor = kalman_->new_armor;

        //     new_armor =   target_armor;
        kalman_->new_armor.x = xyz->pix_x;
        kalman_->new_armor.y = xyz->pix_y;
        kalman_->new_armor.position_x = xyz->x;
        kalman_->new_armor.position_y = xyz->y;
        kalman_->new_armor.position_z = xyz->z;
        kalman_->new_armor.armor_speed = kalman_->kalman.statePost.at<float>(1);

        if (!kalman_->same_armor())
        {
            //last_armor = target_armor;
            kalman_->last_armor.x = xyz->pix_x;
            kalman_->last_armor.y = xyz->pix_y;
            kalman_->last_armor.position_x = xyz->x;
            kalman_->last_armor.position_y = xyz->y;
            kalman_->last_armor.position_z = xyz->z;
            // 重置卡尔曼滤波
            kalman_->kalman.statePost.at<float>(0) = kalman_->new_armor.x;  //更新位置
            kalman_->kalman.statePost.at<float>(1) = 0;                     //重置速度
        }
    }
    measurement = (cv::Mat_<float>(1, 1) << kalman_->new_armor.x);
    kalman_->kalman_predict();
    kalman_->kalman_updata(measurement);
    kalman_->new_armor.pre_x = kalman_->kalman.statePost.at<float>(0);
    kalman_->new_armor.pre_y = kalman_->new_armor.y;
}

void Armor_Predict_Node::armor_predict(const sensor_msgs::msg::Image::ConstSharedPtr &image)
{
    cv::Mat src;
    // 图像处理
    auto src_ = cv_bridge::toCvShare(image, "bgr8");
    src = src_->image;
    if(kalman_->new_armor.pre_x != 0)
    {
      cv::circle(src, cv::Point(kalman_->new_armor.pre_x, kalman_->new_armor.pre_y), 10, cv::Scalar(0, 0, 255), 5);
    }
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera";
    image_pub_.publish(*cv_bridge::CvImage(header,"bgr8",src).toImageMsg());
    cv::imshow("video", src);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Armor_Predict_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}