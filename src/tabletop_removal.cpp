// ros includes
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>


// pcl includes
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using namespace std;
using std::placeholders::_1;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PC;
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> PCN;
typedef pcl::PointIndices PtIdxs;

namespace template_matching3d
{
    class TableTopSegmentation : public rclcpp::Node 
    {
        public:
        TableTopSegmentation():
            Node("Tabletop_Segmentation")
        {
            
            this->declare_parameter("debug", false);
            debug_ = this->get_parameter("debug").as_bool(); 

            this->declare_parameter("maxZ", 0.5);
            maxZ_ = this->get_parameter("maxZ").as_double(); 

            this->declare_parameter("inlier_dist", 0.1);
            distThreshold_ = this->get_parameter("inlier_dist").as_double(); 

            this->declare_parameter("input_topic", "inny");
            string inTopic = this->get_parameter("input_topic").as_string();
            
            this->declare_parameter("output_topic", "outy");
            string outTopic = this->get_parameter("output_topic").as_string();
               
            publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(outTopic, 10);
            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(inTopic, 
                10,
                std::bind(&TableTopSegmentation::scene_callback, this, _1)
            );
        }
        private:
        bool debug_;
        void scene_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            // unpack pcl
            PC::Ptr scene(new PC);
            PC::Ptr filtered_scene(new PC);
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg,pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2,*scene);

            // get inliner indexs
            PtIdxs::Ptr inliers (new PtIdxs);
            inliers = getInliers(scene);

            // add z-filter
            imposeMaxZ(scene, inliers);

            // remove inliers
            // Create the filtering object
            pcl::ExtractIndices<PointType> extract;
            extract.setInputCloud (scene);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*filtered_scene);

            // publish new pcl
            publishPC(filtered_scene, msg);
        }
        void publishPC(PC::Ptr output, 
                    const sensor_msgs::msg::PointCloud2::SharedPtr input)
        {
            auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*output, *pc2_msg_);
            //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            pc2_msg_->header.frame_id = input->header.frame_id;
            pc2_msg_->header.stamp = input->header.stamp;
            publisher_->publish(*pc2_msg_);
        }

        void imposeMaxZ(PC::Ptr scene, PtIdxs::Ptr idxs)
        {
            for(long unsigned int nIndex = 0; nIndex < scene->points.size(); nIndex++)
            {
                if(scene->points[nIndex].z > maxZ_)
                {
                    idxs->indices.push_back(nIndex);
                }
            }
        }

        PtIdxs::Ptr getInliers(PC::Ptr scene)
        {
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            PtIdxs::Ptr inliers (new PtIdxs);
            pcl::SACSegmentation<PointType> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (distThreshold_);

            seg.setInputCloud (scene);
            seg.segment (*inliers, *coefficients);
            return inliers;
        }
        float distThreshold_;
        float maxZ_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    };

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<template_matching3d::TableTopSegmentation>());
    rclcpp::shutdown();
}