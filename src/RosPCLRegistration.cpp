// ros includes
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

// system includes
#include <memory>

using namespace std;
using std::placeholders::_1;
//using namespace pcl::recognition;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PC;

namespace template_matching3d
{
    class RosPCLRegistration : public rclcpp::Node
    {
    public: 
        RosPCLRegistration()://string _scene_topic, string obj_file, 
                            //, float maxCorr, 
                            //int maxIter, float fitEps):
            Node("PCLRegistration")
        {

            this->declare_parameter("debug", false);
            debug_ = this->get_parameter("debug").as_bool();  

            if(debug_)
            {
                this->declare_parameter("debug_topic", "in_node");
                string debug_topic = this->get_parameter("debug_topic").as_string();
                publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(debug_topic, 10);
            }

            this->declare_parameter("initial_est", rclcpp::PARAMETER_DOUBLE_ARRAY);
            std::vector<double> init = this->get_parameter("initial_est").as_double_array();
            initEst_ << init[0], init[1], init[2], init[3],
                        init[4], init[5], init[6], init[7],
                        init[8], init[9], init[10], init[11],
                        init[12], init[13], init[14], init[15];


            this->declare_parameter("input_model", "none");
            string obj_file = this->get_parameter("input_model").as_string();
            // initialize the object recognition
            PC::Ptr model = loadPCL(obj_file); // load file
            this->declare_parameter("center_model", false);
            bool center = this->get_parameter("center_model").as_bool();  
            if(center)
            {
                recenterModel(model);
            }

            icp_.setInputSource(model);

            this->declare_parameter("max_corr_dist", 0.1);
            double maxCorr = this->get_parameter("max_corr_dist").as_double();            
            // Set the max correspondence distance to 5cm (e.g., correspondences with higher
            // distances will be ignored)
            icp_.setMaxCorrespondenceDistance (maxCorr);

            this->declare_parameter("max_iterations", 50);
            double maxIter = this->get_parameter("max_iterations").as_int();  
            // Set the maximum number of iterations (criterion 1)
            icp_.setMaximumIterations (maxIter);

            this->declare_parameter("euclidean_fitness_epsilon", 0.1);
            double fitEps = this->get_parameter("euclidean_fitness_epsilon").as_double();  
            // Set the transformation epsilon (criterion 2)
            icp_.setTransformationEpsilon (1e-8);
            // Set the euclidean distance difference epsilon (criterion 3)
            icp_.setEuclideanFitnessEpsilon (fitEps);

            //icp_.setRANSACOutlierRejectionThreshold (0.01);


            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            // set up the subcription
            this->declare_parameter("scene_topic", "scene");
            string scene_topic = this->get_parameter("scene_topic").as_string();  
            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(scene_topic, 
                10,
                std::bind(&RosPCLRegistration::scene_callback, this, _1)
            );

        }    
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        pcl::IterativeClosestPoint<PointType, PointType> icp_;
        Eigen::Matrix4f initEst_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        bool debug_;
        void recenterModel(PC::Ptr model)
        {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            for(long unsigned int i = 0; i < model->points.size(); i ++)
            {
                PointType pt = model->points[i];
                x += pt.x;
                y += pt.y;
                z += pt.z;
            }
            x /= model->points.size();
            y /= model->points.size();
            z /= model->points.size();
            for(long unsigned int i = 0; i < model->points.size(); i++)
            {
                model->points[i].x -= x;
                model->points[i].y -= y;
                model->points[i].z -= z;
            }
        }

        void scene_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            // convert to pcl
            //cout << msg->header.frame_id << endl;
            PC::Ptr scene(new PC);
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg,pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2,*scene);

            // remove nans?
            pcl::Indices indxs;// = new(pcl::indices);
            //pcl::removeNaNFromPointCloud(*scene, indxs);
            pcl::removeNaNFromPointCloud(*scene, *scene, indxs);

            // setup ICP
            icp_.setInputTarget(scene);
            PC::Ptr Final(new(PC));
            //icp_.align(*Final);
            icp_.align(*Final, initEst_);
            Eigen::Matrix4f est = icp_.getFinalTransformation();
            if(debug_)
            {
                cout.precision(2);
                cout << "Fitness: " << icp_.getFitnessScore() << endl;
                cout << est << endl;
            }

            // publish tf
            publishTransform(est);
            if(debug_)
            {
                pubICP(Final);
            }

        }

        void pubICP(PC::Ptr cloud)
        {
            auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*cloud, *pc2_msg_);
            //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            pc2_msg_->header.frame_id = "map";
            pc2_msg_->header.stamp = now();
            publisher_->publish(*pc2_msg_);
        }

        PC::Ptr loadPCL(string fp)
        {
            PC::Ptr cloud (new pcl::PointCloud<PointType>);
            if (pcl::io::loadPCDFile<PointType> (fp, *cloud) == -1) //* load the file
            {           
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            }
            return cloud;
        }

        void publishTransform(Eigen::Matrix4f tf)
        {

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();//.to_msg();//+60000000;
            t.header.stamp.sec += 1;
            t.header.frame_id = "milk_frame";
            t.child_frame_id = "map";
            t.transform.translation.x = tf(0,3);
            t.transform.translation.y = tf(1,3);
            t.transform.translation.z = tf(2,3);
            //cout << "x:" << t.transform.translation.x << 
            //        " y:" << t.transform.translation.y << 
            //        " z:" << t.transform.translation.z << endl;
            tf2::Quaternion q;
            tf2::Matrix3x3 rot(tf(0,0), tf(0,1), tf(0,2),
                            tf(1,0), tf(1,1), tf(1,2),
                            tf(2,0), tf(2,1), tf(2,2));
            rot.getRotation(q);
            //q = q.inverse();
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_static_broadcaster_->sendTransform(t);
        }
    };

};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<template_matching3d::RosPCLRegistration>());
  rclcpp::shutdown();
  return 0;
}
