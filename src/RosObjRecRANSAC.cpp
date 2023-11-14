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

// system includes
#include <memory>

using namespace std;
using std::placeholders::_1;
//using namespace pcl::recognition;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PC;
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> PCN;
typedef pcl::recognition::ObjRecRANSAC ORR;

namespace template_matching3d
{
    class RosObjRecRANSAC : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

        pcl::recognition::ObjRecRANSAC ObjRec_;
        float minConfidence_;
        void scene_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            // convert to pcl
            cout << msg->header.frame_id << endl;
            PC::Ptr scene(new PC);
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg,pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2,*scene);
            // extract normals
            PCN::Ptr scene_norms = calcNormals(scene);
            // recognize model
            std::list<ORR::Output> outs;
            ObjRec_.recognize(*scene, *scene_norms, outs);
            std::vector<ORR::Output> outVec(outs.begin(), outs.end());
            // publish transform
            //std::list<pcl::recognition::ObjRecRANSAC::Output>::iterator it = outs.begin();
            //advance(it, 0);
            ORR::Output best = outVec[0];
            double bestVal = -1.0;
            for(int i = 0; i < outVec.size(); i++)
            {
                if(outVec[i].match_confidence_ > bestVal)
                {
                    best = outVec[i];
                    bestVal = outVec[i].match_confidence_;
                }
            }
            cout.precision(2);
            cout << "\t\t" << best.object_name_ << endl;std::cout << "Transformation matrix:" << std::endl << std::endl;
		    cout << "\t\t    |" << best.rigid_transform_[0] << ", " << best.rigid_transform_[1]<< ", " << best.rigid_transform_[2] << "|\n";
		    cout << "\t\tR = |" << best.rigid_transform_[3] << ", " << best.rigid_transform_[4]<< ", " << best.rigid_transform_[5]<<"|\n";
		    cout << "\t\t    |" << best.rigid_transform_[6]<< ", " << best.rigid_transform_[7]<< ", " << best.rigid_transform_[8] << "|\n";

		    cout << "\t\tt = [" << best.rigid_transform_[9]<< ", " << best.rigid_transform_[10]<< ", " << best.rigid_transform_[11] << "]\n";
            //cout << "\t\t" << best.rigid_transform_ << endl;
            cout << "\t\t" << best.match_confidence_ << endl;
            if(bestVal > minConfidence_)
            {
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "milk_frame";
                t.child_frame_id = "map";
                t.transform.translation.x = best.rigid_transform_[9];
                t.transform.translation.y = best.rigid_transform_[10];
                t.transform.translation.z = best.rigid_transform_[11];
                tf2::Quaternion q;
                tf2::Matrix3x3 rot(best.rigid_transform_[0], best.rigid_transform_[1], best.rigid_transform_[2],
                                best.rigid_transform_[3], best.rigid_transform_[4], best.rigid_transform_[5],
                                best.rigid_transform_[6],best.rigid_transform_[7],best.rigid_transform_[8]);
                rot.getRotation(q);
                q = q.inverse();
                t.transform.rotation.x = q.x();
                t.transform.rotation.y = q.y();
                t.transform.rotation.z = q.z();
                t.transform.rotation.w = q.w();

                tf_static_broadcaster_->sendTransform(t);
            }


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

        PCN::Ptr calcNormals(PC::Ptr cloud) const
        {
            pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
            norm_est.setKSearch (10);
            norm_est.setInputCloud (cloud);
            PCN::Ptr normals (new PCN);
            norm_est.compute (*normals);
            return(normals);
        }


    public:

        RosObjRecRANSAC(string _scene_topic, string obj_file, float pair_width=0.01, float voxel_size=0.01, float minCon=0.7):
            Node("Object_Recognition"), ObjRec_(pair_width, voxel_size), minConfidence_(minCon)
        {
            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            ObjRec_.icpHypothesesRefinementOn();
            // set up the subcription
            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(_scene_topic, 
                10,
                std::bind(&RosObjRecRANSAC::scene_callback, this, _1)
            );
            // initialize the object recognition
            PC::Ptr model = loadPCL(obj_file); // load file
            PCN::Ptr model_norms = calcNormals(model); // calc normals
            // add model
            ObjRec_.addModel(*model, *model_norms, "Milk", NULL);

        }    
    };

}
int main(int argc, char * argv[])
{
    float pair_width = 0.01;
    float voxel_size = 0.01;
    float min_con = 0.7;

    if(argc > 2){
        pair_width = atof(argv[1]);
        voxel_size = atof(argv[2]);

    }
    if(argc > 3)
    {
        min_con = atof(argv[3]);
    }
  rclcpp::init(argc, argv);
  string fp = "/home/hunter/PiH_ws/src/template_matching3d/data/milk.pcd";
  rclcpp::spin(std::make_shared<template_matching3d::RosObjRecRANSAC>("point_cloud", fp, pair_width, voxel_size, min_con));
  rclcpp::shutdown();
  return 0;
}

/* TODO
    1) Configureation File
        - Model Paths
        - Model Name
        - System Parameters (pair_width, voxel_size, min_cond) 
        - Frame Names
        - Topic Names
    2) Class Variables
        - Model Name
        - Frame Names
*/