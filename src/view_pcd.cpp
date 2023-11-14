
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// system
#include <thread>
#include <chrono>
#include <iostream>

// ros specific
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

typedef pcl::PointXYZRGB POINT;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(pcl::PointCloud<POINT>::Ptr _cloud, std::string topic, std::string frame)
    : Node("minimal_publisher"), count_(0), cloud_(_cloud), frame_(frame)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);
      timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*cloud_, *pc2_msg_);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      pc2_msg_->header.frame_id = frame_;
      pc2_msg_->header.stamp = now();
      publisher_->publish(*pc2_msg_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    size_t count_;
    pcl::PointCloud<POINT>::Ptr cloud_;
    std::string frame_;
};

/*
pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
 {
   // --------------------------------------------
   // -----Open 3D viewer and add point cloud-----
   // --------------------------------------------
   pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0, 0, 0);
   viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
   viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters ();
   return (viewer);
 }
*/
int main (int argc, char** argv)
{

    std::string fp = "/home/hunter/PiH_ws/src/template_matching3d/data/";
    if(argc > 1){
        fp = fp + argv[1];
    } else{
        fp = fp + "milk_cartoon_all_small_clorox.pcd";
    }
    cout << fp << endl;
    std::string topic = "point_cloud";
    if(argc > 2){
        topic = argv[2];
    }
    std::string frame = "map";
    if(argc > 3){
        frame = argv[3];
    }
    pcl::PointCloud<POINT>::Ptr cloud (new pcl::PointCloud<POINT>);

    if (pcl::io::loadPCDFile<POINT> (fp, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    cloud->is_dense = false;
    for(long unsigned int i=0; i < cloud->points.size(); i++)
    {
      if(!pcl::isFinite(cloud->points[i]))
      {
        cloud->points[i].x = 0;
        cloud->points[i].y = 0;
        cloud->points[i].z = 5;
      }
    }

    if(argc > 1)
    {
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      for(long unsigned int i = 0; i < cloud->points.size(); i ++)
      {
        POINT pt = cloud->points[i];
        x += pt.x;
        y += pt.y;
        z += pt.z;
      }
      x /= cloud->points.size();
      y /= cloud->points.size();
      z /= cloud->points.size();
      for(long unsigned int i = 0; i < cloud->points.size(); i++)
      {
        cloud->points[i].x -= x;
        cloud->points[i].y -= y;
        cloud->points[i].z -= z;
      }
    }
    //std::cout << "Loaded "
    //          << cloud->width * cloud->height
    //          << " data points from test_pcd.pcd with the following fields: "
    //          << std::endl;
    //for (const auto& point: *cloud)
    //  std::cout << "    " << point.x
    //            << " "    << point.y
    //            << " "    << point.z << std::endl;


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>(cloud, topic, frame));
    rclcpp::shutdown();
    /*
        pcl::visualization::PCLVisualizer::Ptr viewer;
        viewer = simpleVis(cloud);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }*/
    return (0);
}