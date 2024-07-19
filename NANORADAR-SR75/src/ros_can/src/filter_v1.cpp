#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>	 
#include <pcl/filters/project_inliers.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>  
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/region_growing.h>  
#include <pcl/segmentation/extract_clusters.h>  

ros::Publisher pub;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr motional(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr person(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr filter(new pcl::PointCloud<pcl::PointXYZINormal>);
  cloud.reset (new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::fromROSMsg (*input, *cloud); 

  for (uint i = 0; i <cloud->points.size(); i++)
  {
    if(abs(cloud->points[i].normal_y)>0 && cloud->points[i].intensity==0)
    {
      motional->points.push_back(cloud->points[i]); 
    }
             
  }

  if ( motional->empty()==0) 
  { 
    float rdis=0,dis=0; 
    /*
    pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
    sor.setInputCloud(motional);  
    sor.setMeanK(80); // 点数    
    sor.setStddevMulThresh(0.2); //阈值  
    sor.setNegative(false); //去噪点  
    sor.filter(*filter);  

    pcl::RadiusOutlierRemoval<pcl::PointXYZINormal> ror;  
    ror.setInputCloud(filter); 
    ror.setRadiusSearch(0.2);  
    ror.setMinNeighborsInRadius(20);  
    ror.filter(*person);   
    */
    std::vector<pcl::PointIndices> cluster; 
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZINormal>);
	  kdtree->setInputCloud(motional);
    // 创建欧氏聚类对象
	  pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
	  ec.setClusterTolerance(0.2); // 设置欧氏聚类的容差，聚类时允许的点之间的最大距离
	  ec.setMinClusterSize(10);    // 设置聚类的最小点数
	  ec.setMaxClusterSize(100);  // 设置聚类的最大点数
	  ec.setSearchMethod(kdtree);
	  ec.setInputCloud(motional);
	  ec.extract(cluster);
    
    for (uint i = 0; i <person->points.size(); i++)
    {
      rdis+=person->points[i].normal_x;
    }
    dis=rdis/(person->points.size());
    if(dis>0)
    {
      std::cout<<"RDis:"<<dis<<std::endl;
    }
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*person, output);
    output.header.frame_id = "radar"; 
    // Publish the data
    pub.publish (output); /**/
  } 
}
   

int main (int argc, char** argv)
{
  ros::init (argc, argv, "filter_v");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("CanFDData", 1, cloud_cb);
  pub= nh.advertise<sensor_msgs::PointCloud2> ("person", 1);    //
  ros::spin ();
}