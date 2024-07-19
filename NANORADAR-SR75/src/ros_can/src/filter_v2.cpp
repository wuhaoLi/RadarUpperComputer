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
#include <pcl/filters/extract_indices.h>
#include <algorithm>    
#include <valarray>    
#include <vector>  
#include <math.h> 

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
    pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
    sor.setInputCloud(motional);  
    sor.setMeanK(80); // 点数    
    sor.setStddevMulThresh(0.2); //阈值  
    sor.setNegative(false); //去噪点  
    sor.filter(*filter);  

    pcl::RegionGrowing<pcl::PointXYZINormal, pcl::Normal> reg;
	  reg.setInputCloud(filter);
	  reg.setInputNormals(0);
	  reg.setMinClusterSize(34);      // 最小聚类大小
	  reg.setMaxClusterSize(10000);  // 最大聚类大小
	  //reg.setSearchMethod(pcl::search::Search<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>));
	  //reg.setNumberOfNeighbours(30);  // 相邻点的数量
	  pcl::search::Octree<pcl::PointXYZINormal>::Ptr octree(new pcl::search::Octree<pcl::PointXYZINormal>(0.2));
	  reg.setSearchMethod(octree);
	  reg.setSmoothnessThreshold(0.5 / 180.0 * M_PI); // 平滑度阈值
	  reg.setCurvatureThreshold(0.23); // 曲率阈值
	  reg.setResidualThreshold(180);// 强度残差阈值

	  std::vector<pcl::PointIndices> cluster;
	  reg.extract(cluster);

	  for (std::size_t i = 0; i < cluster.size(); ++i) 
    {
		pcl::ExtractIndices<pcl::PointXYZINormal> extract;
		extract.setInputCloud(filter);
		extract.setIndices(boost::make_shared<const pcl::PointIndices>(cluster[i]));
		extract.filter(*person);
	  }

    for (uint i = 0; i < person->points.size(); ++i)
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