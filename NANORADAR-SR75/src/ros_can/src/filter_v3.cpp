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
    sor.setStddevMulThresh(0.25); //阈��1�7  
    sor.setNegative(false); //去噪炄1�7  
    sor.filter(*filter);  

    std::vector<pcl::PointIndices> cluster; 
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZINormal>);
	  kdtree->setInputCloud(filter);
	  pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
	  ec.setClusterTolerance(0.3); // 设置欧氏聚类的容差，聚类时允许的点之间的朢�大距禄1�7
	  ec.setMinClusterSize(40);    // 设置聚类的最小点敄1�7
	  ec.setMaxClusterSize(200);  // 设置聚类的最大点敄1�7
	  ec.setSearchMethod(kdtree);
	  ec.setInputCloud(filter);
	  ec.extract(cluster);

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