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

ros::Publisher pub;



void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
        float dis;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane(new pcl::PointCloud<pcl::PointXYZINormal>);
         // Container for original & filtered data
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        cloud.reset (new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::fromROSMsg (*input, *cloud); 
       
        pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
        sor.setInputCloud(cloud);  
        sor.setMeanK(50); // 设置用于计算平均距离的最近邻点数    
        sor.setStddevMulThresh(1.0); // 设置标准差阈值，大于这个值的点被认为是噪声  
        sor.setNegative(false); // 如果设置为true，则保留被认为是噪声的点，去除其它点  
        sor.filter(*plane);   
        for (uint i = 0; i <plane->points.size(); i++)
        {
            dis+=plane->points[i].y;
        }
        std::cout<<dis/(plane->points.size())<<std::endl;
        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*plane, output);
        output.header.frame_id = "radar"; 
        // Publish the data
        pub.publish (output); /**/
        
}

int main (int argc, char** argv)
   {
        // Initialize ROS
        ros::init (argc, argv, "plane_dis");
        ros::NodeHandle nh;

        // Create a ROS subscriber for the input point cloud
        ros::Subscriber sub = nh.subscribe ("CanFDData", 1, cloud_cb);
        
        // Create a ROS publisher for the output point cloud
        pub= nh.advertise<sensor_msgs::PointCloud2> ("plane", 1);    //

        // Spin
        ros::spin ();
   }