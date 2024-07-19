
#include "canfd.h"
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h"
#include "math.h"
#include <sensor_msgs/PointCloud2.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/point_types.h>  

using namespace std;
using std::cout;  
using std::endl;  



int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"canfd_data_rx");
    ros::NodeHandle nh;
    std::cout << "start receive!\n";
    ros::Publisher pub = nh.advertise< sensor_msgs::PointCloud2 >("CanFDData", 1000);  
    // 加载so
    if (load_so_dll() != 0)
    {
        return -1;
    }
    // 打开设备
    DEVICE_HANDLE dev_handle = zcan_open_device(USBCANFD_200U, device_index, 0);
    if (dev_handle == INVALID_DEVICE_HANDLE)
    {
        cout << "打开设备失败" << endl;
        return -1;
    }
    cout << "打开设备成功" << endl;
    m_h_device[device_index] = dev_handle;

    /*设置波特率*/
    IProperty * _pPro=GetIProperty(dev_handle);
    const char *str;
    if (_pPro==NULL)
    {
        cout << "IProperty NULL" << endl;
    }
    if(STATUS_OK!=zcan_set_abit_baud(m_h_device[device_index], 0,1000000))
    {
        cout << "set baudA failed" << endl;
        ReleaseIProperty(_pPro);
    }
    if(STATUS_OK!=zcan_set_dbit_baud(m_h_device[device_index], 0,4000000))
    {
        cout << "set baudD failed" << endl;
        ReleaseIProperty(_pPro);
    }
    
    cout << "初始化通道1" << endl;
    UINT ret = initCAN(0);
    if (ret != 0)
    {
        cout << "初始化通道1失败" << endl;
        return -1;
    }
    usleep(100000);
    
    cout << "清除通道1滤波设置" << endl;
    ret = clear_filter(0);
    usleep(100000);
      
    cout << "启动通道1" << endl;
    ret = start_can(0);
    usleep(100000);

    ZCAN_Receive_Data CanObj[2500];
    ZCAN_ReceiveFD_Data CanFDObj[2500];
    UINT ReadLen,ReadLenfd, num;
    uint data[8][8];  //用来存一条报文的数组

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZINormal>);  

    uint can_num=zcan_get_receive_num(m_h_channel[device_index][0],0); // 表示CAN模式
    cout << "缓冲区can报文num:" <<can_num<< endl;
    
    if( can_num)
    {
        ReadLen=0;
        ReadLen=zcan_receive(m_h_channel[device_index][0],CanObj,can_num,50);
        cout << "实际接收can报文num:" <<ReadLen<< endl; 
        can_num=0;
    }

    uint canfd_num=zcan_get_receive_num(m_h_channel[device_index][0],1); // 表示CANfd模式
    cout << "缓冲区canfd报文num:" <<canfd_num<< endl;
    while(canfd_num)
    {
        num++;
        ReadLenfd=zcan_receive_fd(m_h_channel[device_index][0],CanFDObj,canfd_num,50);
        ReadLenfd=ReadLenfd-can_num;
        cout << "接收canfd报文num:" <<ReadLenfd<< endl;
        for (int i = 0; i < ReadLenfd; i++)     //第i条报文，每条报文包含多个点（一般是64个0x**，8个点）
        { 
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);  
            // 添加一些点到点云中  
            cloud->width = ReadLenfd*8; // 点的数量  
            cloud->height = 1; // 单层点云  
            cloud->is_dense = true; // 假设没有无效的点  
            cloud->points.resize(cloud->width * cloud->height);  
            if(GET_ID(CanFDObj[i].frame.can_id==0x701))      //第i条报文
            {
                printf(" TimeStamp:0x%08X", CanFDObj[i].timestamp);
                printf("    ");
                printf("CANFD_ID:0x%08X",  GET_ID(CanFDObj[i].frame.can_id));
                printf("    ");
                printf("DLC:0x%02X", CanFDObj[i].frame.len);
                printf("\n");
                for (int j=0; j < CanFDObj[i].frame.len; j++)   //第i条报文的第j个点。
                {
                    data[j/8][j%8]=CanFDObj[i].frame.data[j];  //data的每一行是一个点
                }
                float PId,VDis,HDis,RVel,High,MovAtt,SigNoiRatio,RDis,thea;
                POINTFD pointfd[300];
                for(int a = 0; a < 8; a++)  //第a个点
                {     
                    for(int b = 0; b < 8; b++) {    //解析第a个点的第b个0x**
                       switch (b)
                       {
                            case 0:
                            PId=data[a][b];
                            pointfd[a].ID=  PId;
                            break;
                            case 1:
                            VDis=(((data[a][b]*32) +(data[a][b+1]>>3))*0.05)-100;
                            pointfd[a].VDis=VDis;
                            break;
                            case 2:
                            HDis=((((data[a][b] &0x07)*256) + (data[a][b+1])) *0.05)-50;
                            pointfd[a].HDis=HDis;
                            break;
                            case 4:
                            RVel=(((data[a][b]*4)+(data[a][b+1]>>6))*0.05)-16;
                            pointfd[a].RVel=RVel;
                            break;
                            case 5:
                            High=((((data[a][b]&0x3F)*8)+(data[a][b+1]>>5))*0.25)-64;
                            pointfd[a].High=High;
                            break;
                            case 6:
                            MovAtt=(data[a][b]&0x07);
                            pointfd[a].MovArr=MovAtt;
                            break;
                            case 7:
                            SigNoiRatio=data[a][b];
                            pointfd[a].SigNoiRatio=SigNoiRatio;
                            break;
                            default:
                            break;
                       }
                    }  
                    RDis=sqrt((VDis*VDis)+(HDis*HDis));
                    thea=(atan(HDis/VDis))*(180.0 / M_PI); 
                    pointfd[a].RDis= RDis;
                    pointfd[a].thea= thea;
                     // 填充点云数据  
                    cloud->points[i*8+a].intensity=MovAtt;
                    cloud->points[i*8+a].normal_x=RDis;
                    cloud->points[i*8+a].normal_y=RVel;
                    cloud->points[i*8+a].x = HDis;
                    cloud->points[i*8+a].y = VDis; 
                    cloud->points[i*8+a].z = High;  
                    *cloud1+= *cloud ;
                }
            }
        }
        if ( num==3) // 将PCL点云转换为ROS PointCloud2消息
        {  
            sensor_msgs::PointCloud2 ros_cloud;  
            pcl::toROSMsg(*cloud1, ros_cloud);  
            ros_cloud.header.frame_id = "radar"; // 设置参考帧ID  
            pub.publish(ros_cloud);
            cloud1->clear(); // 清空点云
            num=0;
         }
        while (zcan_get_receive_num(m_h_channel[device_index][0],1)==0){}
    }        
    zcan_reset_can(m_h_channel[device_index][0]); // 复位CAN1通道1
    usleep(100000);                               // 延时100ms

    // 关闭设备
    zcan_close_device(m_h_device[device_index]);
    cout << "关闭设备" << endl;
    return 0;
}
