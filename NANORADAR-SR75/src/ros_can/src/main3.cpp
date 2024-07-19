#include <iostream>
#include <sys/time.h>
#include <stdio.h>
#include <sstream>
#include <dlfcn.h>
#include <unistd.h>
#include "controlcanfd.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "math.h"
#include <sensor_msgs/PointCloud2.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/point_types.h>  

using namespace std;
using std::cout;  
using std::endl;  

typedef DEVICE_HANDLE (*pZCAN_OpenDevice)(UINT deviceType, UINT deviceIndex, UINT reserved);
typedef UINT (*pZCAN_CloseDevice)(DEVICE_HANDLE device_handle);
typedef UINT (*pZCAN_GetDeviceInf)(DEVICE_HANDLE device_handle, ZCAN_DEVICE_INFO *pInfo);
typedef UINT (*pZCAN_IsDeviceOnLine)(DEVICE_HANDLE device_handle);
typedef CHANNEL_HANDLE (*pZCAN_InitCAN)(DEVICE_HANDLE device_handle, UINT can_index, ZCAN_CHANNEL_INIT_CONFIG *pInitConfig);
typedef UINT (*pZCAN_StartCAN)(CHANNEL_HANDLE channel_handle);
typedef UINT (*pZCAN_ResetCAN)(CHANNEL_HANDLE channel_handle);
typedef UINT (*pZCAN_ClearBuffer)(CHANNEL_HANDLE channel_handle);
typedef UINT (*pZCAN_GetReceiveNum)(CHANNEL_HANDLE channel_handle, BYTE type); // type:TYPE_CAN TYPE_CANFD
typedef UINT (*pZCAN_Transmit)(CHANNEL_HANDLE channel_handle, ZCAN_Transmit_Data *pTransmit, UINT len);
typedef UINT (*pZCAN_Receive)(CHANNEL_HANDLE channel_handle, ZCAN_Receive_Data *pReceive, UINT len, int wait_time);
typedef UINT (*pZCAN_TransmitFD)(CHANNEL_HANDLE channel_handle, ZCAN_TransmitFD_Data *pTransmit, UINT len);
typedef UINT (*pZCAN_ReceiveFD)(CHANNEL_HANDLE channel_handle, ZCAN_ReceiveFD_Data *pReceive, UINT len, int wait_time);
typedef IProperty *(*pGetIProperty)(DEVICE_HANDLE device_handle);
typedef UINT (*pReleaseIProperty)(IProperty *pIProperty);
typedef UINT (*pZCAN_SetAbitBaud)(DEVICE_HANDLE device_handle, UINT can_index, UINT abitbaud);
typedef UINT (*pZCAN_SetDbitBaud)(DEVICE_HANDLE device_handle, UINT can_index, UINT dbitbaud);
typedef UINT (*pZCAN_SetCANFDStandard)(DEVICE_HANDLE device_handle, UINT can_index, UINT canfd_standard);
typedef UINT (*pZCAN_SetResistanceEnable)(DEVICE_HANDLE device_handle, UINT can_index, UINT enable);
typedef UINT (*pZCAN_SetBaudRateCustom)(DEVICE_HANDLE device_handle, UINT can_index, char *RateCustom);
typedef UINT (*pZCAN_ClearFilter)(CHANNEL_HANDLE channel_handle);
typedef UINT (*pZCAN_AckFilter)(CHANNEL_HANDLE channel_handle);
typedef UINT (*pZCAN_SetFilterMode)(CHANNEL_HANDLE channel_handle, UINT mode);
typedef UINT (*pZCAN_SetFilterStartID)(CHANNEL_HANDLE channel_handle, UINT start_id);
typedef UINT (*pZCAN_SetFilterEndID)(CHANNEL_HANDLE channel_handle, UINT EndID);

pZCAN_OpenDevice zcan_open_device;
pZCAN_CloseDevice zcan_close_device;
pZCAN_GetDeviceInf zcan_get_device_inf;
pZCAN_IsDeviceOnLine zcan_is_device_online;
pZCAN_InitCAN zcan_init_can;
pZCAN_StartCAN zcan_start_can;
pZCAN_ResetCAN zcan_reset_can;
pZCAN_ClearBuffer zcan_clear_buffer;
pZCAN_GetReceiveNum zcan_get_receive_num;
pZCAN_Transmit zcan_transmit;
pZCAN_Receive zcan_receive;
pZCAN_TransmitFD zcan_transmit_fd;
pZCAN_ReceiveFD zcan_receive_fd;
pGetIProperty get_iproperty;
pReleaseIProperty release_iproperty;
pZCAN_SetAbitBaud zcan_set_abit_baud;
pZCAN_SetDbitBaud zcan_set_dbit_baud;
pZCAN_SetCANFDStandard zcan_set_canfd_standard;
pZCAN_SetResistanceEnable zcan_set_resistance_enable;
pZCAN_SetBaudRateCustom zcan_set_baud_rate_custom;
pZCAN_ClearFilter zcan_clear_filter;
pZCAN_AckFilter zcan_ack_filter;
pZCAN_SetFilterMode zcan_set_filter_mode;
pZCAN_SetFilterStartID zcan_set_filter_start_id;
pZCAN_SetFilterEndID zcan_set_filter_end_id;

// so 句柄
void *m_h_ins_drv = nullptr;

// 设备句柄
DEVICE_HANDLE m_h_device[16];
// 通道句柄
CHANNEL_HANDLE m_h_channel[16][16];

int device_index = 0;
int device_type = 0;

int load_so_dll()
{
    char current_path[1024] = {0};
    getcwd(current_path, 1024);
    cout << "load_so_dll current_path:" << current_path << endl;
    char dll_path[512] = {0};
#ifdef __APPLE__
    sprintf(dll_path, "%s/libcontrolcanfd.dylib", current_path);
#else
    sprintf(dll_path, "%s/libcontrolcanfd.so", current_path);
#endif
    cout << "load_so_dll dll_path:" << dll_path << endl;
    m_h_ins_drv = dlopen(dll_path, RTLD_LAZY);
    if (!m_h_ins_drv)
    {
        printf("load_so_dll dlopen error:%s\n", dlerror());
        return -1;
    }
    else
    {
        zcan_open_device = (pZCAN_OpenDevice)dlsym(m_h_ins_drv, "ZCAN_OpenDevice");
        zcan_close_device = (pZCAN_CloseDevice)dlsym(m_h_ins_drv, "ZCAN_CloseDevice");
        zcan_get_device_inf = (pZCAN_GetDeviceInf)dlsym(m_h_ins_drv, "ZCAN_GetDeviceInf");
        zcan_is_device_online = (pZCAN_IsDeviceOnLine)dlsym(m_h_ins_drv, "ZCAN_IsDeviceOnLine");
        zcan_init_can = (pZCAN_InitCAN)dlsym(m_h_ins_drv, "ZCAN_InitCAN");
        zcan_start_can = (pZCAN_StartCAN)dlsym(m_h_ins_drv, "ZCAN_StartCAN");
        zcan_reset_can = (pZCAN_ResetCAN)dlsym(m_h_ins_drv, "ZCAN_ResetCAN");
        zcan_clear_buffer = (pZCAN_ClearBuffer)dlsym(m_h_ins_drv, "ZCAN_ClearBuffer");
        zcan_get_receive_num = (pZCAN_GetReceiveNum)dlsym(m_h_ins_drv, "ZCAN_GetReceiveNum");
        zcan_transmit = (pZCAN_Transmit)dlsym(m_h_ins_drv, "ZCAN_Transmit");
        zcan_receive = (pZCAN_Receive)dlsym(m_h_ins_drv, "ZCAN_Receive");
        zcan_transmit_fd = (pZCAN_TransmitFD)dlsym(m_h_ins_drv, "ZCAN_TransmitFD");
        zcan_receive_fd = (pZCAN_ReceiveFD)dlsym(m_h_ins_drv, "ZCAN_ReceiveFD");
        get_iproperty = (pGetIProperty)dlsym(m_h_ins_drv, "GetIProperty");
        release_iproperty = (pReleaseIProperty)dlsym(m_h_ins_drv, "ReleaseIProperty");
        zcan_set_abit_baud = (pZCAN_SetAbitBaud)dlsym(m_h_ins_drv, "ZCAN_SetAbitBaud");
        zcan_set_dbit_baud = (pZCAN_SetDbitBaud)dlsym(m_h_ins_drv, "ZCAN_SetDbitBaud");
        zcan_set_canfd_standard = (pZCAN_SetCANFDStandard)dlsym(m_h_ins_drv, "ZCAN_SetCANFDStandard");
        zcan_set_resistance_enable = (pZCAN_SetResistanceEnable)dlsym(m_h_ins_drv, "ZCAN_SetResistanceEnable");
        zcan_set_baud_rate_custom = (pZCAN_SetBaudRateCustom)dlsym(m_h_ins_drv, "ZCAN_SetBaudRateCustom");
        zcan_clear_filter = (pZCAN_ClearFilter)dlsym(m_h_ins_drv, "ZCAN_ClearFilter");
        zcan_ack_filter = (pZCAN_AckFilter)dlsym(m_h_ins_drv, "ZCAN_AckFilter");
        zcan_set_filter_mode = (pZCAN_SetFilterMode)dlsym(m_h_ins_drv, "ZCAN_SetFilterMode");
        zcan_set_filter_start_id = (pZCAN_SetFilterStartID)dlsym(m_h_ins_drv, "ZCAN_SetFilterStartID");
        zcan_set_filter_end_id = (pZCAN_SetFilterEndID)dlsym(m_h_ins_drv, "ZCAN_SetFilterEndID");
    }
    if (!zcan_open_device || !zcan_close_device || !zcan_get_device_inf || !zcan_is_device_online || !zcan_init_can || !zcan_start_can || !zcan_reset_can || !zcan_clear_buffer || !zcan_get_receive_num || !zcan_transmit || !zcan_receive || !zcan_transmit_fd || !zcan_receive_fd || !get_iproperty || !release_iproperty || !zcan_set_abit_baud || !zcan_set_dbit_baud || !zcan_set_canfd_standard || !zcan_set_resistance_enable || !zcan_set_baud_rate_custom || !zcan_clear_filter || !zcan_clear_filter || !zcan_ack_filter || !zcan_set_filter_mode || !zcan_set_filter_start_id || !zcan_set_filter_end_id)
    {
        printf("load_so_dll dlsym error:%s\n", dlerror);
        return -2;
    }
    return 0;
}

int initCAN(int can_index)
{
    int filter = 1;
    int mode = 0;
    int canType = 1;

    ZCAN_CHANNEL_INIT_CONFIG zcan_channel_init_config;
    zcan_channel_init_config.can_type = canType;

    zcan_channel_init_config.canfd.acc_code = 0;
    zcan_channel_init_config.canfd.acc_mask = 0xFFFFFFFF;
    //=1 表示单滤波，=0 表示双滤泄1�71ￄ1�771ￄ1�71ￄ1�777
    zcan_channel_init_config.canfd.filter = filter;
    zcan_channel_init_config.canfd.mode = mode;
    zcan_channel_init_config.canfd.brp = 0;

    CHANNEL_HANDLE channel_handle = zcan_init_can(m_h_device[device_index], can_index, &zcan_channel_init_config);
    if (channel_handle == INVALID_CHANNEL_HANDLE)
    {
        cout << "初始化失贄1�71ￄ1�77" << endl;
        return -1;
    }
    m_h_channel[device_index][can_index] = channel_handle;
    return 0;
}

int start_can(int can_index)
{
    int ret = zcan_start_can(m_h_channel[device_index][can_index]);
    if (ret != STATUS_OK)
    {
        cout << "启动CAN失败" << endl;
        return -1;
    }
    return 0;
}

int clear_filter(int can_index)
{
    int ret = zcan_clear_filter(m_h_channel[device_index][can_index]);
    if (ret != STATUS_OK)
    {
        cout << "清除滤波器失败！" << endl;
        return -1;
    }
    return 0;
}

int set_filter_mode(int can_index)
{
    int mode = 0;
    int ret = zcan_set_filter_mode(m_h_channel[device_index][can_index], mode);
    if (ret != STATUS_OK)
    {
        cout << "配置通道滤波模式" << endl;
        return -1;
    }
    return 0;
}

int set_filter_start_id(int can_index, int start_id)
{
    int mode = 0;
    int ret = zcan_set_filter_start_id(m_h_channel[device_index][can_index], mode);
    if (ret != STATUS_OK)
    {
        cout << "配置通道滤波起始id" << endl;
        return -1;
    }
    return 0;
}

int set_filter_end_id(int can_index, int end_id)
{
    int mode = 0;
    int ret = zcan_set_filter_end_id(m_h_channel[device_index][can_index], mode);
    if (ret != STATUS_OK)
    {
        cout << "配置通道滤波结束id" << endl;
        return -1;
    }
    return 0;
}

int ack_filter(int can_index)
{
    int mode = 0;
    int ret = zcan_ack_filter(m_h_channel[device_index][can_index]);
    if (ret != STATUS_OK)
    {
        cout << "生效通道滤波设置" << endl;
        return -1;
    }
    return 0;
}


int main(int argc, char *argv[])
{
   setlocale(LC_ALL,"");
   ros::init(argc,argv,"canfd_data_rx");
   ros::NodeHandle nh;
    std::cout << "start receive!\n";
    ros::Publisher pub = nh.advertise< std_msgs::UInt8MultiArray >("CanFDData", 1000);  

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

   /*设置波特玄1�71ￄ1�77*/
    IProperty * _pPro=GetIProperty(dev_handle);
    const char *str;
    if (_pPro==NULL)
    {
        cout << "IProperty NULL" << endl;
    }
   /* if(STATUS_OK!=_pPro->SetValue("0/canfd_abit_baud_rate","1000000"))*/
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
    
    cout << "初始化��道1" << endl;
     UINT ret = initCAN(0);
    if (ret != 0)
    {
        cout << "初始化��道1失败" << endl;
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
 uint data[8][8];  //用来存一条报文的数组



    uint can_num=zcan_get_receive_num(m_h_channel[device_index][0],0); // 表示CAN模式
    cout << "缓冲区can报文num:" <<can_num<< endl;
    if( can_num)
       {
        UINT ReadLen=0;
        ReadLen=zcan_receive(m_h_channel[device_index][0],CanObj,can_num,50);
        cout << "实际接收can报文num:" <<ReadLen<< endl; 
        can_num=0;
       }

    uint canfd_num=zcan_get_receive_num(m_h_channel[device_index][0],1); // 表示CANfd模式
    cout << "缓冲区canfd报文num:" <<canfd_num<< endl;
     while(canfd_num)
    {
        UINT ReadLenfd=zcan_receive_fd(m_h_channel[device_index][0],CanFDObj,canfd_num,50);
        ReadLenfd=ReadLenfd-can_num;
        cout << "接收canfd报文num:" <<ReadLenfd<< endl;
        
         for (int i = 0; i < ReadLenfd; i++)     //第i条报文，每条报文包含多个点（一般是64个0x**，8个点）
         { 
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
                float PId,VDis,HDis,RVel,High,MovArr,SigNoiRatio,RDis,thea;
                POINTFD pointfd[300];
                for(int a = 0; a < 8; a++) {      //第a个点
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
                           MovArr=(data[a][b]&0x07);
                           pointfd[a].MovArr=MovArr;
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
                    cout <<"PId:"<<pointfd[a].ID<<" ";   
                    cout << "HDis:"<<pointfd[a].HDis<<" ";
                    cout << "VDis:"<<pointfd[a].VDis<<" "; 
                    cout << "High:"<<pointfd[a].High<<" ";
                    cout << "RVel:"<<pointfd[a].RVel<<" "; 
                    cout << "SigNoiRatio:"<<pointfd[a].SigNoiRatio<<" ";
                    cout << "MovArr:"<<pointfd[a].MovArr<<" ";
                    cout << "RDis:"<<pointfd[a].RDis<<" ";
                    cout << "thea:"<<pointfd[a].thea<<" ";
                    cout << endl;   
                }
            }
        }
         usleep(100000);
    }        
       
    

    zcan_reset_can(m_h_channel[device_index][0]); // 复位CAN1通道1
    usleep(100000);                               // 延时100ms

    // 关闭设备
    zcan_close_device(m_h_device[device_index]);
    cout << "关闭设备" << endl;
    return 0;
}
