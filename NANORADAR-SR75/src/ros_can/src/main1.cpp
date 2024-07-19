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

using namespace std;

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

static void uti_unpack(unsigned char *pSrc, unsigned char *pDest, int len)
{
    unsigned char ch1, ch2;
    int i;
    for (i = 0; i < len; i++)
    {
        ch1 = (pSrc[i] & 0xF0) >> 4;
        ch2 = pSrc[i] & 0x0F;
        /*ch1 += ((ch1 > 9) ? 0x37 : 0x30);
        ch2 += ((ch2 > 9) ? 0x37 : 0x30);*/
        ch1 > 9 ? ch1 - 10 + 'A' : ch1 + '0';
        ch2 > 9 ? ch2 - 10 + 'A' : ch2 + '0';
        pDest[i * 2] = ch1;
        pDest[i * 2 + 1] = ch2;
    }
}
/*
static void uti_pack(unsigned char *pSrc, unsigned char *pDest, int len)
{
    char ch1, ch2;
    int i;
    for (i = 0; i < (len / 2); i++)
    {
        ch1 = pSrc[i * 2];
        ch2 = pSrc[i * 2 + 1];
        (ch1 >= 'a' && ch1 <= 'z'? (ch1 -= 32: (ch1);
        (ch2 >= 'a' && ch2 <= 'z'? (ch2 -= 32: (ch2);
        ch1 -= ((ch1 > '9') ? 0x37 : 0x30);
        ch2 -= ((ch2 > '9') ? 0x37 : 0x30);
        pDest[i] = (ch1 << 4) | ch2;
    }
}

void hex_num_to_string(int hex_num, char *out)
{
    if (out == nullptr)
    {
        return;
    }
    sprintf(out, "%x", hex_num);
}
*/
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
    //=1 表示单滤波，=0 表示双滤泄1�7
    zcan_channel_init_config.canfd.filter = filter;
    zcan_channel_init_config.canfd.mode = mode;
    zcan_channel_init_config.canfd.brp = 0;

    CHANNEL_HANDLE channel_handle = zcan_init_can(m_h_device[device_index], can_index, &zcan_channel_init_config);
    if (channel_handle == INVALID_CHANNEL_HANDLE)
    {
        cout << "初始化��道失败＄1�7" << endl;
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
        cout << "启动CAN失败＄1�7" << endl;
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

/*
int message_count = 0;

void *receive_func(void *param) // 接收线程〄1�7
{
    int *run = (int *)param; // 线程启动，���出控制��1�7
    int wait_time = 100;
    ZCAN_ReceiveFD_Data zcan_receive_fd_data[3000];
    int can_index = 0;

    while ((*run) & 0x0f)
    {
        int real_len = zcan_receive_fd(m_h_channel[device_index][can_index], zcan_receive_fd_data, 3000, wait_time);
        if (real_len > 0)
        {
	    int printf_len = 0;
            for (int i = 0; i < real_len; i++)
            {
                printf("Index:%04d  ", message_count);
                message_count++;

                printf("CANFD%d RX ID:0x%08X", can_index + 1, GET_ID(zcan_receive_fd_data[i].frame.can_id));
		printf_len = 35;
                if (IS_EFF(zcan_receive_fd_data[i].frame.can_id== 0)
                {
                    // 帧格式：标准帄1�7
                    printf(" Standard ");
		    printf_len += 10;
                }
                if (IS_EFF(zcan_receive_fd_data[i].frame.can_id) == 1)
                {
                    // 帧格式：扩展帄1�7
                    printf(" Extend   ");
	            printf_len += 10;
                }
                if (IS_RTR(zcan_receive_fd_data[i].frame.can_id) == 0)
                {
                    // 帧类型：数据帄1�7
                    printf(" Data   ");
                    printf_len += 10;
                }
                if (IS_RTR(zcan_receive_fd_data[i].frame.can_id== 1)
                {
                    printf(" Remote ");
                    printf_len += 8;
                }
                printf("DLC:0x%02X", zcan_receive_fd_data[i].frame.len);
		printf_len += 8;
                printf(" data:(0x)"); // 数据
                printf_len += 8;
                for (int j = 0; j < zcan_receive_fd_data[i].frame.len; j++)
                {
		    
                    printf(" %02X", zcan_receive_fd_data[i].frame.data[j]);
	            if ((j + 1% 8 == 0 )
		    {
			printf("\n");
                        printf("%*s", printf_len, "");
                    }
		    
                }
                printf(" TimeStamp:0x%08X", zcan_receive_fd_data[i].timestamp); // 时间标识〄1�7
                printf("\n");
            }
        }
        can_index = !can_index;
    }
    printf("run thread exit\n"); // 逢�出接收线稄1�7
    pthread_exit(0);
}*/

int main(int argc, char *argv[])
{
   setlocale(LC_ALL,"");
//1.初始匄1�7 ROS 节点
   ros::init(argc,argv,"canfd_data_rx");
   ros::NodeHandle nh;
    std::cout << "start receive!\n";
    ros::Publisher pub = nh.advertise< std_msgs::UInt8MultiArray >("CanFDData", 1000);  

    // 动��加载so，函数指钄1�7
    if (load_so_dll() != 0)
    {
        return -1;
    }

    // 打开设备
    DEVICE_HANDLE dev_handle = zcan_open_device(USBCANFD_200U, device_index, 0);
    if (dev_handle == INVALID_DEVICE_HANDLE)
    {
        cout << "打开设备失败＄1�7" << endl;
        return -1;
    }
    cout << "打开设备成功＄1�7" << endl;
    m_h_device[device_index] = dev_handle;

   /*设置波特玄1�7*/
    IProperty * _pPro=GetIProperty(dev_handle);
    const char *str;
    if (_pPro==NULL)
    {
        cout << "IProperty NULL＄1�7" << endl;
    }
   /* if(STATUS_OK!=_pPro->SetValue("0/canfd_abit_baud_rate","1000000"))*/
   if(STATUS_OK!=zcan_set_abit_baud(m_h_device[device_index], 0,1000000))
    {
        cout << "set baudA failed＄1�7" << endl;
        ReleaseIProperty(_pPro);
    }
   if(STATUS_OK!=zcan_set_dbit_baud(m_h_device[device_index], 0,4000000))
    {
        cout << "set baudD failed＄1�7" << endl;
        ReleaseIProperty(_pPro);
    }
    /*  
    // 获取设备信息
    ZCAN_DEVICE_INFO device_info;

    UINT ret = zcan_get_device_inf(dev_handle, &device_info);

    cout << "zcan_get_device_inf return:" << ret << endl;

    if (ret != STATUS_OK)
    {
        cout << "获取设备信息失败:" << ret << endl;
        return -1;
    }
    cout << "设备信息:" << endl;
    printf("hw_Version:%x\n", device_info.hw_Version);
    printf("fw_Version:%x\n", device_info.fw_Version);
    printf("dr_Version:%x\n", device_info.dr_Version);
    printf("in_Version:%x\n", device_info.in_Version);
    printf("irq_Num:%d\n", device_info.irq_Num);
    printf("can_Num:%d\n", device_info.can_Num);
    printf("str_Serial_Num:%s\n", device_info.str_Serial_Num);
    printf("str_hw_Type:%s\n", device_info.str_hw_Type);
*/
    cout << "初始化��道1" << endl;
     UINT ret = initCAN(0);
    if (ret != 0)
    {
        cout << "初始化��道1失败＄1�7" << endl;
        return -1;
    }
    usleep(100000);
     /* 
    ret = initCAN(1);
    if (ret != 0)
    {
        cout << "初始化��道2失败＄1�7" << endl;
        return -1;
    }
    usleep(100000);
   */
    cout << "清除通道1滤波设置" << endl;
    ret = clear_filter(0);
    usleep(100000);
      /* 
    cout << "清除通道2滤波设置" << endl;
    ret = clear_filter(1);
    usleep(100000);
  */
    // cout << "清除通道1滤波模式" << endl;
    // ret = set_filter_mode(0);
    // usleep(100000);
    // cout << "清除通道2滤波模式" << endl;
    // ret = set_filter_mode(1);
    // usleep(100000);
 /*
     int start_id = 0x100;
     cout << "设置通道1滤波起始id" << endl;
     ret = set_filter_start_id(0, start_id);
     usleep(100000);
    // cout << "设置通道2滤波起始id" << endl;
    // ret = set_filter_start_id(1, start_id);
    // usleep(100000);

     int end_id = 0x200;
     cout << "设置通道1滤波结束id" << endl;
     ret = set_filter_end_id(0, end_id);
     usleep(100000);
    // cout << "设置通道2滤波结束id" << endl;
    // ret = set_filter_end_id(1, end_id);
    // usleep(100000);

    cout << "生效通道1滤波设置" << endl;
    ret = ack_filter(0);
    usleep(100000);
*/
    /*
    cout << "生效通道2滤波设置" << endl;
    ret = ack_filter(1);
    usleep(100000);
    */
    cout << "启动通道1" << endl;
    ret = start_can(0);
    usleep(100000);
    /*
    cout << "启动通道2" << endl;
    ret = start_can(1);
    usleep(100000);
   */
 ZCAN_Receive_Data CanObj[2500];
 ZCAN_ReceiveFD_Data CanFDObj[2500];
 BYTE data_unpack[128] ;
 //while(STATUS_OK)
 //{}
    uint can_num=zcan_get_receive_num(m_h_channel[device_index][0],0); // 表示CAN模式
    cout << "缓冲区can报文数:" <<can_num<< endl;
    if( can_num)
       {
        UINT ReadLen=0;
        ReadLen=zcan_receive(m_h_channel[device_index][0],CanObj,can_num,50);
        cout << "实际接收can报文数:" <<ReadLen<< endl; 
        can_num=0;
       }

    uint canfd_num=zcan_get_receive_num(m_h_channel[device_index][0],1); // 表示CANfd模式
    cout << "缓冲区canfd报文数:" <<canfd_num<< endl;
   
     while(canfd_num)
    {
        UINT ReadLenfd=zcan_receive_fd(m_h_channel[device_index][0],CanFDObj,canfd_num,50);
        cout << "接收canfd报文数:" <<ReadLenfd<< endl;
         for (int i = 0; i < ReadLenfd; i++)
         {
            printf(" TimeStamp:0x%08X", CanFDObj[i].timestamp);
            printf("CANFD_ID:0x%08X",  GET_ID(CanFDObj[i].frame.can_id));
            printf("DLC:0x%02X", CanFDObj[i].frame.len);
            printf("\n");
            for (uint8_t j=0; j < CanFDObj[i].frame.len; j++)
                {
                       //std::cout << std::hex << std::uppercase << data[j]<<" "; 
                    printf(" %02X", CanFDObj[j].frame.data[i]);
                }
            printf("\n");
         }
         usleep(100000);
          /*while(1)
         {
            canfd_num=zcan_get_receive_num(m_h_channel[device_index][0],1);
            if(canfd_num>0)
               break;
         }*/
    }        
       /*zcan_clear_buffer(m_h_channel[device_index][0]); 
        canfd_num=0;*/ 
    
        //cout << "CanFDObj->frame.len=" <<CanFDObj->frame.len<< endl;
         /*
         std::cout << std::hex << std::uppercase <<CanFDObj->frame.can_id<< std::endl; 
    //  uti_unpack(CanFDObj->frame.len,data_unpack,ReadLenfd);
         
        std_msgs::UInt8MultiArray   msg;
         for(int i=0;i<128;i++)
           {
            msg.data[i]=data_unpack[i];
           }
        pub.publish(msg); */

 

 /*
    int run_flag = 1;
    pthread_t thread_id;
   ret = pthread_create(&thread_id, nullptr, receive_func, &run_flag);
    cout << "数据接收线程已启劄1�7" << endl;
*/

   /*
    cout << "通道1发��数捄1�7" << endl;
    int message_len = 1;
    char hex_string[1024] = {0};
    strcpy(hex_string, "1122334455667788");

    ZCAN_TransmitFD_Data zcan_transmitfd_data[message_len];
    int eff = 0;
    int rtr = 0;
    int id = 0x00;
    zcan_transmitfd_data[0].frame.can_id = MAKE_CAN_ID(id, eff, rtr, 0);
    zcan_transmitfd_data[0].frame.len = 64;
    memset(zcan_transmitfd_data[0].frame.data, 0, sizeof(zcan_transmitfd_data[0].frame.data));
    for (int i = 0; i < 64; i++)
    {
        zcan_transmitfd_data[0].frame.data[i] = i;
    }

    int times = 5;
    int printf_len = 0;
    while (times--)
    {
        if (zcan_transmit_fd(m_h_channel[device_index][0], zcan_transmitfd_data, message_len== 1)
        {
            printf("Index:%04d  ", message_count);
            message_count++;
            printf("CANFD1 TX ID:0x%08X", id);
	    printf_len = 34;
            if (IS_EFF(zcan_transmitfd_data[0].frame.can_id== 0)
            {
                printf(" Standard ");
                printf_len += 10;
            }
            if (IS_EFF(zcan_transmitfd_data[0].frame.can_id) == 1)
            {
                printf(" Extend   ");
		printf_len += 10;
            }
            if (IS_RTR(zcan_transmitfd_data[0].frame.can_id== 0)
            {
                printf(" Data   ");
		printf_len += 10;
            }
            if (IS_RTR(zcan_transmitfd_data[0].frame.can_id) == 1)
            {
                printf(" Remote ");
		printf_len += 9;
            }
            printf("DLC:0x%02X", zcan_transmitfd_data[0].frame.len);
	    printf_len += 8;
            printf(" data:(0x)");
            printf_len += 9;

            for (int i = 0; i < zcan_transmitfd_data[0].frame.len; i++)
            {
                printf(" %02X", zcan_transmitfd_data[0].frame.data[i]);
		if ((i + 1) % 8 == 0 )
		{
		   printf("\n");
                   printf("%*s", printf_len, "");
                }
            }
            printf("\n");
            id += 1;
            zcan_transmitfd_data[0].frame.can_id = MAKE_CAN_ID(id, eff, rtr, 0);
        }
        else
        {
            break;
        }

        usleep(100000 * 5);

        if (zcan_transmit_fd(m_h_channel[device_index][1], zcan_transmitfd_data, message_len) == 1)
        {
            printf("Index:%04d  ", message_count);
            message_count++;
            printf("CANFD2 TX ID:0x%08X", id);
            if (IS_EFF(zcan_transmitfd_data[0].frame.can_id) == 0)
            {
                printf(" Standard ");
            }
            if (IS_EFF(zcan_transmitfd_data[0].frame.can_id== 1)
            {
                printf(" Extend   ");
            }
            if (IS_RTR(zcan_transmitfd_data[0].frame.can_id) == 0)
            {
                printf(" Data   ");
            }
            if (IS_RTR(zcan_transmitfd_data[0].frame.can_id== 1)
            {
                printf(" Remote ");
            }
            printf("DLC:0x%02X", zcan_transmitfd_data[0].frame.len);
            printf(" data:(0x)");

            for (int i = 0; i < zcan_transmitfd_data[0].frame.len; i++)
            {
                printf(" %02X", zcan_transmitfd_data[0].frame.data[i]);
		if ((i + 1% 8 == 0 )
		{
		   printf("\n");
                   printf("%*s", printf_len, "");
                }
            }
            printf("\n");
            id += 1;
            zcan_transmitfd_data[0].frame.can_id = MAKE_CAN_ID(id, eff, rtr, 0);
        }
        else
        {
            break;
        }
        usleep(100000);
    }
    */
  
    /*
    run_flag = 0;                  // 线程关闭指令〄1�7
    pthread_join(thread_id, NULL); // 等待线程关闭〄1�7
    usleep(100000);                // 延时100ms〄1�7
    */
    zcan_reset_can(m_h_channel[device_index][0]); // 复位CAN1通道〄1�7
    usleep(1000);                               // 延时100ms〄1�7
/*
    zcan_reset_can(m_h_channel[device_index][1]); // 复位CAN2通道〄1�7
    usleep(100000); // 延时100ms〄1�7
*/
    // 关闭设备
    zcan_close_device(m_h_device[device_index]);
    cout << "关闭设备" << endl;
    return 0;
}
