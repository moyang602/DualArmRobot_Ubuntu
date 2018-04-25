#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>   /* 文件控制定义*/
#include <termios.h> /* PPSIX 终端控制定义*/
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include "SerialComm.h"

struct CJY901 JY901;
int fd_JY901 = 0;
int fd_GPS = 0;
int set_serial(int fd,int nSpeed,int nBits,char nEvent,int nStop)
{
    struct termios newttys1,oldttys1;

     /*保存原有串口配置*/
     if(tcgetattr(fd,&oldttys1)!=0)
     {
          perror("Setupserial 1");
          return -1;
     }
     memset(&newttys1,0,sizeof(newttys1));/* 先将新串口配置清0 */
     newttys1.c_cflag|=(CLOCAL|CREAD ); /* CREAD 开启串行数据接收，CLOCAL并打开本地连接模式 */

     newttys1.c_cflag &=~CSIZE;/* 设置数据位 */
     /* 数据位选择 */
     switch(nBits)
     {
         case 7:
             newttys1.c_cflag |=CS7;
             break;
         case 8:
             newttys1.c_cflag |=CS8;
             break;
     }
     /* 设置奇偶校验位 */
     switch( nEvent )
     {
         case '0':  /* 奇校验 */
             newttys1.c_cflag |= PARENB;/* 开启奇偶校验 */
             newttys1.c_iflag |= (INPCK | ISTRIP);/*INPCK打开输入奇偶校验；ISTRIP去除字符的第八个比特  */
             newttys1.c_cflag |= PARODD;/*启用奇校验(默认为偶校验)*/
             break;
         case 'E':/*偶校验*/
             newttys1.c_cflag |= PARENB; /*开启奇偶校验  */
             newttys1.c_iflag |= ( INPCK | ISTRIP);/*打开输入奇偶校验并去除字符第八个比特*/
             newttys1.c_cflag &= ~PARODD;/*启用偶校验*/
             break;
         case 'N': /*无奇偶校验*/
             newttys1.c_cflag &= ~PARENB;
             break;
     }
     /* 设置波特率 */
    switch( nSpeed )
    {
        case 2400:
            cfsetispeed(&newttys1, B2400);
            cfsetospeed(&newttys1, B2400);
            break;
        case 4800:
            cfsetispeed(&newttys1, B4800);
            cfsetospeed(&newttys1, B4800);
            break;
        case 9600:
            cfsetispeed(&newttys1, B9600);
            cfsetospeed(&newttys1, B9600);
            break;
        case 115200:
            cfsetispeed(&newttys1, B115200);
            cfsetospeed(&newttys1, B115200);
            break;
        default:
            cfsetispeed(&newttys1, B9600);
            cfsetospeed(&newttys1, B9600);
            break;
    }
     /*设置停止位*/
    if( nStop == 1)/* 设置停止位；若停止位为1，则清除CSTOPB，若停止位为2，则激活CSTOPB */
    {
        newttys1.c_cflag &= ~CSTOPB;/*默认为一位停止位； */
    }
    else if( nStop == 2)
    {
        newttys1.c_cflag |= CSTOPB;/* CSTOPB表示送两位停止位 */
    }

    /* 设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时*/
    newttys1.c_cc[VTIME] = 0;/* 非规范模式读取时的超时时间；*/
    newttys1.c_cc[VMIN]  = 0; /* 非规范模式读取时的最小字符数*/
    tcflush(fd ,TCIFLUSH);/* tcflush清空终端未完成的输入/输出请求及数据；TCIFLUSH表示清空正收到的数据，且不读取出来 */

     /*激活配置使其生效*/
    if((tcsetattr( fd, TCSANOW,&newttys1))!=0)
    {
        perror("com set error");
        exit(1);
    }

    return 0;
}

int JY901_analyse (unsigned char ucData[],int usLength)
{
    static unsigned char chrTemp[2000];
    static unsigned char ucRxCnt = 0;
    static unsigned short usRxLength = 0;


    memcpy(chrTemp,ucData,usLength);
    usRxLength += usLength;
    int rtn = 0;
    while (usRxLength >= 11)
    {
        if (chrTemp[0] != 0x55)
        {
            usRxLength--;
            memcpy(&chrTemp[0],&chrTemp[1],usRxLength);
            continue;
        }
        switch(chrTemp[1])
        {
            case 0x50:  memcpy(&JY901.stcTime,&chrTemp[2],8);break;
            case 0x51:  memcpy(&JY901.stcAcc,&chrTemp[2],8);break;
            case 0x52:  memcpy(&JY901.stcGyro,&chrTemp[2],8);break;
            case 0x53:  memcpy(&JY901.stcAngle,&chrTemp[2],8);break;
            case 0x54:  memcpy(&JY901.stcMag,&chrTemp[2],8);break;
            case 0x55:  memcpy(&JY901.stcDStatus,&chrTemp[2],8);break;
            case 0x56:  memcpy(&JY901.stcPress,&chrTemp[2],8);break;
            case 0x57:  memcpy(&JY901.stcLonLat,&chrTemp[2],8);break;
            case 0x58:  memcpy(&JY901.stcGPSV,&chrTemp[2],8);break;
        }
        usRxLength -= 11;
        memcpy(&chrTemp[0],&chrTemp[11],usRxLength);
        rtn ++;
    }
    return rtn;
}

int JY901_init()
{
    char *dev_name="/dev/ttyUSB0";

    if((fd_JY901=open(dev_name,O_RDWR|O_NOCTTY|O_NDELAY))<0)
    {
        perror("Can't Open the ttyUSB0 Serial Port");
        return -1;
    }
    printf("The ttyUSB0 Serial Port open successfully!\n");
    set_serial(fd_JY901,9600,8,'N',1);
    return 0;
}

int JY901_GetData(double Posture[3])
{
    int n = 0;
    int rtn = 0;
    unsigned char buff[JY901_LEN];
    if((n=read(fd_JY901,buff,sizeof(buff)))<0)
    {
       perror("read error\n");
       Posture[0] = 0.0;
       Posture[1] = 0.0;
       Posture[2] = 0.0;
       return -1;
    }
    //printf("n = %d\n",n);
    //printf("buff:%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",buff[0],buff[1],buff[2],buff[3],buff[4],buff[5],buff[6],buff[7],buff[8],buff[9],buff[10]);
    memset(&JY901, 0 , sizeof(JY901));
    rtn = JY901_analyse (buff,n);
    //printf("Angle:%.3lf %.3lf %.3lf\r\n",(double)JY901.stcAngle.Angle[0]/32768*180,(double)JY901.stcAngle.Angle[1]/32768*180,(double)JY901.stcAngle.Angle[2]/32768*180);
    Posture[0] = (double)JY901.stcAngle.Angle[0]/32768*180;
    Posture[1] = (double)JY901.stcAngle.Angle[1]/32768*180;
    Posture[2] = (double)JY901.stcAngle.Angle[2]/32768*180;
    return rtn;
}

int JY901_end()
{
    close(fd_JY901);
    return 0;
}


int gps_analyse (char *buff,GPRMC *gps_data)
{
    char *ptr=NULL;
     if(gps_data==NULL)
      {
         return -1;
      }
      if(strlen(buff)<10)
      {
         return -1;
      }
/* 如果buff字符串中包含字符"$GPRMC"则将$GPRMC的地址赋值给ptr */
      if(NULL==(ptr=strstr(buff,"$GPRMC")))
      {
         return -1;
      }
/* sscanf函数为从字符串输入，意思是将ptr内存单元的值作为输入分别输入到后面的结构体成员 */
      sscanf(ptr,"$GPRMC,%d.000,%c,%f,N,%f,E,%f,%f,%d,,,%c*",&(gps_data->time),&(gps_data->pos_state),&(gps_data->latitude),&(gps_data->longitude),&(gps_data->speed),&(gps_data->direction),&(gps_data->date),&(gps_data->mode));
      return 0;
}

int GPS_init()
{
    char *dev_name="/dev/ttyUSB1";

    if((fd_GPS=open(dev_name,O_RDWR|O_NOCTTY|O_NDELAY))<0)
    {
        perror("Can't Open the ttyUSB1 Serial Port");
        return -1;
    }
    printf("The ttyUSB1 Serial Port open successfully!\n");
    set_serial(fd_GPS,9600,8,'N',1);
    return 0;
}

int GPS_GetData(double *latitude, double *longitude)
{
    int n = 0;
    int rtn = 0;
    unsigned char buff[GPS_LEN];
    GPRMC gprmc;
    if((n=read(fd_GPS,buff,sizeof(buff)))<0)
    {
       perror("read error\n");
       *latitude = 0.0;
       *longitude = 0.0;
       return -1;
    }
    printf("n = %d\n",n);
    memset(&gprmc, 0 , sizeof(gprmc));
    gps_analyse(buff,&gprmc);
    *latitude = gprmc.latitude;
    *longitude = gprmc.longitude;
    return rtn;
}

int GPS_end()
{
    close(fd_GPS);
    return 0;
}