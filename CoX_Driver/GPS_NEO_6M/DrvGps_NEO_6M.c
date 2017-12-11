/**********************************************************************
 * 文件名称：DrvGps_NEO_6M.c
 * 文件说明：提供对ublox NEO 6M GPS模块的数据分析与gaps信息解码
 * |  创建时间      | 版本信息   |     创建人          |    联系方式                                |
 *  2013-11-01     0.0.1      ShawnFeng         FML927@163.com
**********************************************************************/
#include "stm32f0xx.h"
#include "DrvGps_NEO_6M.h"
#include "stm32f0xx_usart.h"
#include "stdio.h"
#include "string.h"
#include "math.h"


/*数据发送buffer，buffer长度跟最长需求定义*/
#define  BUFFER_LEN  (sizeof(_ublox_cfg_rate)>sizeof(_ublox_cfg_tp)?sizeof(_ublox_cfg_rate):sizeof(_ublox_cfg_tp))
char USART_TX_BUF[BUFFER_LEN] = {0};



/**********************************************************************
 * 函数名称：Gps_GetCommaPos
 * 函数说明：获取输入数据buffer中某个逗号的位置
 * 参数说明：u8 * pBuff  输入数据缓存首地址
 *          u8    cnt   待寻找逗号的编号（从1开始编号）
 * 返回值     ：对应编号的逗号在buffer中的位置，范围：0 ~ (Lenght(pBuff)-1)
 * 错误信息：返回值  == -1，输入数据buffer中有非法字符，无法寻找逗号
**********************************************************************/
int Gps_GetCommaPos(u8 *pBuff, u8 cnt)
{
	u8 *p=pBuff;

	//如果找到的个数完成则退出循环
	while(cnt)
	{
		if(*pBuff=='*'||*pBuff<' '||*pBuff>'z')
		{
			return -1; //遇到'*'或者非法字符,则找不到第cx个逗号，返回错误信息
		}
		//如果找到则计数减一
		if(*pBuff==',')
		{
			cnt--;
		}
		pBuff++;
	}
	//返回偏移位置
	return (pBuff-p);
}

/**********************************************************************
 * 函数名称：Gps_GetPow
 * 函数说明：获取m^n次方计算结果
 * 参数说明: u8 m  底数
 *          u8 n  次数
 * 返回值     ：计算结果
 * 错误信息：none
**********************************************************************/
u32 Gps_Pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--)result*=m;
	return result;
}

int Gps_Str2num(u8 *buf,u8*dx)
{
	u8 *p=buf;
	u32 ires=0,fres=0;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	int res;
	while(1) 
	{
		if(*p=='-'){mask|=0X02;p++;} 
		if(*p==','||(*p=='*'))break; 
		if(*p=='.'){mask|=0X01;p++;} 
		else if(*p>'9'||(*p<'0')) 
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	 
	for(i=0;i<ilen;i++)	 
	{  
		ires+=Gps_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5; 
	*dx=flen;	 	 
	for(i=0;i<flen;i++)	 
	{  
		fres+=Gps_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*Gps_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  	

/**********************************************************************
 * 函数名称：Gps_GPGSV_Analysis
 * 函数说明：分析GPGSV信息，根据分析结果将得到的gps数据填充信息结构体内
 * 参数说明：gpsx:nmea信息结构体
 *          buf:接收到的GPS数据缓冲区首地址
 * 返回值     ：无
 * 错误信息：无
**********************************************************************/
void Gps_GPGSV_Analysis(Gps_msg *gpsx,u8 *buf)
{
	u8 *p,*p1,dx;
	u8 len,i,j,slx=0;
	u8 posx;
	p=buf;
	p1=(u8*)strstr((const char *)p,"$GPGSV");
	if(!p1) return ;
	
	len=p1[7]-'0';								//得到GPGSV的条数
	posx=Gps_GetCommaPos(p1,3); 					//得到可见卫星总数
	if(posx!=0XFF)gpsx->svnum=Gps_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{
		p1=(u8*)strstr((const char *)p,"$GPGSV");
		for(j=0;j<4;j++)
		{
			posx=Gps_GetCommaPos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=Gps_Str2num(p1+posx,&dx);	//得到卫星编号
			else break;
			posx=Gps_GetCommaPos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=Gps_Str2num(p1+posx,&dx);//得到卫星仰角
			else break;
			posx=Gps_GetCommaPos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=Gps_Str2num(p1+posx,&dx);//得到卫星方位角
			else break;
			posx=Gps_GetCommaPos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=Gps_Str2num(p1+posx,&dx);	//得到卫星信噪比
			else break;
			slx++;
		}
 		p=p1+1;//切换到下一个GPGSV信息
	}
}

/**********************************************************************
 * 函数名称：Gps_GPGGA_Analysis
 * 函数说明：分析GPGGA信息，根据分析结果将得到的gps数据填充信息结构体内
 * 参数说明：gpsx:nmea信息结构体
 *          buf:接收到的GPS数据缓冲区首地址
 * 返回值     ：无
 * 错误信息：无
**********************************************************************/
void Gps_GPGGA_Analysis(Gps_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	p1=(u8*)strstr((const char *)buf,"$GPGGA");
	if(!p1) return ;
	posx=Gps_GetCommaPos(p1,6);								//得到GPS状态
	if(posx!=0XFF)gpsx->gpssta=Gps_Str2num(p1+posx,&dx);
	posx=Gps_GetCommaPos(p1,7);								//得到用于定位的卫星数
	if(posx!=0XFF)gpsx->posslnum=Gps_Str2num(p1+posx,&dx);
	posx=Gps_GetCommaPos(p1,9);								//得到海拔高度
	if(posx!=0XFF)gpsx->altitude=Gps_Str2num(p1+posx,&dx);
}

/**********************************************************************
 * 函数名称：Gps_GPGSA_Analysis
 * 函数说明：分析GPGSA信息，根据分析结果将得到的gps数据填充信息结构体内
 * 参数说明：gpsx:nmea信息结构体
 *          buf:接收到的GPS数据缓冲区首地址
 * 返回值     ：无
 * 错误信息：无
**********************************************************************/
void Gps_GPGSA_Analysis(Gps_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	u8 i;
	p1=(u8*)strstr((const char *)buf,"$GPGSA");
	if(!p1) return ;
	posx=Gps_GetCommaPos(p1,2);								//得到定位类型
	if(posx!=0XFF)gpsx->fixmode=Gps_Str2num(p1+posx,&dx);
	for(i=0;i<12;i++)										//得到定位卫星编号
	{
		posx=Gps_GetCommaPos(p1,3+i);
		if(posx!=0XFF)gpsx->possl[i]=Gps_Str2num(p1+posx,&dx);
		else break;
	}
	posx=Gps_GetCommaPos(p1,15);								//得到PDOP位置精度因子
	if(posx!=0XFF)gpsx->pdop=Gps_Str2num(p1+posx,&dx);
	posx=Gps_GetCommaPos(p1,16);								//得到HDOP位置精度因子
	if(posx!=0XFF)gpsx->hdop=Gps_Str2num(p1+posx,&dx);
	posx=Gps_GetCommaPos(p1,17);								//得到VDOP位置精度因子
	if(posx!=0XFF)gpsx->vdop=Gps_Str2num(p1+posx,&dx);
}

/**********************************************************************
 * 函数名称：Gps_GPRMC_Analysis
 * 函数说明：分析GPRMC信息，根据分析结果将得到的gps数据填充信息结构体内
 * 参数说明：gpsx:nmea信息结构体
 *          buf:接收到的GPS数据缓冲区首地址
 * 返回值     ：无
 * 错误信息：无
**********************************************************************/
void Gps_GPRMC_Analysis(Gps_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	u32 temp;
	float rs;
	p1=(u8*)strstr((const char *)buf,"$GPRMC");
	if(!p1) return ;
	posx=Gps_GetCommaPos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp=Gps_Str2num(p1+posx,&dx)/Gps_Pow(10,dx);	 	//得到UTC时间,去掉ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;
	}
	posx=Gps_GetCommaPos(p1,3);								//得到纬度
	if(posx!=0XFF)
	{
		temp=Gps_Str2num(p1+posx,&dx);
		gpsx->latitude=temp/Gps_Pow(10,dx+2);	//得到°
		rs=temp%Gps_Pow(10,dx+2);				//得到'
		gpsx->latitude=gpsx->latitude*Gps_Pow(10,5)+(rs*Gps_Pow(10,5-dx))/60;//转换为°
	}
	posx=Gps_GetCommaPos(p1,4);								//南纬还是北纬
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);
 	posx=Gps_GetCommaPos(p1,5);								//得到经度
	if(posx!=0XFF)
	{
		temp=Gps_Str2num(p1+posx,&dx);
		gpsx->longitude=temp/Gps_Pow(10,dx+2);	//得到°
		rs=temp%Gps_Pow(10,dx+2);				//得到'
		gpsx->longitude=gpsx->longitude*Gps_Pow(10,5)+(rs*Gps_Pow(10,5-dx))/60;//转换为°
	}
	posx=Gps_GetCommaPos(p1,6);								//东经还是西经
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);
	posx=Gps_GetCommaPos(p1,9);								//得到UTC日期
	if(posx!=0XFF)
	{
		temp=Gps_Str2num(p1+posx,&dx);		 				//得到UTC日期
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;
	}
}
//

/**********************************************************************
 * 函数名称：Gps_GPVTG_Analysis
 * 函数说明：分析GPVTG信息，根据分析结果将得到的gps数据填充信息结构体内
 * 参数说明：gpsx:nmea信息结构体
 *          buf:接收到的GPS数据缓冲区首地址
 * 返回值     ：无
 * 错误信息：无
**********************************************************************/
void Gps_GPVTG_Analysis(Gps_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	p1=(u8*)strstr((const char *)buf,"$GPVTG");
	if(!p1) return ;
	posx=Gps_GetCommaPos(p1,7);								//得到地面速率
	if(posx!=0XFF)
	{
		gpsx->speed=Gps_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=Gps_Pow(10,3-dx);	 	 		//确保扩大1000倍
	}
}

/**********************************************************************
 * 函数名称：GPS_Analysis
 * 函数说明：根据输入缓冲区的信息解码得到有效的gps信息
 *          结果存入结构体指针对应的结构体buffer
 * 参数说明：gpsx:  nmea信息结构体
 *			buf:   接收到的GPS数据缓冲区首地址
 * 返回值     ：无
 * 错误信息：无
**********************************************************************/
void GPS_Analysis(Gps_msg *gpsx,u8 *buf)
{
	Gps_GPGSV_Analysis(gpsx,buf);	//GPGSV解析
	Gps_GPGGA_Analysis(gpsx,buf);	//GPGGA解析
	Gps_GPGSA_Analysis(gpsx,buf);	//GPGSA解析
	Gps_GPRMC_Analysis(gpsx,buf);	//GPRMC解析
	Gps_GPVTG_Analysis(gpsx,buf);	//GPVTG解析
}

/**********************************************************************
 * 函数名称：Gps_CheckSum
 * 函数说明：GPS校验和数据计算
 * 参数说明：buf:数据缓存区首地址
 *          len:数据长度
 *          cka,ckb:两个校验结果.
 * 返回值     ：无
 * 错误信息：无
**********************************************************************/
void Gps_CheckSum(u8 *buf,u16 len,u8* cka,u8*ckb)
{
	u16 i;
	*cka=0;*ckb=0;
	for(i=0;i<len;i++)
	{
		*cka=*cka+buf[i];
		*ckb=*ckb+*cka;
	}
}


int Uart_SendBuffer(USART_TypeDef* USARTx, char * pBuff, int Len)
{
	int i=0;
	char data = *pBuff;
	while(i<Len)
	{
		USART_SendData( USARTx, data );
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
		pBuff++;
		data = *pBuff;
		i++;
	}
	
	return i;
}


/**********************************************************************
 * 函数名称：Ublox_Cfg_Tp
 * 函数说明：配置UBLOX NEO-6的时钟脉冲输出
 * 参数说明：interval:脉冲间隔
 *          length:脉冲宽度
 *          status:脉冲配置:1,高电平有效;0,关闭;-1,低电平有效.
 * 返回值     ：无
 * 错误信息：无
**********************************************************************/
void Ublox_Cfg_Tp(u32 interval,u32 length,signed char status)
{
	_ublox_cfg_tp *cfg_tp=(_ublox_cfg_tp *)USART_TX_BUF;
	cfg_tp->header=0X62B5;		//cfg header
	cfg_tp->id=0X0706;			//cfg tp id
	cfg_tp->dlength=20;			//数据区长度为20个字节.
	cfg_tp->interval=interval;	//脉冲间隔,us
	cfg_tp->length=length;		//脉冲宽度,us
	cfg_tp->status=status;	   	//时钟脉冲配置
	cfg_tp->timeref=0;			//参考UTC 时间
	cfg_tp->flags=0;			//flags为0
	cfg_tp->reserved=0;		 	//保留位为0
	cfg_tp->antdelay=820;    	//天线延时为820ns
	cfg_tp->rfdelay=0;    		//RF延时为0ns
	cfg_tp->userdelay=0;    	//用户延时为0ns
	Gps_CheckSum((u8*)(&cfg_tp->id),sizeof(_ublox_cfg_tp)-4,&cfg_tp->cka,&cfg_tp->ckb);
	
	Uart_SendBuffer(USART2,  (char*)USART_TX_BUF,  (int)sizeof(_ublox_cfg_tp));
	
}



/**********************************************************************
 * 函数名称：UBLOX_Cfg_Rate
 * 函数说明：配置UBLOX NEO-6的刷新速率
 * 参数说明：u32 measrate 测量时间间隔，单位为ms，最少不能小于200ms（5Hz）
 *          u8 reftime   参考时间，0=UTC Time；1=GPS Time（一般设置为1）
 * 返回值     ：错误信息
 * 错误信息：-1 设置参考时间小于200ms
**********************************************************************/
int UBLOX_Cfg_Rate(u32 measrate, u8 reftime)
{
	_ublox_cfg_rate *cfg_rate=(_ublox_cfg_rate *)USART_TX_BUF;
 	if(measrate<200)
 	{
 		return -1;		//如果小于200ms，返回错误信息-1
 	}
 	cfg_rate->header=0X62B5;	//cfg header
	cfg_rate->id=0X0806;	 	//cfg rate id
	cfg_rate->dlength=6;	 	//数据区长度为6个字节.
	cfg_rate->measrate=measrate;//脉冲间隔,us
	cfg_rate->navrate=1;		//导航速率（周期），固定为1
	cfg_rate->timeref=reftime; 	//参考时间为GPS时间
	Gps_CheckSum((u8*)(&cfg_rate->id),sizeof(_ublox_cfg_rate)-4,&cfg_rate->cka,&cfg_rate->ckb);

	Uart_SendBuffer(USART2,   (char*)USART_TX_BUF,  (int)sizeof(_ublox_cfg_rate));
	return  0;
}




