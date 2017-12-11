/**********************************************************************
 * �ļ����ƣ�DrvGps_NEO_6M.c
 * �ļ�˵�����ṩ��ublox NEO 6M GPSģ������ݷ�����gaps��Ϣ����
 * |  ����ʱ��      | �汾��Ϣ   |     ������          |    ��ϵ��ʽ                                |
 *  2013-11-01     0.0.1      ShawnFeng         FML927@163.com
**********************************************************************/
#include "stm32f0xx.h"
#include "DrvGps_NEO_6M.h"
#include "stm32f0xx_usart.h"
#include "stdio.h"
#include "string.h"
#include "math.h"


/*���ݷ���buffer��buffer���ȸ��������*/
#define  BUFFER_LEN  (sizeof(_ublox_cfg_rate)>sizeof(_ublox_cfg_tp)?sizeof(_ublox_cfg_rate):sizeof(_ublox_cfg_tp))
char USART_TX_BUF[BUFFER_LEN] = {0};



/**********************************************************************
 * �������ƣ�Gps_GetCommaPos
 * ����˵������ȡ��������buffer��ĳ�����ŵ�λ��
 * ����˵����u8 * pBuff  �������ݻ����׵�ַ
 *          u8    cnt   ��Ѱ�Ҷ��ŵı�ţ���1��ʼ��ţ�
 * ����ֵ     ����Ӧ��ŵĶ�����buffer�е�λ�ã���Χ��0 ~ (Lenght(pBuff)-1)
 * ������Ϣ������ֵ  == -1����������buffer���зǷ��ַ����޷�Ѱ�Ҷ���
**********************************************************************/
int Gps_GetCommaPos(u8 *pBuff, u8 cnt)
{
	u8 *p=pBuff;

	//����ҵ��ĸ���������˳�ѭ��
	while(cnt)
	{
		if(*pBuff=='*'||*pBuff<' '||*pBuff>'z')
		{
			return -1; //����'*'���߷Ƿ��ַ�,���Ҳ�����cx�����ţ����ش�����Ϣ
		}
		//����ҵ��������һ
		if(*pBuff==',')
		{
			cnt--;
		}
		pBuff++;
	}
	//����ƫ��λ��
	return (pBuff-p);
}

/**********************************************************************
 * �������ƣ�Gps_GetPow
 * ����˵������ȡm^n�η�������
 * ����˵��: u8 m  ����
 *          u8 n  ����
 * ����ֵ     ��������
 * ������Ϣ��none
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
 * �������ƣ�Gps_GPGSV_Analysis
 * ����˵��������GPGSV��Ϣ�����ݷ���������õ���gps���������Ϣ�ṹ����
 * ����˵����gpsx:nmea��Ϣ�ṹ��
 *          buf:���յ���GPS���ݻ������׵�ַ
 * ����ֵ     ����
 * ������Ϣ����
**********************************************************************/
void Gps_GPGSV_Analysis(Gps_msg *gpsx,u8 *buf)
{
	u8 *p,*p1,dx;
	u8 len,i,j,slx=0;
	u8 posx;
	p=buf;
	p1=(u8*)strstr((const char *)p,"$GPGSV");
	if(!p1) return ;
	
	len=p1[7]-'0';								//�õ�GPGSV������
	posx=Gps_GetCommaPos(p1,3); 					//�õ��ɼ���������
	if(posx!=0XFF)gpsx->svnum=Gps_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{
		p1=(u8*)strstr((const char *)p,"$GPGSV");
		for(j=0;j<4;j++)
		{
			posx=Gps_GetCommaPos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=Gps_Str2num(p1+posx,&dx);	//�õ����Ǳ��
			else break;
			posx=Gps_GetCommaPos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=Gps_Str2num(p1+posx,&dx);//�õ���������
			else break;
			posx=Gps_GetCommaPos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=Gps_Str2num(p1+posx,&dx);//�õ����Ƿ�λ��
			else break;
			posx=Gps_GetCommaPos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=Gps_Str2num(p1+posx,&dx);	//�õ����������
			else break;
			slx++;
		}
 		p=p1+1;//�л�����һ��GPGSV��Ϣ
	}
}

/**********************************************************************
 * �������ƣ�Gps_GPGGA_Analysis
 * ����˵��������GPGGA��Ϣ�����ݷ���������õ���gps���������Ϣ�ṹ����
 * ����˵����gpsx:nmea��Ϣ�ṹ��
 *          buf:���յ���GPS���ݻ������׵�ַ
 * ����ֵ     ����
 * ������Ϣ����
**********************************************************************/
void Gps_GPGGA_Analysis(Gps_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	p1=(u8*)strstr((const char *)buf,"$GPGGA");
	if(!p1) return ;
	posx=Gps_GetCommaPos(p1,6);								//�õ�GPS״̬
	if(posx!=0XFF)gpsx->gpssta=Gps_Str2num(p1+posx,&dx);
	posx=Gps_GetCommaPos(p1,7);								//�õ����ڶ�λ��������
	if(posx!=0XFF)gpsx->posslnum=Gps_Str2num(p1+posx,&dx);
	posx=Gps_GetCommaPos(p1,9);								//�õ����θ߶�
	if(posx!=0XFF)gpsx->altitude=Gps_Str2num(p1+posx,&dx);
}

/**********************************************************************
 * �������ƣ�Gps_GPGSA_Analysis
 * ����˵��������GPGSA��Ϣ�����ݷ���������õ���gps���������Ϣ�ṹ����
 * ����˵����gpsx:nmea��Ϣ�ṹ��
 *          buf:���յ���GPS���ݻ������׵�ַ
 * ����ֵ     ����
 * ������Ϣ����
**********************************************************************/
void Gps_GPGSA_Analysis(Gps_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	u8 i;
	p1=(u8*)strstr((const char *)buf,"$GPGSA");
	if(!p1) return ;
	posx=Gps_GetCommaPos(p1,2);								//�õ���λ����
	if(posx!=0XFF)gpsx->fixmode=Gps_Str2num(p1+posx,&dx);
	for(i=0;i<12;i++)										//�õ���λ���Ǳ��
	{
		posx=Gps_GetCommaPos(p1,3+i);
		if(posx!=0XFF)gpsx->possl[i]=Gps_Str2num(p1+posx,&dx);
		else break;
	}
	posx=Gps_GetCommaPos(p1,15);								//�õ�PDOPλ�þ�������
	if(posx!=0XFF)gpsx->pdop=Gps_Str2num(p1+posx,&dx);
	posx=Gps_GetCommaPos(p1,16);								//�õ�HDOPλ�þ�������
	if(posx!=0XFF)gpsx->hdop=Gps_Str2num(p1+posx,&dx);
	posx=Gps_GetCommaPos(p1,17);								//�õ�VDOPλ�þ�������
	if(posx!=0XFF)gpsx->vdop=Gps_Str2num(p1+posx,&dx);
}

/**********************************************************************
 * �������ƣ�Gps_GPRMC_Analysis
 * ����˵��������GPRMC��Ϣ�����ݷ���������õ���gps���������Ϣ�ṹ����
 * ����˵����gpsx:nmea��Ϣ�ṹ��
 *          buf:���յ���GPS���ݻ������׵�ַ
 * ����ֵ     ����
 * ������Ϣ����
**********************************************************************/
void Gps_GPRMC_Analysis(Gps_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	u32 temp;
	float rs;
	p1=(u8*)strstr((const char *)buf,"$GPRMC");
	if(!p1) return ;
	posx=Gps_GetCommaPos(p1,1);								//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		temp=Gps_Str2num(p1+posx,&dx)/Gps_Pow(10,dx);	 	//�õ�UTCʱ��,ȥ��ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;
	}
	posx=Gps_GetCommaPos(p1,3);								//�õ�γ��
	if(posx!=0XFF)
	{
		temp=Gps_Str2num(p1+posx,&dx);
		gpsx->latitude=temp/Gps_Pow(10,dx+2);	//�õ���
		rs=temp%Gps_Pow(10,dx+2);				//�õ�'
		gpsx->latitude=gpsx->latitude*Gps_Pow(10,5)+(rs*Gps_Pow(10,5-dx))/60;//ת��Ϊ��
	}
	posx=Gps_GetCommaPos(p1,4);								//��γ���Ǳ�γ
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);
 	posx=Gps_GetCommaPos(p1,5);								//�õ�����
	if(posx!=0XFF)
	{
		temp=Gps_Str2num(p1+posx,&dx);
		gpsx->longitude=temp/Gps_Pow(10,dx+2);	//�õ���
		rs=temp%Gps_Pow(10,dx+2);				//�õ�'
		gpsx->longitude=gpsx->longitude*Gps_Pow(10,5)+(rs*Gps_Pow(10,5-dx))/60;//ת��Ϊ��
	}
	posx=Gps_GetCommaPos(p1,6);								//������������
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);
	posx=Gps_GetCommaPos(p1,9);								//�õ�UTC����
	if(posx!=0XFF)
	{
		temp=Gps_Str2num(p1+posx,&dx);		 				//�õ�UTC����
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;
	}
}
//

/**********************************************************************
 * �������ƣ�Gps_GPVTG_Analysis
 * ����˵��������GPVTG��Ϣ�����ݷ���������õ���gps���������Ϣ�ṹ����
 * ����˵����gpsx:nmea��Ϣ�ṹ��
 *          buf:���յ���GPS���ݻ������׵�ַ
 * ����ֵ     ����
 * ������Ϣ����
**********************************************************************/
void Gps_GPVTG_Analysis(Gps_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;
	u8 posx;
	p1=(u8*)strstr((const char *)buf,"$GPVTG");
	if(!p1) return ;
	posx=Gps_GetCommaPos(p1,7);								//�õ���������
	if(posx!=0XFF)
	{
		gpsx->speed=Gps_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=Gps_Pow(10,3-dx);	 	 		//ȷ������1000��
	}
}

/**********************************************************************
 * �������ƣ�GPS_Analysis
 * ����˵�����������뻺��������Ϣ����õ���Ч��gps��Ϣ
 *          �������ṹ��ָ���Ӧ�Ľṹ��buffer
 * ����˵����gpsx:  nmea��Ϣ�ṹ��
 *			buf:   ���յ���GPS���ݻ������׵�ַ
 * ����ֵ     ����
 * ������Ϣ����
**********************************************************************/
void GPS_Analysis(Gps_msg *gpsx,u8 *buf)
{
	Gps_GPGSV_Analysis(gpsx,buf);	//GPGSV����
	Gps_GPGGA_Analysis(gpsx,buf);	//GPGGA����
	Gps_GPGSA_Analysis(gpsx,buf);	//GPGSA����
	Gps_GPRMC_Analysis(gpsx,buf);	//GPRMC����
	Gps_GPVTG_Analysis(gpsx,buf);	//GPVTG����
}

/**********************************************************************
 * �������ƣ�Gps_CheckSum
 * ����˵����GPSУ������ݼ���
 * ����˵����buf:���ݻ������׵�ַ
 *          len:���ݳ���
 *          cka,ckb:����У����.
 * ����ֵ     ����
 * ������Ϣ����
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
 * �������ƣ�Ublox_Cfg_Tp
 * ����˵��������UBLOX NEO-6��ʱ���������
 * ����˵����interval:������
 *          length:������
 *          status:��������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.
 * ����ֵ     ����
 * ������Ϣ����
**********************************************************************/
void Ublox_Cfg_Tp(u32 interval,u32 length,signed char status)
{
	_ublox_cfg_tp *cfg_tp=(_ublox_cfg_tp *)USART_TX_BUF;
	cfg_tp->header=0X62B5;		//cfg header
	cfg_tp->id=0X0706;			//cfg tp id
	cfg_tp->dlength=20;			//����������Ϊ20���ֽ�.
	cfg_tp->interval=interval;	//������,us
	cfg_tp->length=length;		//������,us
	cfg_tp->status=status;	   	//ʱ����������
	cfg_tp->timeref=0;			//�ο�UTC ʱ��
	cfg_tp->flags=0;			//flagsΪ0
	cfg_tp->reserved=0;		 	//����λΪ0
	cfg_tp->antdelay=820;    	//������ʱΪ820ns
	cfg_tp->rfdelay=0;    		//RF��ʱΪ0ns
	cfg_tp->userdelay=0;    	//�û���ʱΪ0ns
	Gps_CheckSum((u8*)(&cfg_tp->id),sizeof(_ublox_cfg_tp)-4,&cfg_tp->cka,&cfg_tp->ckb);
	
	Uart_SendBuffer(USART2,  (char*)USART_TX_BUF,  (int)sizeof(_ublox_cfg_tp));
	
}



/**********************************************************************
 * �������ƣ�UBLOX_Cfg_Rate
 * ����˵��������UBLOX NEO-6��ˢ������
 * ����˵����u32 measrate ����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
 *          u8 reftime   �ο�ʱ�䣬0=UTC Time��1=GPS Time��һ������Ϊ1��
 * ����ֵ     ��������Ϣ
 * ������Ϣ��-1 ���òο�ʱ��С��200ms
**********************************************************************/
int UBLOX_Cfg_Rate(u32 measrate, u8 reftime)
{
	_ublox_cfg_rate *cfg_rate=(_ublox_cfg_rate *)USART_TX_BUF;
 	if(measrate<200)
 	{
 		return -1;		//���С��200ms�����ش�����Ϣ-1
 	}
 	cfg_rate->header=0X62B5;	//cfg header
	cfg_rate->id=0X0806;	 	//cfg rate id
	cfg_rate->dlength=6;	 	//����������Ϊ6���ֽ�.
	cfg_rate->measrate=measrate;//������,us
	cfg_rate->navrate=1;		//�������ʣ����ڣ����̶�Ϊ1
	cfg_rate->timeref=reftime; 	//�ο�ʱ��ΪGPSʱ��
	Gps_CheckSum((u8*)(&cfg_rate->id),sizeof(_ublox_cfg_rate)-4,&cfg_rate->cka,&cfg_rate->ckb);

	Uart_SendBuffer(USART2,   (char*)USART_TX_BUF,  (int)sizeof(_ublox_cfg_rate));
	return  0;
}




