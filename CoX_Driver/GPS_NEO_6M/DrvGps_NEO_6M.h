/**********************************************************************
 * �ļ����ƣ�DrvGps_NEO_6M.h
 * �ļ�˵�����ṩ��ublox NEO 6M GPSģ������ݷ�����gps��Ϣ����
 * |  ����ʱ��      | �汾��Ϣ   |     ������          |    ��ϵ��ʽ                                |
 *  2013-11-01     0.0.1      ShawnFeng         FML927@163.com
**********************************************************************/

#ifndef _DrvGps_NEO_6M_h_
#define _DrvGps_NEO_6M_h_	 

#include "stm32f0xx.h"

typedef   unsigned char     u8;
typedef   signed   char     s8;
typedef  unsigned short int u16;
typedef  signed   short int s16;
//typedef  unsigned long  int u32;
//typedef  signed   long  int s32;

typedef  float f32;


//GPS NMEA-0183Э����Ҫ�����ṹ�嶨�� 
//������Ϣ
typedef struct  
{										    
 	u8 num;		//���Ǳ��
	u8 eledeg;	//��������
	u16 azideg;	//���Ƿ�λ��
	u8 sn;		//�����		   
}Gps_slmsg;  
//UTCʱ����Ϣ
typedef struct  
{										    
 	u16 year;	//���
	u8 month;	//�·�
	u8 date;	//����
	u8 hour; 	//Сʱ
	u8 min; 	//����
	u8 sec; 	//����
}Gps_utc_time;   	   
//NMEA 0183 Э����������ݴ�Žṹ��
typedef struct  
{										    
 	u8 svnum;					//�ɼ�������
	Gps_slmsg slmsg[12];		//���12������
	Gps_utc_time utc;			//UTCʱ��
	u32 latitude;				//γ�� ������100000��,ʵ��Ҫ����100000
	u8 nshemi;					//��γ/��γ,N:��γ;S:��γ				  
	u32 longitude;			    //���� ������100000��,ʵ��Ҫ����100000
	u8 ewhemi;					//����/����,E:����;W:����
	u8 gpssta;					//GPS״̬:0,δ��λ;1,�ǲ�ֶ�λ;2,��ֶ�λ;6,���ڹ���.				  
 	u8 posslnum;				//���ڶ�λ��������,0~12.
 	u8 possl[12];				//���ڶ�λ�����Ǳ��
	u8 fixmode;					//��λ����:1,û�ж�λ;2,2D��λ;3,3D��λ
	u16 pdop;					//λ�þ������� 0~500,��Ӧʵ��ֵ0~50.0
	u16 hdop;					//ˮƽ�������� 0~500,��Ӧʵ��ֵ0~50.0
	u16 vdop;					//��ֱ�������� 0~500,��Ӧʵ��ֵ0~50.0 

	int altitude;			 	//���θ߶�,�Ŵ���10��,ʵ�ʳ���10.��λ:0.1m	 
	u16 speed;					//��������,�Ŵ���1000��,ʵ�ʳ���10.��λ:0.001����/Сʱ	 
}Gps_msg; 


//////////////////////////////////////////////////////////////////////////////////////////////////// 	
//UBLOX NEO-6M ʱ���������ýṹ��
typedef struct
{										    
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG TP ID:0X0706 (С��ģʽ)
	u16 dlength;				//���ݳ���
	u32 interval;				//ʱ��������,��λΪus
	u32 length;				 	//������,��λΪus
	signed char status;			//ʱ����������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.			  
	u8 timeref;			   		//�ο�ʱ��:0,UTCʱ��;1,GPSʱ��;2,����ʱ��.
	u8 flags;					//ʱ���������ñ�־
	u8 reserved;				//����			  
 	signed short antdelay;	 	//������ʱ
 	signed short rfdelay;		//RF��ʱ
	signed int userdelay; 	 	//�û���ʱ	
	u8 cka;						//У��CK_A 							 	 
	u8 ckb;						//У��CK_B							 	 
}_ublox_cfg_tp; 


//UBLOX NEO-6M ˢ���������ýṹ��
 typedef struct
{										    
 	u16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	u16 id;						//CFG RATE ID:0X0806 (С��ģʽ)
	u16 dlength;				//���ݳ���
	u16 measrate;				//����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
	u16 navrate;				//�������ʣ����ڣ����̶�Ϊ1
	u16 timeref;				//�ο�ʱ�䣺0=UTC Time��1=GPS Time��
 	u8 cka;						//У��CK_A 							 	 
	u8 ckb;						//У��CK_B							 	 
}_ublox_cfg_rate; 
				 
/**************************   �ⲿ�ӿں���   ****************************/
/**********************************************************************
 * �������ƣ�GPS_Analysis
 * ����˵�����������뻺��������Ϣ����õ���Ч��gps��Ϣ
 *          �������ṹ��ָ���Ӧ�Ľṹ��buffer
 * ����˵����gpsx:  nmea��Ϣ�ṹ��
 *			buf:   ���յ���GPS���ݻ������׵�ַ
 * ����ֵ     ����
 * ������Ϣ����
**********************************************************************/
void GPS_Analysis(Gps_msg *gpsx,u8 *buf);
/**********************************************************************
 * �������ƣ�Ublox_Cfg_Tp
 * ����˵��������UBLOX NEO-6��ʱ���������
 * ����˵����interval:������
 *          length:������
 *          status:��������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.
 * ����ֵ     ����
 * ������Ϣ����
**********************************************************************/
void Ublox_Cfg_Tp(u32 interval,u32 length,signed char status);
/**********************************************************************
 * �������ƣ�UBLOX_Cfg_Rate
 * ����˵��������UBLOX NEO-6��ˢ������
 * ����˵����u32 measrate ����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
 *          u8 reftime   �ο�ʱ�䣬0=UTC Time��1=GPS Time��һ������Ϊ1��
 * ����ֵ     ��������Ϣ
 * ������Ϣ��-1 ���òο�ʱ��С��200ms
**********************************************************************/
int UBLOX_Cfg_Rate(u32 measrate, u8 reftime);




#endif  

 





