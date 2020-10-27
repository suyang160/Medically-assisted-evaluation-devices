#ifndef __MYIIC_H
#define __MYIIC_H
#include "stdint.h"
#include <ti/drivers/PIN.h>

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ģ��IIC��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

//IO��������
//#define SDA_IN()  {GPIOB->MODER&=~(3<<(4*2));GPIOB->MODER|=(0<<(4*2));}	//PB4����ģʽ
//#define SDA_OUT() {GPIOB->MODER&=~(3<<(4*2));GPIOB->MODER|=(1<<(4*2));} //PB4���ģʽ
//IO��������	 
//#define IIC_SCL    PBout(5) //SCL
//#define IIC_SDA    PBout(4) //SDA	 
//#define READ_SDA   PBin(4)  //����SDA 

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;


//IIC���в�������
uint_t READ_SDA();
void  SDA_IN();
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź� 
uint8_t IIC_ReadByte(uint8_t devaddr,uint8_t addr);	/*��һ�ֽ�*/
void IIC_WriteByte(uint8_t devaddr,uint8_t addr,uint8_t data);	/*дһ�ֽ�*/
void IIC_Read(uint8_t devaddr,uint8_t addr,uint8_t len,uint8_t *rbuf);	/*������ȡ����ֽ�*/
void IIC_Write(uint8_t devaddr,uint8_t addr,uint8_t len,uint8_t *wbuf);/*����д�����ֽ�*/
void SDA_IN();
void SDA_OUT();
void delay_us(uint32_t temp);

#endif
















