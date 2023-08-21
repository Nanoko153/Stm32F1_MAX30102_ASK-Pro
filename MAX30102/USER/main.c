/*************************************************************************************
STM32F103-30102:
	VCC<->3.3V
	GND<->GND
	SCL<->PB7
	SDA<->PB8
	IM<->PB9
0.96inch OLED :
	VCC<->3.3V
	GND<->GND
	SCL<->PA5
	SDA<->PA6
	RST<->PA3
	DC<->PA4
	CS<->PA2
USB-TTL:
	5V<->5V
	GND<->GND
	RXD<->PA9
	TXD<->PA10
LED:
	PC13 1:On 0:off
**************************************************************************************/
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "max30102.h" 
#include "myiic.h"
#include "algorithm.h"
#include "oled.h"
#include "speaker.h"

uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

uint32_t un_min, un_max, un_prev_data;  
int i;
int32_t n_brightness;
float f_temp;
u8 temp_num=0;
u8 temp[6];
u8 str[100];
u8 dis_hr=0,dis_spo2=0;	

	
char h[4] = {0,0,0,'\0'};
char s[4] = {0,0,0,'\0'};
#define MAX_BRIGHTNESS 255

void dis_DrawCurve(u32* data,u8 x);
void SpeakerInput(void);
void MAX30102_Working(void);
void My_USART3_Init(void);
void usart3_send_str(char *pstr);
void ble_config_set(void);

int main(void)
{ 
	
	 GPIO_InitTypeDef  GPIO_InitStructure;
		
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
		
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //LED0-->PB.5 �˿�����
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
	 GPIO_SetBits(GPIOA,GPIO_Pin_8);						 //PB.5 �����

	NVIC_Configuration();
	delay_init();	    	 //��ʱ������ʼ��	  
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
	LED_Init();
	SpeakerInput_Init();
	My_USART3_Init();
	ble_config_set();
	//OLED
	OLED_Init();
	OLED_ShowString(0,0,"  Loading  ",16);
	OLED_Refresh_Gram();//������ʾ��OLED	 

	max30102_init();

	printf("\r\n MAX30102  init  \r\n");

	un_min=0x3FFFF;
	un_max=0;
	
	n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
	//read the first 500 samples, and determine the signal range
    for(i=0;i<n_ir_buffer_length;i++)
    {
        while(MAX30102_INT==1);   //wait until the interrupt pin asserts
        
		max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp);
		aun_red_buffer[i] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
		aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];   // Combine values to get the actual number
            
        if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];    //update signal min
        if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];    //update signal max
    }
	un_prev_data=aun_red_buffer[i];
	//calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
	
	LED0 = 0;			//LED��˸ϵͳ�����ɹ�
	delay_ms(100);
	LED0 = 1;
	delay_ms(100);
	LED0 = 0;
	delay_ms(100);
	LED0 = 1;
	delay_ms(100);
	LED0 = 0;
	delay_ms(100);
	LED0 = 1;

	
	
	while(1)
	{
		if(PBin(0)==0)
		{
			LED0 = 1;
			PAout(8) = 0;
			OLED_Clear();
			while(PBin(0)==0)
			{
				//*****ִ�����ʹر�ʱ������*****//
				OLED_ShowString(0,0,"  Sleeping  ",16);
			}
		}
		else
		{
			LED0 = 0;
			PAout(8) = 1;
			max30102_init();
			
			while(PBin(0)==1)
			{
				//*****ִ����������ʱ������*****//
				MAX30102_Working();
			}
		}
	}
}

void dis_DrawCurve(u32* data,u8 x)
{
	u16 i;
	u32 max=0,min=262144;
	u32 temp;
	u32 compress;
	
	for(i=0;i<128*2;i++)
	{
		if(data[i]>max)
		{
			max = data[i];
		}
		if(data[i]<min)
		{
			min = data[i];
		}
	}
	
	compress = (max-min)/20;
	
	for(i=0;i<128;i++)
	{
		temp = data[i*2] + data[i*2+1];
		temp/=2;
		temp -= min;
		temp/=compress;
		if(temp>20)temp=20;
		OLED_DrawPoint(i,63-x-temp+21,1);
	}
}

void SpeakerInput(void)
{
	if(PBin(0)==0)
	{
		LED0 = 1;
	}
	else
	{
		LED0 = 0;
	}
		
}

void MAX30102_Working(void)
{
		i=0;
        un_min=0x3FFFF;
        un_max=0;
		
		SpeakerInput();
		
		//dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for(i=100;i<500;i++)
        {
            aun_red_buffer[i-100]=aun_red_buffer[i];
            aun_ir_buffer[i-100]=aun_ir_buffer[i];
            
            //update the signal min and max
            if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];
            if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];
        }
		//take 100 sets of samples before calculating the heart rate.
        for(i=400;i<500;i++)
        {
            un_prev_data=aun_red_buffer[i-1];
            while(MAX30102_INT==1);
            max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp);
			aun_red_buffer[i] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
			aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];   // Combine values to get the actual number
        
            if(aun_red_buffer[i]>un_prev_data)
            {
                f_temp=aun_red_buffer[i]-un_prev_data;
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness-=(int)f_temp;
                if(n_brightness<0)
                    n_brightness=0;
            }
            else
            {
                f_temp=un_prev_data-aun_red_buffer[i];
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness+=(int)f_temp;
                if(n_brightness>MAX_BRIGHTNESS)
                    n_brightness=MAX_BRIGHTNESS;
            }
			//send samples and calculation result to terminal program through UART
			if(ch_hr_valid == 1 && n_heart_rate<120)//**/ ch_hr_valid == 1 && ch_spo2_valid ==1 && n_heart_rate<120 && n_sp02<101
			{
				dis_hr = n_heart_rate;
				dis_spo2 = n_sp02;
			}
			else
			{
				dis_hr = 0;
				dis_spo2 = 0;
			}
				printf("HR=%i, ", n_heart_rate);   
				printf("HRvalid=%i, ", ch_hr_valid);
				printf("SpO2=%i, ", n_sp02);
				printf("SPO2Valid=%i\r\n", ch_spo2_valid);
		}
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
		
		//��ʾˢ��
		if((dis_hr == 0 && dis_spo2 == 0) || dis_hr<60 || dis_hr > 200)  //**dis_hr == 0 && dis_spo2 == 0
		{
			sprintf((char *)str,"HR:--- SpO2:--- ");//**HR:--- SpO2:--- 
		}
		else
		{
			//*****ִ�л����Ч����ʱ������*****//
			sprintf((char *)str,"HR:%3d SpO2:%3d ",dis_hr,dis_spo2);//**HR:%3d SpO2:%3d 
			//���ڷ���
			usart3_send_str(str);
			
		}
		OLED_ShowString(0,0,str,16);
		OLED_Fill(0,23,127,63,0);
		//������ϣ���������
		dis_DrawCurve(aun_red_buffer,20);
		dis_DrawCurve(aun_ir_buffer,0);
		OLED_Refresh_Gram();//������ʾ��OLED	 
}

void My_USART3_Init(void)  
{  
    GPIO_InitTypeDef GPIO_InitStrue;  
    USART_InitTypeDef USART_InitStrue;  
    NVIC_InitTypeDef NVIC_InitStrue;  
      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//GPIO�˿�ʹ��  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//���ڶ˿�ʹ��  
      
    GPIO_InitStrue.GPIO_Mode=GPIO_Mode_AF_PP;  
    GPIO_InitStrue.GPIO_Pin=GPIO_Pin_10;  
    GPIO_InitStrue.GPIO_Speed=GPIO_Speed_10MHz;  
    GPIO_Init(GPIOB,&GPIO_InitStrue);  
      
    GPIO_InitStrue.GPIO_Mode=GPIO_Mode_IN_FLOATING;  
    GPIO_InitStrue.GPIO_Pin=GPIO_Pin_11;  
    GPIO_InitStrue.GPIO_Speed=GPIO_Speed_10MHz;  
    GPIO_Init(GPIOB,&GPIO_InitStrue);  
      
    USART_InitStrue.USART_BaudRate=9600;  
    USART_InitStrue.USART_HardwareFlowControl=USART_HardwareFlowControl_None;  
    USART_InitStrue.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;  
    USART_InitStrue.USART_Parity=USART_Parity_No;  
    USART_InitStrue.USART_StopBits=USART_StopBits_1;  
    USART_InitStrue.USART_WordLength=USART_WordLength_8b;  
      
    USART_Init(USART3,&USART_InitStrue);
      
    USART_Cmd(USART3,ENABLE);					//ʹ�ܴ���2  
      
    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//���������ж�  
      
    NVIC_InitStrue.NVIC_IRQChannel=USART3_IRQn;  
    NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE;  
    NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=0;  
    NVIC_InitStrue.NVIC_IRQChannelSubPriority=1;  
    NVIC_Init(&NVIC_InitStrue);  
      
}  

void usart3_send_str(char *pstr)
{
	char *p=pstr;
	
	while(*p!='\0')
	{
		//�����յ������ݷ�����PC
		USART_SendData(USART3,*p);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);
		USART_ClearFlag(USART3,USART_FLAG_TXE);
		
		//ָ��ƫ��
		p++;
	}
}


//ATָ������ģ�飬����4.0ģ�鲻�ܸ��ֻ���������
void ble_config_set(void)
{
	//����ATָ��
	usart3_send_str("AT\r\n");
	delay_ms(500);
	
	//���͸���ģ������ָ���Ҫ��λ��Ч���������AT+RESET������������û�б������ô���¶�����ģ���ϵ磩
	usart3_send_str("AT+NAMENano\r\n");
	delay_ms(500);
	
	//���͸�λģ���ָ��
	usart3_send_str("AT+RESET\r\n");
	delay_ms(2000);
}

void USART3_IRQHandler(void)
{
	uint8_t d;
	
	//����־λ
	if(USART_GetITStatus(USART3,USART_IT_RXNE)==SET)
	{
		/*
		//��������
		d=USART_ReceiveData(USART3);
		
		//�����յ������ݷ�����PC
		USART_SendData(USART3,d);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);
		USART_ClearFlag(USART3,USART_FLAG_TXE);
		
		//��ձ�־λ
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		*/
				//��������
		d=USART_ReceiveData(USART3);
		
		//�����յ������ݷ�����PC
		USART_SendData(USART3,d);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);	
		
		/*
		if(d=='A')PAout(6)^=1;
		if(d=='B')PAout(7)^=1;
		if(d=='C')PAout(4)^=1;
		if(d=='D')PAout(5)^=1;
		*/
		
		//��ձ�־λ
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	}
}