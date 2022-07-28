/*
 * main.c
 *
 *  Created on: Apr 1, 2022
 *      Author: ThanhNguyen
 */

#include <stm32f401re_rcc.h>
#include <stm32f401re_gpio.h>
#include <stm32f401re_exti.h>
#include <stm32f401re_syscfg.h>
#include <stm32f401re_spi.h>
#include <stm32f401re_i2c.h>
#include <stm32f401re_tim.h>
#include <stm32f401re_usart.h>
#include <misc.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define I2C1_GPIO_PORT							GPIOB
#define I2C1_GPIO_SCL							GPIO_Pin_8
#define I2C1_GPIO_SDA							GPIO_Pin_9

#define TRANSMITTER								0
#define RECEIVER								1

#define MPU6050_ADDRESS							0x68
#define SMPRT_DIV_REGEGISTER            		0x19
#define CONFIG_REGISTER                 		0x1A
#define GYRO_CONFIG_REGISTER            		0x1B
#define ACCEL_CONFIG_REGISTER           		0x1C
#define INT_ENABLE_REGISTER             		0x38
#define PWR_MGMT_1_REGISTER             		0x6B
#define ACCEL_XOUT_REGISTER             		0x3B
#define ACCEL_YOUT_REGISTER             		0x3D
#define ACCEL_ZOUT_REGISTER             		0x3F
#define GYRO_XOUT_REGISTER              		0x43
#define GYRO_YOUT_REGISTER              		0x45
#define GYRO_ZOUT_REGISTER              		0x47
#define INT_ENABLE_REGISTER             		0x38
#define INT_STATUS_REGISTER             		0x3A

#define SPI2_GPIO_PORT							GPIOB
#define SPI2_SCK_PIN							GPIO_Pin_13
#define SPI2_MOSI_PIN							GPIO_Pin_15
#define SPI2_NSS_PIN							GPIO_Pin_12

uint32_t msTick = 0;

typedef struct
{
	float x;
	float y;
	float z;
} acc_value_t;

typedef enum
{
    huong_len = 0, huong_xuong = 2, huong_trai = 1, huong_phai = 3,
} huong_t;

huong_t huong_di_chuyen = huong_phai, huong_cam_bien = huong_phai;

int8_t toa_do_moi;
int8_t chieu_dai_ran;
int8_t toa_do_ran[64];
uint8_t gia_tri_in[8];

uint8_t game_status = 0;
uint8_t button_status = 0;

void SysTick_Init();
uint32_t SysTick_getTick();
void Delay_ms(uint32_t ms);
void Button_Init();
void I2C1_Init();
void I2Cx_SendData(I2C_TypeDef * I2Cx, uint8_t SlaveAddress, uint8_t RegisterAddress, uint8_t data);
uint8_t I2Cx_ReceiveData(I2C_TypeDef * I2Cx, uint8_t SlaveAddress, uint8_t RegisterAddress);
void MPU6050_Init();
acc_value_t MPU6050_ReadAcc(float accIntensity);
void SPI2_Init();
void SPI_Send_Data(SPI_TypeDef * SPIx, uint16_t data);
void MAX7219_Init();
void cho_vao_game();
void khoi_tao_game();
void in_man_hinh();
huong_t xac_dinh_huong_di_chuyen(huong_t huong_hien_tai);
void xac_dinh_toa_do_ran(huong_t huong_hien_tai);
void xac_dinh_toa_do_moi();
void nhap_nhay_moi();
uint8_t kiem_tra_ran_an_moi(huong_t huong_hien_tai);
void tang_chieu_dai_ran();
uint8_t kiem_tra_trang_thai_ran();
void thong_bao_game_over();
void xu_ly_chuong_trinh();

int main()
{
	SystemCoreClockUpdate();
	SysTick_Init();
	Button_Init();
	I2C1_Init();
	MPU6050_Init();
	SPI2_Init();
	MAX7219_Init();

	while (1)
	{
		xu_ly_chuong_trinh();
	}
}

/********* System Tick ***************************************************************************/
void SysTick_Init()
{
	SysTick_Config(SystemCoreClock / 1000);
}

void SysTick_Handler()
{
	msTick++;
}

uint32_t SysTick_getTick()
{
	return msTick;
}

void Delay_ms(uint32_t ms)
{
	uint32_t to, t1, t = 0;

	to = SysTick_getTick();

	do
	{
		t1 = SysTick_getTick();

		if (t1 >= to)
		{
			t += (t1 - to);
		}
		else
		{
			t += (0xFFFFFFFFu + t1 - to);
		}

		to = t1;
	} while (t < ms);
}

/********* Button Pin C13 ***************************************************************************/

void Button_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

	EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI15_10_IRQHandler()
{
	if (EXTI_GetFlagStatus(EXTI_Line13) == 1)
	{
		if (game_status == 0)
		{
			button_status = 1;
		}
	}

	EXTI_ClearITPendingBit(EXTI_Line13);
}

/********* I2C & MPU6050 ***************************************************************************/
void I2C1_Init()
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	I2C_InitTypeDef		I2C_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_OType = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 400000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}

void I2Cx_SendData(I2C_TypeDef * I2Cx, uint8_t SlaveAddress, uint8_t RegisterAddress, uint8_t data)
{
	// I2C start
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2Cx, ENABLE);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	// I2C address phase
	I2C_Send7bitAddress(I2Cx, SlaveAddress << 1, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2Cx, RegisterAddress);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_SendData(I2Cx, data);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

uint8_t I2Cx_ReceiveData(I2C_TypeDef * I2Cx, uint8_t SlaveAddress, uint8_t RegisterAddress)
{
	uint8_t data;

	// I2C start
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2Cx, ENABLE);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	// I2C address phase
	I2C_Send7bitAddress(I2Cx, SlaveAddress << 1, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	// I2C send register address
	I2C_SendData(I2Cx, RegisterAddress);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	// I2C stop
	I2C_GenerateSTOP(I2Cx, ENABLE);

	// I2C start
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2Cx, ENABLE);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	// I2C address phase
	I2C_Send7bitAddress(I2Cx, SlaveAddress << 1, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	// I2C send receive data
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
	data = I2C_ReceiveData(I2Cx);
	// I2C stop
	I2C_GenerateSTOP(I2Cx, ENABLE);

	return data;
}

void MPU6050_Init()
{
	I2C1_Init();
	// sample rate 100 Hz
	I2Cx_SendData(I2C1, MPU6050_ADDRESS, SMPRT_DIV_REGEGISTER, 0x09);
	// DLPF
	I2Cx_SendData(I2C1, MPU6050_ADDRESS, CONFIG_REGISTER, 0x03);
	// gyro full scale 500 deg/s
	I2Cx_SendData(I2C1, MPU6050_ADDRESS, GYRO_CONFIG_REGISTER, 0x08);
	// acc full scale 8g
	I2Cx_SendData(I2C1, MPU6050_ADDRESS, ACCEL_CONFIG_REGISTER, 0x10);
	// power management
	I2Cx_SendData(I2C1, MPU6050_ADDRESS, PWR_MGMT_1_REGISTER, 0x00);
}

acc_value_t MPU6050_ReadAcc(float accIntensity)
{
	uint8_t highReg, lowReg;
	int16_t data;
	acc_value_t acc;

	highReg = I2Cx_ReceiveData(I2C1, MPU6050_ADDRESS, ACCEL_XOUT_REGISTER);
	lowReg = I2Cx_ReceiveData(I2C1, MPU6050_ADDRESS, ACCEL_XOUT_REGISTER + 1);
	data = (highReg << 8) | lowReg;
	acc.x = (float)data / accIntensity;

	highReg = I2Cx_ReceiveData(I2C1, MPU6050_ADDRESS, ACCEL_YOUT_REGISTER);
	lowReg = I2Cx_ReceiveData(I2C1, MPU6050_ADDRESS, ACCEL_YOUT_REGISTER + 1);
	data = (highReg << 8) | lowReg;
	acc.y = (float)data / accIntensity;

	highReg = I2Cx_ReceiveData(I2C1, MPU6050_ADDRESS, ACCEL_ZOUT_REGISTER);
	lowReg = I2Cx_ReceiveData(I2C1, MPU6050_ADDRESS, ACCEL_ZOUT_REGISTER + 1);
	data = (highReg << 8) | lowReg;
	acc.z = (float)data / accIntensity;

	return acc;
}

/********* SPI & MAX7219 ***************************************************************************/

void SPI2_Init()
{
	GPIO_InitTypeDef GPIO_Init_Structure;
	SPI_InitTypeDef SPI_Init_Structure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_Init_Structure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init_Structure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_Structure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init_Structure.GPIO_Pin = SPI2_SCK_PIN;
	GPIO_PinAFConfig(SPI2_GPIO_PORT, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_Init(SPI2_GPIO_PORT, &GPIO_Init_Structure);

	GPIO_Init_Structure.GPIO_Pin = SPI2_MOSI_PIN;
	GPIO_PinAFConfig(SPI2_GPIO_PORT, GPIO_PinSource15, GPIO_AF_SPI2);
	GPIO_Init(SPI2_GPIO_PORT, &GPIO_Init_Structure);

	GPIO_Init_Structure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init_Structure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_Structure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init_Structure.GPIO_Pin = SPI2_NSS_PIN;
	GPIO_Init(SPI2_GPIO_PORT, &GPIO_Init_Structure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	SPI_Init_Structure.SPI_Mode = SPI_Mode_Master;
	SPI_Init_Structure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_Init_Structure.SPI_CPOL = SPI_CPOL_Low;
	SPI_Init_Structure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_Init_Structure.SPI_DataSize = SPI_DataSize_16b;
	SPI_Init_Structure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init_Structure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_Init_Structure.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI2,  &SPI_Init_Structure);
	SPI_Cmd(SPI2, ENABLE);
}

void SPI_Send_Data(SPI_TypeDef * SPIx, uint16_t data)
{
	GPIO_ResetBits(SPI2_GPIO_PORT, SPI2_NSS_PIN);
	SPI_I2S_SendData(SPI2, data);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == 1);
	GPIO_SetBits(SPI2_GPIO_PORT, SPI2_NSS_PIN);
}

void MAX7219_Init()
{
	// Decode mode
	SPI_Send_Data(SPI2, (0x09 << 8) | 0x00);
	// Intensity
	SPI_Send_Data(SPI2, (0x0A << 8) | 0x09);
	// Scan limit
	SPI_Send_Data(SPI2, (0x0B << 8) | 0x07);
	// Shutdown mode
	SPI_Send_Data(SPI2, (0x0C << 8) | 0x01);
	// Display test
	SPI_Send_Data(SPI2, (0x0F << 8) | 0x00);
}

/********* Cac ham xu ly chuong trinh ***************************************************************************/

void cho_vao_game()
{
	SPI_Send_Data(SPI2, (0x01 << 8) | 0x07);
	// 3
	SPI_Send_Data(SPI2, (0x03 << 8) | 0x70);
	SPI_Send_Data(SPI2, (0x04 << 8) | 0x40);
	SPI_Send_Data(SPI2, (0x05 << 8) | 0x72);
	SPI_Send_Data(SPI2, (0x06 << 8) | 0x40);
	SPI_Send_Data(SPI2, (0x07 << 8) | 0x70);
	Delay_ms(1000);
	// 2
	SPI_Send_Data(SPI2, (0x03 << 8) | 0x70);
	SPI_Send_Data(SPI2, (0x04 << 8) | 0x10);
	SPI_Send_Data(SPI2, (0x05 << 8) | 0x72);
	SPI_Send_Data(SPI2, (0x06 << 8) | 0x40);
	SPI_Send_Data(SPI2, (0x07 << 8) | 0x70);
	Delay_ms(1000);
	// 1
	SPI_Send_Data(SPI2, (0x03 << 8) | 0x40);
	SPI_Send_Data(SPI2, (0x04 << 8) | 0x40);
	SPI_Send_Data(SPI2, (0x05 << 8) | 0x42);
	SPI_Send_Data(SPI2, (0x06 << 8) | 0x60);
	SPI_Send_Data(SPI2, (0x07 << 8) | 0x40);
	Delay_ms(1000);
}

void khoi_tao_game()
{
	int8_t hang, cot;

	for (int8_t i = 0; i < 8; i++)
	{
		gia_tri_in[i] = 0;
	}

	toa_do_moi = 33;
	hang = toa_do_moi / 8;
	cot = toa_do_moi % 8;
	gia_tri_in[hang] |= (1 << cot);

	chieu_dai_ran = 3;
	int8_t temp = 3;
	for (int8_t i = 0; i < 3; i++)
	{
		toa_do_ran[i] = --temp;
		hang = toa_do_ran[i] / 8;
		cot = toa_do_ran[i] % 8;
		gia_tri_in[hang] |= (1 << cot);
	}
}

void in_man_hinh()
{
	for (int8_t i = 0; i < 8; i++)
	{
		SPI_Send_Data(SPI2, ((i + 1) << 8) | gia_tri_in[i]);
	}
}

huong_t xac_dinh_huong_di_chuyen(huong_t huong_hien_tai)
{
	acc_value_t acc_value;
	float roll, pitch;
	huong_t huong_cam_bien = huong_hien_tai;

	acc_value = MPU6050_ReadAcc(4196);

	roll = atan2(acc_value.y, sqrt(pow(acc_value.x, 2) + pow(acc_value.z, 2))) * 180 / M_PI;
	pitch = atan2(acc_value.x, sqrt(pow(acc_value.y, 2) + pow(acc_value.z, 2))) * 180 / M_PI;

	if ((abs(roll) - abs(pitch)) >= 0)
	{
	    if (roll > 20)
		{
	    	huong_cam_bien = huong_trai;
		}
	    else if (roll < -20)
		{
	    	huong_cam_bien = huong_phai;
		}
	}
	else
	{
	    if (pitch > 20)
		{
	    	huong_cam_bien = huong_len;
	    }
	    else if (pitch < -20)
		{
	    	huong_cam_bien = huong_xuong;
		}
	}

    if (((huong_hien_tai % 2)-(huong_cam_bien % 2)) != 0)
    {
    	return huong_cam_bien;
    }

    return huong_hien_tai;
}

void xac_dinh_toa_do_ran(huong_t huong_hien_tai)
{
	int8_t i, hang, cot;

    // Xóa giá trị tọa độ rắn cũ
    for (i = 0; i < chieu_dai_ran; i++)
    {
        hang = toa_do_ran[i] / 8;
        cot = toa_do_ran[i] % 8;
        gia_tri_in[hang] &= ~(1 << cot);
    }

    // Xác định lại thân rắn mới
    for (i = chieu_dai_ran - 1; i >= 1; i--)
    {
        toa_do_ran[i] = toa_do_ran[i-1];
        hang = toa_do_ran[i] / 8;
        cot = toa_do_ran[i] % 8;
        gia_tri_in[hang] |= (1 << cot);
    }

    // Xác định tọa độ đầu rắn mới theo hướng đã xác định
    switch (huong_hien_tai)
    {
    	case huong_len:
    		if (toa_do_ran[i] / 8 == 7)
    		{
    			toa_do_ran[i] %= 8;
    			hang = toa_do_ran[i] / 8;
    			cot = toa_do_ran[i] % 8;
    			gia_tri_in[hang] |= (1 << cot);
    		}
    		else
    		{
    			toa_do_ran[i] += 8;
    			hang = toa_do_ran[i] / 8;
    			cot = toa_do_ran[i] % 8;
    			gia_tri_in[hang] |= (1 << cot);
    		}
    		break;
    	case huong_xuong:
    		if (toa_do_ran[i] / 8 == 0)
    		{
    			toa_do_ran[i] += 56;
    			hang = toa_do_ran[i] / 8;
    			cot = toa_do_ran[i] % 8;
    			gia_tri_in[hang] |= (1 << cot);
    		}
    		else
    		{
    			toa_do_ran[i] -= 8;
    			hang = toa_do_ran[i] / 8;
    			cot = toa_do_ran[i] % 8;
    			gia_tri_in[hang] |= (1 << cot);
    		}
    		break;
    	case huong_trai:
    		if ((toa_do_ran[i] % 8) == 0)
    		{
    			toa_do_ran[i] += 7;
    			hang = toa_do_ran[i] / 8;
    			cot = toa_do_ran[i] % 8;
    			gia_tri_in[hang] |= (1 << cot);
    		}
    		else
    		{
    			toa_do_ran[i] -= 1;
    			hang = toa_do_ran[i] / 8;
    			cot = toa_do_ran[i] % 8;
    			gia_tri_in[hang] |= (1 << cot);
    		}
    		break;
    	case huong_phai:
    		if (toa_do_ran[i] % 8 == 7)
    		{
    			toa_do_ran[i] -= 7;
    			hang = toa_do_ran[i] / 8;
    			cot = toa_do_ran[i] % 8;
    			gia_tri_in[hang] |= (1 << cot);
    		}
    		else
    		{
    			toa_do_ran[i] += 1;
    			hang = toa_do_ran[i] / 8;
    			cot = toa_do_ran[i] % 8;
    			gia_tri_in[hang] |= (1 << cot);
    			i++;
    		}
    		break;
    	default:
    		break;
    }
}

void xac_dinh_toa_do_moi()
{
    int8_t temp, hang, cot;

    // xóa tọa độ mồi cũ
    hang = toa_do_moi / 8;
    cot = toa_do_moi % 8;
    gia_tri_in[hang] &= ~(1 << cot);

    // tìm tọa độ mồi mới
    do
    {
        temp = rand() % 64;
        for (uint8_t i = 0; i < chieu_dai_ran; i++)
        {
            if (temp == toa_do_ran[i])
            {
                temp = toa_do_moi;
                break;
            }
        }
    } while (temp == toa_do_moi);

    toa_do_moi = temp;
    hang = toa_do_moi / 8;
    cot = toa_do_moi % 8;
    gia_tri_in[hang] |= (1 << cot);
}

void nhap_nhay_moi()
{
    int8_t hang, cot;
    static uint8_t status = 0;

    if (status == 0)
    {
    	hang = toa_do_moi / 8;
    	cot = toa_do_moi % 8;
    	gia_tri_in[hang] &= ~(1 << cot);
    	status = 1;
    }
    else
    {
    	hang = toa_do_moi / 8;
    	cot = toa_do_moi % 8;
    	gia_tri_in[hang] |= (1 << cot);
    	status = 0;
    }

    SPI_Send_Data(SPI2, ((hang + 1) << 8) | gia_tri_in[hang]);
}

uint8_t kiem_tra_ran_an_moi(huong_t huong_hien_tai)
{
    if (abs(toa_do_moi - toa_do_ran[0]) == 8)
    {
        if ((huong_hien_tai == huong_len) || (huong_hien_tai == huong_xuong))
        {
            return 1;
        }
    }
    else if (abs(toa_do_moi - toa_do_ran[0]) == 1)
    {
        if ((huong_hien_tai == huong_trai) || (huong_hien_tai == huong_phai))
        {
            return 1;
        }
    }

    return 0;
}

void tang_chieu_dai_ran()
{
    uint8_t i, hang, cot;

    // xóa giá trị tọa độ rắn cũ
    for (uint8_t i = 0; i < chieu_dai_ran; i++)
    {
        hang = toa_do_ran[i] / 8;
        cot = toa_do_ran[i] % 8;
        gia_tri_in[hang] &= ~(1 << cot);
    }

    chieu_dai_ran++;

    for (i = chieu_dai_ran - 1; i >= 1; i--)
    {
        toa_do_ran[i] = toa_do_ran[i-1];
        hang = toa_do_ran[i] / 8;
        cot = toa_do_ran[i] % 8;
        gia_tri_in[hang] |= (1 << cot);
    }
    toa_do_ran[i] = toa_do_moi;
    hang = toa_do_ran[i] / 8;
    cot = toa_do_ran[i] % 8;
    gia_tri_in[hang] |= (1 << cot);
}

uint8_t kiem_tra_trang_thai_ran()
{
    for (uint8_t i = 1; i < chieu_dai_ran; i++)
    {
        if (toa_do_ran[0] == toa_do_ran[i])
        {
            return 0;
        }
    }

    return 1;
}

void thong_bao_game_over()
{
	while (button_status == 0)
	{
		for (int8_t i = 0; i < 8; i++)
		{
			SPI_Send_Data(SPI2, ((i + 1) << 8) | gia_tri_in[i]);
		}

		Delay_ms(500);

		for (int8_t i = 0; i < 8; i++)
		{
			SPI_Send_Data(SPI2, ((i + 1) << 8) | 0);
		}

		Delay_ms(500);
	}
}

void xu_ly_chuong_trinh()
{
	static uint32_t tim_init1, tim_current1, tim_total1 = 0, tim_init2, tim_current2, tim_total2 = 0,  tim_init3, tim_current3, tim_total3 = 0;

	if (game_status == 0)
	{
		cho_vao_game();
		tim_init1 = SysTick_getTick();
		tim_init2 = tim_init1;
		tim_init3 = tim_init1;
		game_status = 1;
		khoi_tao_game();
		in_man_hinh();
	}

	tim_current1 = SysTick_getTick();
	if (tim_current1 >= tim_init1)
	{
		tim_total1 += (tim_current1 - tim_init1);
	}
	else
	{
		tim_total1 += (0xFFFFFFFFu + tim_current1 - tim_init1);
	}
	tim_init1 = tim_current1;

	if (tim_total1 >= 1000)
	{
		huong_di_chuyen = xac_dinh_huong_di_chuyen(huong_di_chuyen);
		if (kiem_tra_ran_an_moi(huong_di_chuyen))
		{
			tang_chieu_dai_ran();
			xac_dinh_toa_do_moi();
			xac_dinh_toa_do_ran(huong_di_chuyen);
			in_man_hinh();
			tim_total3 = 0;
		}
		else
		{
			xac_dinh_toa_do_ran(huong_di_chuyen);
			if (!kiem_tra_trang_thai_ran())
			{
				tim_total1 = 0;
				game_status = 0;
				button_status = 0;
				huong_di_chuyen = huong_phai;
				huong_cam_bien = huong_phai;
				thong_bao_game_over();
			}
			in_man_hinh();
		}

		tim_total1 = 0;
	}

	tim_current2 = SysTick_getTick();

	if (tim_current2 >= tim_init2)
	{
		tim_total2 += (tim_current2 - tim_init2);
	}
	else
	{
		tim_total2 += (0xFFFFFFFFu + tim_current2 - tim_init2);
	}
	tim_init2 = tim_current2;

	if (tim_total2 >= 200)
	{
		nhap_nhay_moi();
		tim_total2 = 0;
	}

	tim_current3 = SysTick_getTick();

	if (tim_current3 >= tim_init3)
	{
		tim_total3 += (tim_current3 - tim_init3);
	}
	else
	{
		tim_total3 += (0xFFFFFFFFu + tim_current3 - tim_init3);
	}
	tim_init3 = tim_current3;

	if (tim_total3 >= 5000)
	{
		xac_dinh_toa_do_moi();
		in_man_hinh();
		tim_total3 = 0;
	}
}


