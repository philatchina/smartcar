
#include "math.h"
#include "chlib_k.h"
#include "varieble.h"
#include "img_processing.h"
#include "oled.h"
#include "init.h"
#include "ftm.h"
#include "adc.h"
#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "gui.h"

int			page		= 1;    /* ��ʼΪ��һҳ */
int			menuRow		= 1;    /* ��¼��ǰ�ǵڼ��� ֵ����Ϊ1��2��3��4��5  �޸��������ĳ��� */
int			keyState	= 0;    /* ���ڼ�¼����ص�ֵ */
const unsigned char	row_pos[5]	= { 0, 13, 26, 39, 52 };
int			key_counter	= 0;
int			key_temp	= 0;
int			temp1;
int			temp2;
int			temp3;
int			temp4;
int			temp5;
int			temp6;
int			temp7;
int			clearCount = 0;


/*
 *  GPIO_QuickInit(HW_GPIOE, 0, kGPIO_Mode_IPU);   //��
 *  GPIO_QuickInit(HW_GPIOE, 1, kGPIO_Mode_IPU);   //��
 *  GPIO_QuickInit(HW_GPIOE, 3, kGPIO_Mode_IPU);   //��
 *  GPIO_QuickInit(HW_GPIOE, 2, kGPIO_Mode_IPU);   //��
 *  GPIO_QuickInit(HW_GPIOC, 18, kGPIO_Mode_IPU);  //��
 */

int keyCheck( void )                                    /*������� */
{
	int	key_lable = 0;
	int	temp1, temp2;

	/* �ó��ĸ������µı�ǩλ */
	if ( 0 == GPIO_ReadBit( HW_GPIOE, 1 ) )         /* up */
	{
		key_lable = 1;
	}
	if ( 0 == GPIO_ReadBit( HW_GPIOC, 18 ) )        /* down */
	{
		key_lable = 2;
	}
	if ( 0 == GPIO_ReadBit( HW_GPIOE, 3 ) )         /* left */
	{
		key_lable = 3;
	}
	if ( 0 == GPIO_ReadBit( HW_GPIOE, 0 ) )         /* right */
	{
		key_lable = 4;
	}
	if ( 0 == GPIO_ReadBit( HW_GPIOE, 2 ) )         /* select */
	{
		key_lable = 5;
	}


	/* ��ǩλΪ0��û�м����� */
	if ( key_lable == 0 )                           /* û�а������� */
	{
		if ( key_temp != 0 )                    /* ֮ǰ�а������� */
		{
			temp1		= key_counter;  /*���µļ����� */
			temp2		= key_temp;     /* ֮ǰ�İ����ı�� */
			key_counter	= 0;
			key_temp	= 0;
			if ( temp1 > 1 )                /* ���� */
			{
				return(temp2);
			}else {
				return(0);
			}
		}else return(0);
	}
	/* ��ǩλ��Ϊ0���м����� */
	else{                           /*�а������� */
		if ( key_temp == 0 )    /* ֮ǰû�м�⵽�������� */
		{
			key_temp = key_lable;
		}
		key_counter++;

		return(0);
	}
}


void menu()
{
	if ( clearCount == 20 )
	{
		clearCount = 0;
		if ( page != 1 )
		{
			OLED_Clear();
		}
	}else
		++clearCount;
	if ( page == 1 )
	{       /* ��һҳ��ʾͼ�� */
		dispimage();
	}else if ( page == 2 )
	{
		MenuShow2();
	}else if ( page == 3 )
	{
		MenuShow3();
	}
	FlashValueOperate();                                    /* ���ݰ�����Ϣ���޸ı���ֵ */
	keyState = keyCheck();
	SignMove();                                             /* �б��ƶ�+ҳ����ʾ */
	OLED_ShowNum_12061( 0, row_pos[menuRow - 1], page, 1 ); /* ��ǰҳ���͵�ǰ����ʾ */
	OLED_Refresh_Gram();
}


void SignMove()
{
	if ( page == 1 )
	{
		OLED_ShowString_1206( 13, row_pos[0], "image", 1 );
	}
	if ( 0 )
	{
	}else {
		if ( keyState == KEY_UP )
		{
			if ( menuRow == 1 )
			{
				if ( page == 2 )        /* �ڶ�ҳ��һ�а��ϼ� */
				{
					page = 1;
					OLED_Clear();
				}else if ( page == 1 )  /* ��һҳ�������һҳ */
				{
					page = pageMax;
					OLED_Clear();
					menuRow = 5;
				}else {
					page = page - 1;
					OLED_Clear();
					menuRow = 5;
				}
			}else {
				--menuRow;

				OLED_Fill( 0, row_pos[menuRow], 13, row_pos[menuRow] + 13, 0 ); /* ���֮ǰд������ */
			}
		}else if ( keyState == KEY_DOWN )
		{
			if ( page == 1 )                                                        /* ��һҳ���¼� */
			{
				page	= 2;
				menuRow = 1;
				OLED_Clear();
				OLED_ShowNum_12061( 0, row_pos[menuRow - 1], page, 1 );
			}else if ( menuRow == 5 )                                               /* ���һ�а��¼� */
			{
				if ( page == pageMax )
				{
					page	= 1;
					menuRow = 1;
					OLED_Clear();
				}else {
					page	= page + 1;
					menuRow = 1;
					OLED_Clear();
				}
			}else {                                                                         /* ��ҳ */
				OLED_Clear();
				OLED_Fill( 0, row_pos[menuRow - 1], 13, row_pos[menuRow - 1] + 13, 0 ); /* ���֮ǰд������ */
				menuRow = menuRow + 1;
			}
		}             else if ( keyState == KEY_MID )
		{
		}else {
		}
	}
}


void FlashValueOperate()
{
	/* �޸�ÿ��������ֵ */

	if ( keyState != 0 )
		OLED_Clear();
	if ( page == 2 )                /* �˵���һҳ */
	{
		switch ( menuRow )      /* �ж����� */
		{                       /* ���е����� */
		case 0: {
		} break;
		/* ��case1��ʼ  ��һ�� */
		case 1: {
			if ( keyState == KEY_LEFT )
			{
				flashData[0]--;
			}else if ( keyState == KEY_RIGHT )
			{
				flashData[0]++;
			}
		} break;
		/*  �ڶ��� */
		case 2: {
			if ( keyState == KEY_LEFT )
			{
				flashData[1]--;
			}else if ( keyState == KEY_RIGHT )
			{
				flashData[1]++;
			}
		} break;
		case 3: {
			if ( keyState == KEY_LEFT )
			{
			}else if ( keyState == KEY_RIGHT )
			{
			}
		} break;
		case 4: {
			if ( keyState == KEY_LEFT )
			{
			}else if ( keyState == KEY_RIGHT )
			{
			}
		} break;
		}
	}
}


void MenuShow2()                                                                                                        /* ��ʾ���˸����������������Ǹ��������ͣ�������ʵ��ʱ����Ҫ���д��� */
{
	OLED_ShowString_1206( 16, row_pos[0], "string1", 1 );
	if ( number < 0 )
	{
		OLED_ShowString_1206( 65, row_pos[0], "-", 1 ); OLED_ShowNum_12061( 72, row_pos[0], flashData[0], 1 );
	}else { OLED_ShowString_1206( 65, row_pos[0], "+", 1 );  OLED_ShowNum_12061( 72, row_pos[0], flashData[0], 1 ); }     /* OLED_ShowString_1206(72,row_pos[0],"value1",1); */

	OLED_ShowString_1206( 16, row_pos[1], "string2", 1 );
	if ( number < 0 )
	{
		OLED_ShowString_1206( 65, row_pos[1], "-", 1 ); OLED_ShowNum_12061( 72, row_pos[1], flashData[1], 1 );
	}else { OLED_ShowString_1206( 65, row_pos[1], "+", 1 );  OLED_ShowNum_12061( 72, row_pos[1], flashData[1], 1 ); }     /* OLED_ShowString_1206(72,row_pos[0],"value1",1); */

	OLED_ShowString_1206( 16, row_pos[2], "string3", 1 );
	if ( number < 0 )
	{
		OLED_ShowString_1206( 65, row_pos[2], "-", 1 ); OLED_ShowNum_12061( 72, row_pos[2], -number, 1 );
	}else { OLED_ShowString_1206( 65, row_pos[2], "+", 1 );  OLED_ShowNum_12061( 72, row_pos[2], number, 1 ); }     /* OLED_ShowString_1206(72,row_pos[0],"value1",1); */

	OLED_ShowString_1206( 16, row_pos[3], "string4", 1 );
	if ( number < 0 )
	{
		OLED_ShowString_1206( 65, row_pos[3], "-", 1 ); OLED_ShowNum_12061( 72, row_pos[3], -number, 1 );
	}else { OLED_ShowString_1206( 65, row_pos[3], "+", 1 );  OLED_ShowNum_12061( 72, row_pos[3], number, 1 ); }     /* OLED_ShowString_1206(72,row_pos[0],"value1",1); */

	OLED_ShowString_1206( 16, row_pos[4], "string5", 1 );
	if ( number < 0 )
	{
		OLED_ShowString_1206( 65, row_pos[4], "-", 1 ); OLED_ShowNum_12061( 72, row_pos[4], -number, 1 );
	}else { OLED_ShowString_1206( 65, row_pos[4], "+", 1 );  OLED_ShowNum_12061( 72, row_pos[4], number, 1 ); }     /* OLED_ShowString_1206(72,row_pos[0],"value1",1); */
}


void MenuShow3()
{
	OLED_ShowString_1206( 16, row_pos[0], "string1", 1 );  OLED_ShowNum_12061( 72, row_pos[0], number, 1 );         /* OLED_ShowString_1206(72,row_pos[0],"value1",1); ��ʾ���˸����������������Ǹ��������ͣ�������ʵ��ʱ����Ҫ���д��� */
	OLED_ShowString_1206( 16, row_pos[1], "string2", 1 );  OLED_ShowNum_12061( 72, row_pos[1], number, 1 );         /* OLED_ShowString_1206(72,row_pos[1],"value2",1); */
	OLED_ShowString_1206( 16, row_pos[2], "string3", 1 );  OLED_ShowNum_12061( 72, row_pos[2], number, 1 );         /* OLED_ShowString_1206(72,row_pos[2],"value3",1); */
	OLED_ShowString_1206( 16, row_pos[3], "string4", 1 );  OLED_ShowNum_12061( 72, row_pos[3], number, 1 );         /* OLED_ShowString_1206(72,row_pos[3],"value4",1); */
	OLED_ShowString_1206( 16, row_pos[4], "string5", 1 );  OLED_ShowNum_12061( 72, row_pos[4], number, 1 );         /* OLED_ShowString_1206(72,row_pos[4],"value5",1); */
}


/*
 * ����Ϊflash������˵��
 * ������init���Ժ�һ��Ҫ
 *        FLASH_Init();   flash��ʼ��
 *       flashDataRead();
 * flash�����Ǵ����ڴ��һ���������С�ԭ���ӹ�ֱ�ӽ�����
 * ���ǵ�Ҫ��ȡ�����ݴ���flashData[flashInfoNum]���档���԰�������һ����תվ������A��������Ҫ����flash��ȡ������ÿ�ο�����
 * flashDataRead()����ִ�й��������ٰ�flashData[0]��ֵ��A�����Ƕ�ȡ����д��ʱ������A��ֵ�ڳ������й����з����˱仯���Ȱ�A��ֵ��flashData[0]��
 * ��ִ��flashDataSave()��������ô���ǾͰ�������Ҫ��ȡ������д���˶�Ӧ������ÿ��������Ҫ��ȡ�����ݶ�������һһ��Ӧ��
 * ���ڳ��ν���flash����һ��Ҫ��д��һ���ڽ��д�ȡ�����Ȳ�ִ��flashDataRead();����ֱ��ִ��flashDataSave()
 * ֮����ÿ�ο����Ƚ���flashDataRead();
 * flash�������ޣ��Ͻ���ѭ�����ж���ʹ�ã�
 * flash�������ޣ��Ͻ���ѭ�����ж���ʹ�ã�
 * flash�������ޣ��Ͻ���ѭ�����ж���ʹ�ã�
 * flash�������ޣ��Ͻ���ѭ�����ж���ʹ�ã�
 * һ���ǰ���ĳ������������д��洢��ʵ�ֶϵ������
 * һ�����ڳ���ʼmain()������ʼʱִ��flashDataRead()������ȡ�ϵ�ǰ�����ݡ�������һ����ť�������°�ťʱִ��flashDataSave()����
 */


/* flashд�� */
void flashDataSave()
{
	FLASH_EraseSector( 245 );  FLASH_EraseSector( 244 );  FLASH_EraseSector( 243 );                               /* �Ȳ������� ��һ������2kb���������˰ɣ������Լ��ٿ�*/
	FLASH_WriteSector( 245, (uint8_t *)flashData, sizeof(flashData) );      /* ������ */
}


/* flash��ȡ */
void flashDataRead()
{
	Flash_read( 245, 0, sizeof(flashData),  (uint8_t *)flashData );
}


