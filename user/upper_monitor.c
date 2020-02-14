#include "upper_monitor.h"
/*
 * ɽ��๦�ܵ���������λ��������ʾ������ʾ����
 * wareaddr    ����������ʼ��ַ
 * waresize    ��������ռ�ÿռ�Ĵ�С
 * example:
 *        uint32_t var[4]={1,2,3,4};
 *        vcan_sendware((uint8_t *)var,sizeof(var));
 *
 */
void vcan_sendware( uint8_t *wareaddr, uint32_t waresize )
{
#define CMD_WARE 3
	uint8_t cmdf[2] = { CMD_WARE, ~CMD_WARE };      /* ���ڵ��� ʹ�õ�ǰ���� */
	uint8_t cmdr[2] = { ~CMD_WARE, CMD_WARE };      /* ���ڵ��� ʹ�õĺ����� */
	uart_putbuff( HW_UART3, cmdf, sizeof(cmdf) );   /* �ȷ���ǰ���� */
	uart_putbuff( HW_UART3, wareaddr, waresize );   /* �������� */
	uart_putbuff( HW_UART3, cmdr, sizeof(cmdr) );   /* ���ͺ����� */
}


/*
 * ɽ��๦�ܵ���������λ��������ͷ��ʾ����
 * imgaddr    ͼ����ʼ��ַ
 * imgsize    ͼ��ռ�ÿռ�Ĵ�С
 * example:   vcan_sendimg(imgadd,row_num*col_num);   imgaddΪ��Ҫ�����ͼ������
 */
void vcan_sendimg( uint8_t *imgaddr, uint32_t imgsize )
{
#define CMD_IMG 1
	uint8_t cmdf[2] = { CMD_IMG, ~CMD_IMG };                /* ɽ����λ�� ʹ�õ����� */
	uint8_t cmdr[2] = { ~CMD_IMG, CMD_IMG };                /* ɽ����λ�� ʹ�õ����� */

	uart_putbuff( HW_UART3, cmdf, sizeof(cmdf) );           /* �ȷ������� */

	uart_putbuff( HW_UART3, (char *) imgaddr, imgsize );    /* �ٷ���ͼ�� */

	uart_putbuff( HW_UART3, cmdr, sizeof(cmdr) );           /* �ȷ������� */
}