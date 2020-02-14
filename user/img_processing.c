
#include "varieble.h"
#include "img_processing.h"

#include "oled.h"

extern uint8_t L_flag,R_flag;
extern int Enterflag;
uint8_t Angleflag;

/*
 * @name		searchline_OV7620
 * @description	To get the racing track from the imgadd.
 * @inputval	None
 * @outputval	None
 * @retval      0
 */

void searchline_OV7620( void )
{
	int	CurL		= 0, Start = 0;                                                                                                                                                         /* CurL  ��ǰ��   Start ��ʼɨ�ߵ���  ��һ�д�0��ʼ */
	int	Cur_Offset	= car_center;                                                                                                                                                           /* ��ʼɨ������Ϊ80���˱���Ϊ��ǰ������ */

	int CurPoint = Cur_Offset;                                                                                                                                                                      /* CurPointΪ��ǰ����ɨ��ĵ� */
/*
 * ע�⣺���½�Ϊ��0,0��
 * ====================����ɨ��������===============================
 */
	for ( CurL = Start; CurL < row_num; ++CurL )
	{
		CurPoint = Cur_Offset;                                                                                                                                                                  /* CurPoint��ÿһ�п�ʼʱΪ�е� */
		/*
		 * ɨ�߿�ʼ
		 * ����,�ұ߽�ֵӦ�ô���0
		 */
		while ( CurPoint > 0 )
		{
			if ( *(imgadd + CurL * col_num + CurPoint) < whiteRoad && *(imgadd + CurL * col_num + CurPoint - 1) < whiteRoad && *(imgadd + CurL * col_num + CurPoint - 2) < whiteRoad )      /* �ҵ��ұ߽�  ���ҽ���ȥ�� */
			{
				Rx[CurL] = CurPoint;
				break;
			}else  {                                                                                                                                                                        /* û�ҵ��������ƶ�һ�����ص� */
				--CurPoint;
			}
		}
		/* ���� */
		CurPoint = Cur_Offset;                                                                                                                                                                  /* CurPoint��ÿһ�п�ʼʱΪ�е� */

		while ( CurPoint < col_num )
		{
			if ( *(imgadd + CurL * col_num + CurPoint) < whiteRoad && *(imgadd + CurL * col_num + CurPoint + 1) < whiteRoad && *(imgadd + CurL * col_num + CurPoint + 2) < whiteRoad )      /* �ҵ���߽�  ���ҽ���ȥ�� */
			{
				Lx[CurL] = CurPoint;
				break;
			}else  {                                                                                                                                                                        /* û�ҵ��������ƶ�һ�����ص� */
				++CurPoint;
			}
		}
                
                //Ѱ����Զ��Ч��
                if(farflag==0&&Rx[CurL]==Lx[CurL])
                {
                  farflag=1;
                  farline=CurL;
                }
                
		Midx[CurL]	= (Lx[CurL] + Rx[CurL]) >> 1;
		Cur_Offset	= Midx[CurL];
	} /* //////��ɨ��for������//////// */
}


/*
 * @name			dispimage
 * @description	Display the image or racing track on OLED screen.
 * @inputval		None
 * @outputval	None
 * @retval              0
 */
unsigned char display_col[158] = { 0,	0,   1,	  2,   3,   4,	 4,   5,   6,	7,   8,	  8,   9,   10,	 11,  12, 12, 13, 14,
				   15,	16,  17,  17,  18,  19,	 20,  21,  21,	22,  23,  24,  25,  25,	 26,  27,
				   28,	29,  29,  30,  31,  32,	 33,  34,  34,	35,  36,  37,  38,  38,	 39,  40,
				   41,	42,  42,  43,  44,  45,	 46,  46,  47,	48,  49,  50,  51,  51,	 52,  53,
				   54,	55,  55,  56,  57,  58,	 59,  59,  60,	61,  62,  63,  64,  64,	 65,  66,
				   67,	68,  68,  69,  70,  71,	 72,  72,  73,	74,  75,  76,  76,  77,	 78,  79,
				   80,	81,  81,  82,  83,  84,	 85,  85,  86,	87,  88,  89,  89,  90,	 91,  92,
				   93,	93,  94,  95,  96,  97,	 98,  98,  99,	100, 101, 102, 102, 103, 104,
				   105, 106, 106, 107, 108, 109, 110, 110, 111, 112, 113, 114, 115,
				   115, 116, 117, 118, 119, 119, 120, 121, 122, 123, 123, 124, 125, 126, 127 };

void dispimage( void )
{
	uint16_t	i	= 0, j = 0;
	uint16_t	state	= 0;


	/*
	 * OLED_Clear();
	 * ʹ��OLED��������ͷ��ͼ��
	 */
	for ( i = 0; i < row_num; i++ )
	{
		for ( j = 0; j < col_num; j++ )
		{
			if ( imgadd[i * col_num + j] > whiteRoad )
			{
				OLED_DrawPoint( display_col[j], i + 14, 0 );
			}else {
				OLED_DrawPoint( display_col[j], i + 14, 1 );
			}
		}
	}
	OLED_Refresh_Gram();
}


/* ��ʾ�ҵ��ı��� */
void dispimage1( void )
{
	uint16_t	i	= 0, j = 0;
	uint16_t	state	= 0;


	/*
	 * OLED_Clear();
	 * ʹ��OLED��������ͷ��ͼ��
	 */
	for ( i = 0; i < row_num; i++ )
	{
		for ( j = 0; j < col_num; j++ )
		{
			if ( imgadd[i * col_num + j] > whiteRoad )
			{
				OLED_DrawPoint( display_col[j], i + 14, 1 );   
			}else {
				OLED_DrawPoint( display_col[j], i + 14, 0 );
			}
		}


		/* �����ҵ��ı߽��� */
		if ( Lx[i] != col_num && Lx[i] > 0 )
			OLED_DrawPoint( display_col[Lx[i]], i + 14, 0 );
		if ( Rx[i] != 0 && Rx[i] < col_num )
			OLED_DrawPoint( display_col[Rx[i]], i + 14, 0 );
                
                OLED_DrawPoint(display_col[Midx[i]],i+14,0);
	}
	OLED_Refresh_Gram();
}

//ƫ����
void GetOffset()
{
  int sum = 0;
  for(int i = 27;i < 32;i++)
  {
    sum += Midx[i];
  }
  Offset = sum / 5 - 70;
}




//��������(��
bool IsLeftHop()
{
  int i;
  //�ɴ��С
  for(i=15;i<38;i++)
  {
    if(Lx[i]-Lx[i+1]>10 && Lx[i] > Lx[i+1])
    return true;
  }
    return false;
}

//��������(�ң�
bool IsRightHop()
{
 int i;
  for(i=15;i<38;i++)
  {
    if(Rx[i+1]-Rx[i]>10&&Rx[i+1]>Rx[i])
    return true;
  }
    return false;
}

//��յ�
bool IsLeftPoint()
{
  
  int i;
  for(i=1;i<40;i++)
  {
  if((Lx[i-1]>Lx[i])&&(Lx[i+1]>Lx[i])||(Lx[i-1]<Lx[i])&&(Lx[i+1]<Lx[i]))
    return true;
  }
  return false;
}

//�ҹյ�
bool IsRightPoint()
{
  int i;
  for(i=1;i<40;i++)
  {
  if((Rx[i-1]>Rx[i])&&(Rx[i+1]>Rx[i])||(Rx[i-1]<Rx[i])&&(Rx[i+1]<Rx[i]))
    return true;
  }
  return false;
}

extern int pointx,pointy;
//�з���ս�(�н�����)
bool IsRowAngle()
{
  int i,j;          //iΪ�У�jΪ��
  int point1,point2,point3;
  Angleflag=0;
  //����ɨ��
  for(j=5;j<150;j++)
  {
   for(i=40;i>0;i--) 
   {
    if((*(imgadd+i*j)>whiteRoad)&&(*imgadd+(i-1)*j>whiteRoad)&&(*(imgadd+(i+1)*j)<whiteRoad)&&(*(imgadd+(i+2)*j)<whiteRoad))
      point1=i;    //ĳһ�е���ֵ����������
    
    if(*(imgadd+i*(j-2))>whiteRoad&&*(imgadd+(i-1)*(j-2))>whiteRoad&&*(imgadd+(i+1)*(j-2))<whiteRoad&&*(imgadd+(i+2)*(j-2))<whiteRoad) 
      point2=i;    //�������ҵڶ��е���ֵ����������
    
    if(*(imgadd+i*(j+2))>whiteRoad&&*(imgadd+(i-1)*(j+3))>whiteRoad&&*(imgadd+(i+1)*(j+3))<whiteRoad&&*(imgadd+(i+2)*(j+3))<whiteRoad) 
      point3=i;    //��������ڶ��е���ֵ����������
    }
   if(point1<point2&&point1<point3)
   {
     Angleflag=1;
     pointx = j;
     pointy = i;
    return true;   //��⵽�ս�
   }
  }
   return false;
  
}

//Բ��
bool IsCircle()
{
  L_flag == 0;
  R_flag == 0;
    Enterflag=0;
  //ʹ�ñ���������з���յ�ȷ��
 
  if(IsLeftHop()&&!IsRightHop()) 
  {
   L_flag = 1;
   Enterflag=1;
   return true;
  }
  if(!IsLeftHop()&&IsRightHop()) 
  {
    R_flag = 1;
    Enterflag=1;
    return true;
  }
  else return false;
}
  
bool IsEmpty()
{
  int i,j;
  int counter=0;
  for(i=20;i<40;i++)
    for(j=0;j<145;j++)
    {
      if(*(imgadd+i*j)<whiteRoad)
        counter++;
    }
  if(counter>2000) 
    return true;
  else return false;
}


bool IsImLine()
{
 int i,j, counter=0;
 int imcounter = 0;
 for(i=10;i<45;i++)
 {
   counter = 0;
   for(j=10;j<145;j++)
   {
     if((*(imgadd+i*col_num + j)<whiteRoad)&&(*(imgadd+i*col_num + j + 1)>whiteRoad)) 
       counter++;
   }
  if(counter>5) return true;
 //   imcounter++;
   
 }
 /*
 if(imcounter > 2)
   return true;
*/
 return false;
}


