

#define  KEY_UP		1       /*�� */
#define  KEY_DOWN	2       /*�� */
#define  KEY_LEFT	3       /* �� */
#define  KEY_RIGHT	4       /* �� */
#define  KEY_MID	5       /*ѡ�� */
#define pageMax		3       /* ���ҳ�� */
extern int			keyState;
extern int			menuRow;
extern int			temp1;
extern int			temp2;
extern int			temp3;
extern int			temp4;
extern int			temp5;
extern int			temp6;
extern int			temp7;
extern int			key_counter;
extern int			key_temp;
extern const unsigned char	row_pos[5];
extern int clearCount;

extern void menu();

extern void flashWrite();
extern void SignMove();


extern int keyCheck( void );


extern void FlashValueOperate();


extern void MenuShow3();


extern void MenuShow2();

extern void flashDataSave();
extern void flashDataRead();

