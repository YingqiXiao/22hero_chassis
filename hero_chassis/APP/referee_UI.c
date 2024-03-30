#include "referee_UI.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "fifo.h"
#include "referee.h"
#include "bsp_referee.h"
#include "supercap.h"
#include "bsp_can.h"
#include "gimbal_task.h"
extern int time2;
#define Max(a,b) ((a) > (b) ? (a) : (b))
//#define Robot_ID_Current Robot_ID_Red_Infantry3
/* ����UIר�ýṹ�� */
UI_Graph1_t UI_Graph1;
UI_Graph2_t UI_Graph2;
UI_Graph5_t UI_Graph5;
UI_Graph7_t UI_Graph7;
UI_String_t UI_String;
UI_String_t UI_String1;
UI_String_t UI_String2;
UI_Graph5_t UI_Graph5_Arc;
UI_Delete_t UI_Delete;
uint8_t seq=0;
//-------------------------
uint8_t UI_AutoAim_Flag = 0;    //�Ƿ��������־λ
float   UI_Kalman_Speed = 0;    //������Ԥ���ٶ�
float   UI_Gimbal_Pitch = 1.000f; //��̨Pitch��Ƕ�
float   UI_Gimbal_Yaw   = 0.0f; //��̨Yaw��Ƕ�
uint8_t UI_Capacitance  = 10;   //����ʣ������
uint8_t UI_fric_is_on   = 0;    //Ħ�����Ƿ���
uint8_t Robot_ID_Current=0;
 
/* �����߸߶ȱ��� */
uint16_t y01 = 455;
uint16_t y02 = 420;
uint16_t y03 = 280;
uint16_t y04 = 230;


 extern uint8_t supercap_bettery_flag;
//----------------------------

static void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	unsigned char i=i;
	
	uint8_t tx_buff[MAX_SIZE];

	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //����֡����	

	memset(tx_buff,0,frame_length);  //�洢���ݵ���������
	
	/*****֡ͷ���*****/
	tx_buff[0] = sof;//����֡��ʼ�ֽ�
	memcpy(&tx_buff[1],(uint8_t*)&len, sizeof(len));//����֡��data�ĳ���
	tx_buff[3] = seq;//�����
	append_CRC8_check_sum(tx_buff,frameheader_len);  //֡ͷУ��CRC8

	/*****��������*****/
	memcpy(&tx_buff[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****���ݴ��*****/
	memcpy(&tx_buff[frameheader_len+cmd_len], p_data, len);
	append_CRC16_check_sum(tx_buff,frame_length);  //һ֡����У��CRC16

	if (seq == 0xff) seq=0;
  else seq++;
	
	/*****�����ϴ�*****/
	__HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_TC);
	HAL_UART_Transmit(&huart1, tx_buff,frame_length , 100);
	while (__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC) == RESET); //�ȴ�֮ǰ���ַ��������
}

static void get_UI_id(uint16_t *sender_ID,uint16_t *receiver_ID)
{
	switch(get_robot_id())
	{
		case UI_Data_RobotID_RHero:
		{
			*sender_ID=UI_Data_RobotID_RHero;
			*receiver_ID=UI_Data_CilentID_RHero;
			break;
		}
		case UI_Data_RobotID_REngineer:
		{
			*sender_ID=UI_Data_RobotID_REngineer;
			*receiver_ID=UI_Data_CilentID_REngineer;
			break;
		}
		case UI_Data_RobotID_RStandard1:
		{
			*sender_ID=UI_Data_RobotID_RStandard1;
			*receiver_ID=UI_Data_CilentID_RStandard1;
			break;
		}
		case UI_Data_RobotID_RStandard2:
		{
			*sender_ID=UI_Data_RobotID_RStandard2;
			*receiver_ID=UI_Data_CilentID_RStandard2;
			break;
		}
		case UI_Data_RobotID_RStandard3:
		{
			*sender_ID=UI_Data_RobotID_RStandard3;
			*receiver_ID=UI_Data_CilentID_RStandard3;
			break;
		}
		case UI_Data_RobotID_RAerial:
		{
			*sender_ID=UI_Data_RobotID_RAerial;
			*receiver_ID=UI_Data_CilentID_RAerial;
			break;
		}
		case UI_Data_RobotID_BHero:
		{
			*sender_ID=UI_Data_RobotID_BHero;
			*receiver_ID=UI_Data_CilentID_BHero;
			break;
		}
		case UI_Data_RobotID_BEngineer:
		{
			*sender_ID=UI_Data_RobotID_BEngineer;
			*receiver_ID=UI_Data_CilentID_BEngineer;
			break;
		}
		case UI_Data_RobotID_BStandard1:
		{
			*sender_ID=UI_Data_RobotID_BStandard1;
			*receiver_ID=UI_Data_CilentID_BStandard1;
			break;
		}	
		case UI_Data_RobotID_BStandard2:
		{
			*sender_ID=UI_Data_RobotID_BStandard2;
			*receiver_ID=UI_Data_CilentID_BStandard2;
			break;
		}	
		case UI_Data_RobotID_BStandard3:
		{
			*sender_ID=UI_Data_RobotID_BStandard3;
			*receiver_ID=UI_Data_CilentID_BStandard3;
			break;
		}	
		case UI_Data_RobotID_BAerial:
		{
			*sender_ID=UI_Data_RobotID_BAerial;
			*receiver_ID=UI_Data_CilentID_BAerial;
			break;
		}	
	}
}

uint16_t Sender_ID,Receiver_ID;
/************************************************����ֱ��*************************************************
**������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Start_x��Start_y    ��ʼ����
        End_x��End_y   ��������
**********************************************************************************************************/

void UI_Draw_Line(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
									uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
									uint16_t               Width,        //�߿�
									uint16_t               StartX,       //��ʼ����X
									uint16_t               StartY,       //��ʼ����Y
									uint16_t               EndX,         //��ֹ����X
									uint16_t               EndY)         //��ֹ����Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Line;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->end_x           = EndX;
	Graph->end_y           = EndY;
	
	
}

void UI_Draw_Rectangle(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                     char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									     uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									     uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	 	 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
							     	   uint16_t               Width,        //�߿�
							     		 uint16_t               StartX,       //��ʼ����X
							     		 uint16_t               StartY,       //��ʼ����Y
							     		 uint16_t               EndX,         //��ֹ����X
							     		 uint16_t               EndY)         //��ֹ����Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Rectangle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->end_x           = EndX;
	Graph->end_y           = EndY;
}

void UI_Draw_Circle(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                  char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									  uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									  uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										uint16_t               Width,        //�߿�
										uint16_t               CenterX,      //Բ������X
							      uint16_t               CenterY,      //Բ������Y
										uint16_t               Radius)       //�뾶
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Circle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->radius          = Radius;
	
}

void UI_Draw_Ellipse(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                   char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									   uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										 uint16_t               Width,        //�߿�
										 uint16_t               CenterX,      //Բ������X
							       uint16_t               CenterY,      //Բ������Y
										 uint16_t               XHalfAxis,    //X���᳤
										 uint16_t               YHalfAxis)    //Y���᳤
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Ellipse;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->end_x           = XHalfAxis;
	Graph->end_y           = YHalfAxis;
}

void UI_Draw_Arc(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	               char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								 uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							   uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
								 uint16_t               StartAngle,   //��ʼ�Ƕ� [0,360]
								 uint16_t               EndAngle,     //��ֹ�Ƕ� [0,360]
								 uint16_t               Width,        //�߿�
								 uint16_t               CenterX,      //Բ������X
							   uint16_t               CenterY,      //Բ������Y
								 uint16_t               XHalfAxis,    //X���᳤
								 uint16_t               YHalfAxis)    //Y���᳤
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Arc;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = StartAngle;
	Graph->end_angle       = EndAngle;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->end_x           = XHalfAxis;
	Graph->end_y           = YHalfAxis;
}

void UI_Draw_Float(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                 char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							     uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								   uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
									 uint16_t               NumberSize,   //�����С
									 uint16_t               Significant,  //��Чλ��
									 uint16_t               Width,        //�߿�
							     uint16_t               StartX,       //��ʼ����X
							     uint16_t               StartY,       //��ʼ����Y
									 float                  FloatData)    //��������
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Float;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = NumberSize;
	Graph->end_angle       = Significant;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	int32_t IntData = FloatData * 1000;
	Graph->radius          = (IntData & 0x000003ff) >>  0;
	Graph->end_x           = (IntData & 0x001ffc00) >> 10;
	Graph->end_y           = (IntData & 0xffe00000) >> 21;
	
}




void UI_Draw_String(string_data_struct_t *String,        //UIͼ�����ݽṹ��ָ��
	                  char                  StringName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							      uint8_t               StringOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								    uint8_t               Layer,         //UIͼ��ͼ�� [0,9]
							      uint8_t               Color,         //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										uint16_t              CharSize,      //�����С
									  uint16_t              StringLength,  //�ַ�������
									  uint16_t              Width,         //�߿�
							      uint16_t              StartX,        //��ʼ����X
							      uint16_t              StartY,        //��ʼ����Y
										char                 *StringData)    //�ַ�������
{
	String->string_name[0] = StringName[0];
	String->string_name[1] = StringName[1];
	String->string_name[2] = StringName[2];
	String->operate_tpye   = StringOperate;
	String->graphic_tpye   = UI_Graph_String;
	String->layer          = Layer;
	String->color          = Color;
	String->start_angle    = CharSize;
	String->end_angle      = StringLength;
	String->width          = Width;
	String->start_x        = StartX;
	String->start_y        = StartY;
	for(int i = 0; i < StringLength; i ++) String->stringdata[i] = *StringData ++;
}






void UI_PushUp_Graphs(uint8_t Counter /* 1,2,5,7 */, void *Graphs /* ��Counter��һ�µ�UI_Graphx�ṹ��ͷָ�� */, uint8_t RobotID)
{
	UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs; //����ֻ��һ������ͼ��
	
	/* ��� frame_header */
	Graph->Referee_Transmit_Header.SOF  = HEADER_SOF;
	     if(Counter == 1) Graph->Referee_Transmit_Header.data_length = 6 + 1 * 15;
	else if(Counter == 2) Graph->Referee_Transmit_Header.data_length = 6 + 2 * 15;
	else if(Counter == 5) Graph->Referee_Transmit_Header.data_length = 6 + 5 * 15;
	else if(Counter == 7) Graph->Referee_Transmit_Header.data_length = 6 + 7 * 15;
	Graph->Referee_Transmit_Header.seq  = Graph->Referee_Transmit_Header.seq + 1;
	Graph->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Graph->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	Graph->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	     if(Counter == 1) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw1;
	else if(Counter == 2) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw2;
	else if(Counter == 5) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw5;
	else if(Counter == 7) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw7;
	Graph->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	Graph->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	     if(Counter == 1)
	{
		UI_Graph1_t *Graph1 = (UI_Graph1_t *)Graphs;
		Graph1->CRC16 =CRC16_Calculate((uint8_t *)Graph1, sizeof(UI_Graph1_t) - 2);
	}
	else if(Counter == 2)
	{
		UI_Graph2_t *Graph2 = (UI_Graph2_t *)Graphs;
		Graph2->CRC16 = CRC16_Calculate((uint8_t *)Graph2, sizeof(UI_Graph2_t) - 2);
	}
	else if(Counter == 5)
	{
		UI_Graph5_t *Graph5 = (UI_Graph5_t *)Graphs;
		Graph5->CRC16 =CRC16_Calculate((uint8_t *)Graph5, sizeof(UI_Graph5_t) - 2);
	}
	else if(Counter == 7)
	{
		UI_Graph7_t *Graph7 = (UI_Graph7_t *)Graphs;
		Graph7->CRC16 = CRC16_Calculate((uint8_t *)Graph7, sizeof(UI_Graph7_t) - 2);
	}
	
	/* ʹ�ô���PushUp������ϵͳ */
	     if(Counter == 1) HAL_UART_Transmit(&huart1, (uint8_t *)Graph, sizeof(UI_Graph1_t),0xff);
	else if(Counter == 2) HAL_UART_Transmit(&huart1, (uint8_t *)Graph, sizeof(UI_Graph2_t),0xff);
	else if(Counter == 5) HAL_UART_Transmit(&huart1, (uint8_t *)Graph, sizeof(UI_Graph5_t),0xff);
	else if(Counter == 7) HAL_UART_Transmit(&huart1, (uint8_t *)Graph, sizeof(UI_Graph7_t),0xff);
}

void UI_PushUp_String(UI_String_t *String, uint8_t RobotID)
{
	/* ��� frame_header */
	String->Referee_Transmit_Header.SOF  = HEADER_SOF;
	String->Referee_Transmit_Header.data_length = 6 + 45;
	String->Referee_Transmit_Header.seq  = String->Referee_Transmit_Header.seq + 1;
	String->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&String->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	String->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	String->Interactive_Header.data_cmd_id = UI_DataID_DrawChar;
	String->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	String->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	String->CRC16 = CRC16_Calculate((uint8_t *)String, sizeof(UI_String_t) - 2);
	
	/* ʹ�ô���PushUp������ϵͳ */
	HAL_UART_Transmit(&huart1, (uint8_t *)String, sizeof(UI_String_t),0xff);
}

void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID)
{
	/* ��� frame_header */
	Delete->Referee_Transmit_Header.SOF  = HEADER_SOF;
	Delete->Referee_Transmit_Header.data_length = 6 + 2;
	Delete->Referee_Transmit_Header.seq  = Delete->Referee_Transmit_Header.seq + 1;
	Delete->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Delete->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	Delete->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	Delete->Interactive_Header.data_cmd_id = UI_DataID_Delete;
	Delete->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	Delete->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	Delete->CRC16 = CRC16_Calculate((uint8_t *)Delete, sizeof(UI_Delete_t) - 2);
	
	/* ʹ�ô���PushUp������ϵͳ */
	HAL_UART_Transmit(&huart1, (uint8_t *)Delete, sizeof(UI_Delete_t),0xff);
}
uint16_t UI_PushUp_Counter = 0;

void referee_usart_task()
{
	/* ��̬UI���Ʊ��� */
	Robot_ID_Current=get_robot_id();
	float    Capacitance_X;
		
		/* UI���� */
		UI_PushUp_Counter++;

		if(UI_PushUp_Counter % 91 == 0) //��̬UIԤ���� ͼ��
		{
		UI_Draw_Float (&UI_Graph2.Graphic[0], "201", UI_Graph_Add, 3, UI_Color_Orange, 22, 3, 3, 1355, 632, 1.0f);   //Pith��Ƕ�
		UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
		}	
	
		if(UI_PushUp_Counter % 101 == 0) //��̬UIԤ���� ������1
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_White, 2,  840,   y01,  920,   y01); //��һ�������
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add, 0, UI_Color_White, 2,  950,   y01,  970,   y01); //��һ��ʮ�ֺ�
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add, 0, UI_Color_White, 2,1000,   y01, 1080,   y01); //��һ���Һ���
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add, 0, UI_Color_White, 2,  960,y01-10,  960,y01+10); //��һ��ʮ����
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add, 0,UI_Color_White, 2,  870,   y02, 930,   y02); //�ڶ��������
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add, 0, UI_Color_White, 6,  959,   y02,  960,   y02); //�ڶ������ĵ�
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add, 0, UI_Color_White, 2,  990,   y02, 1050,   y02); //�ڶ����Һ���
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
		}
		if(UI_PushUp_Counter % 111 == 0) //��̬UIԤ���� ������2
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, UI_Color_White, 2,  900,   y03,  940,   y03); //�����������
			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, UI_Color_White, 6,  959,   y03,  960,   y03); //���������ĵ�
			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, UI_Color_White, 2,  980,   y03, 1020,   y03); //�������Һ���
			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, UI_Color_White, 2,  930,   y04,  950,   y04); //�����������
			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, UI_Color_White, 6,  959,   y04,  960,   y04); //���������ĵ�
			UI_Draw_Line(&UI_Graph7.Graphic[5], "013", UI_Graph_Add, 0, UI_Color_White, 2,  970,   y04,  990,   y04); //�������Һ���
			UI_Draw_Line(&UI_Graph7.Graphic[6], "014", UI_Graph_Add, 0,UI_Color_White, 2,  960,y04-10,  960,y04-30); //������������
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
		}
		
				if(UI_PushUp_Counter % 131 == 0) //��̬UIԤ���� ԲȦ
		{ 	UI_Draw_Arc(&UI_Graph5_Arc.Graphic[0],"106",UI_Graph_Add,2,UI_Color_White,0,360,5,150,590,15,15);
			UI_Draw_Arc(&UI_Graph5_Arc.Graphic[1],"107",UI_Graph_Add,2,UI_Color_White,0,360,5,150,640,15,15);
			UI_Draw_Arc(&UI_Graph5_Arc.Graphic[2],"108",UI_Graph_Add,2,UI_Color_White,0,360,5,150,690,15,15);
			UI_Draw_Arc(&UI_Graph5_Arc.Graphic[3],"109",UI_Graph_Add,2,UI_Color_White,0,360,5,150,740,15,15);
			UI_PushUp_Graphs(5, &UI_Graph5_Arc, Robot_ID_Current);
		}


		if(UI_PushUp_Counter % 141 == 0) //��̬UIԤ���� �ַ���
		{
			UI_Draw_String(&UI_String.String, "304", UI_Graph_Add, 2, UI_Color_White, 18, 6, 3,  38, 600, "auto");//����
			UI_PushUp_String(&UI_String, Robot_ID_Current);			
		}
		
		if(UI_PushUp_Counter % 151 == 0) //��̬UIԤ���� �ַ���1
		{
			UI_Draw_String(&UI_String1.String, "305", UI_Graph_Add, 2, UI_Color_White, 18, 6, 3,  38, 650, "follow");//��������
			UI_PushUp_String(&UI_String1, Robot_ID_Current);			
		}
		
		if(UI_PushUp_Counter % 161 == 0) //��̬UIԤ���� �ַ���2
		{
			UI_Draw_String(&UI_String2.String, "306", UI_Graph_Add, 2, UI_Color_White,18, 4, 3,  38, 700, "spin");//С����	
			UI_PushUp_String(&UI_String2, Robot_ID_Current);	
		}
		
		if(UI_PushUp_Counter % 171 == 0) //��̬UIԤ���� �ַ���3
		{
			UI_Draw_String(&UI_String2.String, "307", UI_Graph_Add, 2, UI_Color_White,18, 4, 3,  38, 750, "fric");//С����	
			UI_PushUp_String(&UI_String2, Robot_ID_Current);	
		}


		if(UI_PushUp_Counter % 21 == 0) //��̬UI���� ԲȦλ�ú���ɫ
		{
			
			UI_Draw_Float (&UI_Graph2.Graphic[0], "201", UI_Graph_Change, 3, UI_Color_Orange, 22, 3, 3, 1355, 632, gimbal_move.imu->imu_pitch / 100.0f );   //Pith��Ƕ�
			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);

			if(channel[0].sR == 1)
			{
				UI_Draw_Arc(&UI_Graph5_Arc.Graphic[0],"106",UI_Graph_Change,2,UI_Color_Green,0,360,5,150,590,15,15);
			}
			
			else
			{
				UI_Draw_Arc(&UI_Graph5_Arc.Graphic[0],"106",UI_Graph_Change,2,UI_Color_White,0,360,5,150,590,15,15);
			}
		
			if(channel[0].sR == 3 && channel[0].sL == 3)
			{
				UI_Draw_Arc(&UI_Graph5_Arc.Graphic[1],"107",UI_Graph_Change,2,UI_Color_Green,0,360,5,150,640,15,15);
			}
			
			else
			{
				UI_Draw_Arc(&UI_Graph5_Arc.Graphic[1],"107",UI_Graph_Change,2,UI_Color_White,0,360,5,150,640,15,15);
			}			

			if(channel[0].sR == 3 && channel[0].sL == 2)
			{
				UI_Draw_Arc(&UI_Graph5_Arc.Graphic[1],"108",UI_Graph_Change,2,UI_Color_Green,0,360,5,150,690,15,15);
			}
			
			else
			{
				UI_Draw_Arc(&UI_Graph5_Arc.Graphic[1],"108",UI_Graph_Change,2,UI_Color_White,0,360,5,150,690,15,15);
			}
			
			if(fric[0].fric_flag == 1)
			{
				UI_Draw_Arc(&UI_Graph5_Arc.Graphic[1],"109",UI_Graph_Change,2,UI_Color_Green,0,360,5,150,740,15,15);
			}
			
			else
			{
				UI_Draw_Arc(&UI_Graph5_Arc.Graphic[1],"109",UI_Graph_Change,2,UI_Color_White,0,360,5,150,740,15,15);
			}			
			UI_PushUp_Graphs(5, &UI_Graph5_Arc, Robot_ID_Current);
//				if(vision_mode==0)
//				{
//			  UI_Draw_Arc(&UI_Graph7.Graphic[0],"106",UI_Graph_Change,2,UI_Color_Green,0,360,5,200,  590,15,15);//vision_on
//				}
//				else if(vision_mode==1)
//					
//				{
//			UI_Draw_Arc(&UI_Graph7.Graphic[0],"106",UI_Graph_Change,2,UI_Color_White,0,360,5,150,  590,15,15);//vision_off
//				}
//				if( supercap_reboot_flag==0)//������־λ
//				{
//					UI_Draw_Arc(&UI_Graph7.Graphic[2],"107",UI_Graph_Change,2,UI_Color_White ,0,360,5,150, 640,15,15);//supercup_off
//				}
//				if( supercap_reboot_flag==1)
//				{
//					UI_Draw_Arc(&UI_Graph7.Graphic[2],"107",UI_Graph_Change,2,UI_Color_Pink ,0,360,5,200,  640,15,15);//supercup_on
//				}
//				 if(chassis_control_order.chassis_mode==CHASSIS_FOLLOW)
				
//				if(channel[0].sL == 2 && channel[0].sR == 3)
//				{
//					UI_Draw_Arc(&UI_Graph7.Graphic[1],"108",UI_Graph_Change,2,UI_Color_Green ,0,360,5,200,690,15,15);//spin_on
//				}

//				 else	 
//				{
//				UI_Draw_Arc(&UI_Graph7.Graphic[1],"108",UI_Graph_Change,2,UI_Color_White,0,360,5,150,690,15,15);//spin_off
//				}

//				
//				if(channel[0].sL != 3 && channel[0].sR != 3) 	 
//				{
//				UI_Draw_Arc(&UI_Graph7.Graphic[1],"109",UI_Graph_Change,2,UI_Color_White,0,360,0,10,10,15,15);//spin_off
//				}
////				else if(chassis_control_order.chassis_mode==CHASSIS_SPIN)
//				else if(channel[0].sL == 3 && channel[0].sR == 3)
//				{
//					UI_Draw_Arc(&UI_Graph7.Graphic[1],"109",UI_Graph_Change,2,UI_Color_Green ,0,360,0,10,10,15,15);//spin_on
//				}				
//				if(target_exit==0)
//				{  
//					UI_Draw_Arc(&UI_Graph7.Graphic[3],"109",UI_Graph_Change,2,UI_Color_White,0,360,0,10,10,15,15);
//			
//				}	
//				else if((target_exit==1)&&(vision_mode==0))
//				{ 
//					UI_Draw_Arc(&UI_Graph7.Graphic[3],"109",UI_Graph_Change,2,UI_Color_Green,0,360,5,960,540,15,15);
//				}	
//				if(reboot_flag==1)
//				{UI_Draw_Arc(&UI_Graph7.Graphic[4],"110",UI_Graph_Change,2,UI_Color_Pink,0,360,5,1375,700,15,15);
//				}
//				else if(reboot_flag==0)
//	    	{UI_Draw_Arc(&UI_Graph7.Graphic[4],"110",UI_Graph_Change,2,UI_Color_Pink,0,360,0,1355,590,15,15);
//				}
//			 UI_Draw_Float (&UI_Graph7.Graphic[5], "201", UI_Graph_Change, 3, UI_Color_Orange, 22, 3, 3, 1355, 632, supercap_per); 
				
//			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
		}
				
		
		}