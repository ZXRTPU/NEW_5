/**
 * @file referee_UI.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-1-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_UI.h"
#include "string.h"
#include "crc_ref.h"
#include "stdio.h"
#include "rm_referee.h"
#include "stdarg.h"

// �����
/********************************************ɾ������*************************************
**������_id ��Ӧ��id�ṹ��
		Del_Operate  ��Ӧͷ�ļ�ɾ������
		Del_Layer    Ҫɾ���Ĳ� ȡֵ0-9
*****************************************************************************************/
void UIDelete(referee_id_t *_id, uint8_t Del_Operate, uint8_t Del_Layer)
{
	static UI_delete_t UI_delete_data;
	uint8_t temp_datalength = Interactive_Data_LEN_Head + UI_Operate_LEN_Del; // ���㽻�����ݳ���

	UI_delete_data.FrameHeader.SOF = REFEREE_SOF;
	UI_delete_data.FrameHeader.DataLength = temp_datalength;
	UI_delete_data.FrameHeader.Seq = UI_Seq;
	UI_delete_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&UI_delete_data, LEN_CRC8, 0xFF);

	UI_delete_data.CmdID = ID_student_interactive;

	UI_delete_data.datahead.data_cmd_id = UI_Data_ID_Del;
	UI_delete_data.datahead.receiver_ID = _id->Cilent_ID;
	UI_delete_data.datahead.sender_ID = _id->Robot_ID;

	UI_delete_data.Delete_Operate = Del_Operate; // ɾ������
	UI_delete_data.Layer = Del_Layer;

	UI_delete_data.frametail = Get_CRC16_Check_Sum((uint8_t *)&UI_delete_data, LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);
	/* ����0xFFFF,����crcУ�� */

	RefereeSend((uint8_t *)&UI_delete_data, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // ����

	UI_Seq++; // �����+1
}
/************************************************����ֱ��*************************************************
**������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
		Graph_Operate   ͼƬ��������ͷ�ļ�
		Graph_Layer    ͼ��0-9
		Graph_Color    ͼ����ɫ
		Graph_Width    ͼ���߿�
		Start_x��Start_y  ���xy����
		End_x��End_y   �յ�xy����
**********************************************************************************************************/

void UILineDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
				uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
	int i;
	for (i = 0; i < 3 && graphname[i] != '\0'; i++) // �������0��Ϊֹ
	{
		graph->graphic_name[2 - i] = graphname[i]; // ���ڴ��ַ��������䣬���Ի���i��2-i
	}

	graph->operate_tpye = Graph_Operate;
	graph->graphic_tpye = UI_Graph_Line;
	graph->layer = Graph_Layer;
	graph->color = Graph_Color;

	graph->start_angle = 0;
	graph->end_angle = 0;
	graph->width = Graph_Width;
	graph->start_x = Start_x;
	graph->start_y = Start_y;
	graph->radius = 0;
	graph->end_x = End_x;
	graph->end_y = End_y;
}

/************************************************���ƾ���*************************************************
**������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
		Graph_Operate   ͼƬ��������ͷ�ļ�
		Graph_Layer    ͼ��0-9
		Graph_Color    ͼ����ɫ
		Graph_Width    ͼ���߿�
		Start_x��Start_y    ���xy����
		End_x��End_y        �ԽǶ���xy����
**********************************************************************************************************/
void UIRectangleDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
					 uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
	int i;
	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
	{
		graph->graphic_name[2 - i] = graphname[i];
	}

	graph->graphic_tpye = UI_Graph_Rectangle;
	graph->operate_tpye = Graph_Operate;
	graph->layer = Graph_Layer;
	graph->color = Graph_Color;

	graph->start_angle = 0;
	graph->end_angle = 0;
	graph->width = Graph_Width;
	graph->start_x = Start_x;
	graph->start_y = Start_y;
	graph->radius = 0;
	graph->end_x = End_x;
	graph->end_y = End_y;
}

/************************************************������Բ*************************************************
**������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
		Graph_Operate   ͼƬ��������ͷ�ļ�
		Graph_Layer    ͼ��0-9
		Graph_Color    ͼ����ɫ
		Graph_Width    ͼ���߿�
		Start_x��Start_y    Բ��xy����
		Graph_Radius  Բ�ΰ뾶
**********************************************************************************************************/

void UICircleDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
				  uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
{
	int i;
	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
	{
		graph->graphic_name[2 - i] = graphname[i];
	}

	graph->graphic_tpye = UI_Graph_Circle;
	graph->operate_tpye = Graph_Operate;
	graph->layer = Graph_Layer;
	graph->color = Graph_Color;

	graph->start_angle = 0;
	graph->end_angle = 0;
	graph->width = Graph_Width;
	graph->start_x = Start_x;
	graph->start_y = Start_y;
	graph->radius = Graph_Radius;
	graph->end_x = 0;
	graph->end_y = 0;
}
/************************************************������Բ*************************************************
**������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
		Graph_Operate   ͼƬ��������ͷ�ļ�
		Graph_Layer    ͼ��0-9
		Graph_Color    ͼ����ɫ
		Graph_Width    ͼ���߿�
		Start_x��Start_y    Բ��xy����
		End_x��End_y        xy���᳤��
**********************************************************************************************************/
void UIOvalDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
				uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y)
{
	int i;
	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
	{
		graph->graphic_name[2 - i] = graphname[i];
	}

	graph->graphic_tpye = UI_Graph_Ellipse;
	graph->operate_tpye = Graph_Operate;
	graph->layer = Graph_Layer;
	graph->color = Graph_Color;
	graph->width = Graph_Width;

	graph->start_angle = 0;
	graph->end_angle = 0;
	graph->width = Graph_Width;
	graph->start_x = Start_x;
	graph->start_y = Start_y;
	graph->radius = 0;
	graph->end_x = end_x;
	graph->end_y = end_y;
}

/************************************************����Բ��*************************************************
**������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
		Graph_Operate   ͼƬ��������ͷ�ļ�
		Graph_Layer    ͼ��0-9
		Graph_Color    ͼ����ɫ
		Graph_StartAngle,Graph_EndAngle    ��ʼ��ֹ�Ƕ�
		Graph_Width    ͼ���߿�
		Start_y,Start_y    Բ��xy����
		x_Length,y_Length   xy���᳤��
**********************************************************************************************************/

void UIArcDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
			   uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
			   uint32_t end_x, uint32_t end_y)
{
	int i;
	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
	{
		graph->graphic_name[2 - i] = graphname[i];
	}

	graph->graphic_tpye = UI_Graph_Arc;
	graph->operate_tpye = Graph_Operate;
	graph->layer = Graph_Layer;
	graph->color = Graph_Color;

	graph->start_angle = Graph_StartAngle;
	graph->end_angle = Graph_EndAngle;
	graph->width = Graph_Width;
	graph->start_x = Start_x;
	graph->start_y = Start_y;
	graph->radius = 0;
	graph->end_x = end_x;
	graph->end_y = end_y;
}

/************************************************���Ƹ���������*************************************************
**������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
		Graph_Operate   ͼƬ��������ͷ�ļ�
		Graph_Layer    ͼ��0-9
		Graph_Color    ͼ����ɫ
		Graph_Size     �ֺ�
		Graph_Digit    С��λ��
		Graph_Width    ͼ���߿�
		Start_x��Start_y    ��ʼ����
		radius=a&0x3FF;   aΪ����������1000���32λ������
		end_x=(a>>10)&0x7FF;
		end_y=(a>>21)&0x7FF;
**********************************************************************************************************/

void UIFloatDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
				 uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Float)
{

	int i;
	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
	{
		graph->graphic_name[2 - i] = graphname[i];
	}
	graph->graphic_tpye = UI_Graph_Float;
	graph->operate_tpye = Graph_Operate;
	graph->layer = Graph_Layer;
	graph->color = Graph_Color;

	graph->width = Graph_Width;
	graph->start_x = Start_x;
	graph->start_y = Start_y;
	graph->start_angle = Graph_Size;
	graph->end_angle = Graph_Digit;

	graph->radius = Graph_Float & 0x3FF;
	graph->end_x = (Graph_Float >> 10) & 0x7FF;
	graph->end_y = (Graph_Float >> 21) & 0x7FF;
}

/************************************************������������*************************************************
**������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
		Graph_Operate   ͼƬ��������ͷ�ļ�
		Graph_Layer    ͼ��0-9
		Graph_Color    ͼ����ɫ
		Graph_Size     �ֺ�
		Graph_Width    ͼ���߿�
		Start_x��Start_y    ��ʼ����
		radius=a&0x3FF;   aΪ32λ������
		end_x=(a>>10)&0x7FF;
		end_y=(a>>21)&0x7FF;
**********************************************************************************************************/
void UIIntDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
			   uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer)
{
	int i;
	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
	{
		graph->graphic_name[2 - i] = graphname[i];
	}
	graph->graphic_tpye = UI_Graph_Int;
	graph->operate_tpye = Graph_Operate;
	graph->layer = Graph_Layer;
	graph->color = Graph_Color;

	graph->start_angle = Graph_Size;
	graph->end_angle = 0;
	graph->width = Graph_Width;
	graph->start_x = Start_x;
	graph->start_y = Start_y;
	graph->radius = Graph_Integer & 0x3FF;
	graph->end_x = (Graph_Integer >> 10) & 0x7FF;
	graph->end_y = (Graph_Integer >> 21) & 0x7FF;
}

/************************************************�����ַ�������*************************************************
**������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
		Graph_Operate   ͼƬ��������ͷ�ļ�
		Graph_Layer    ͼ��0-9
		Graph_Color    ͼ����ɫ
		Graph_Size     �ֺ�
		Graph_Width    ͼ���߿�
		Start_x��Start_y    ��ʼ����

**������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
		fmt��Ҫ��ʾ���ַ���
		�˺�����ʵ�ֺ;���ʹ��������printf����
**********************************************************************************************************/
void UICharDraw(String_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
				uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *fmt, ...)
{
	int i;
	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
	{
		graph->Graph_Control.graphic_name[2 - i] = graphname[i];
	}

	graph->Graph_Control.graphic_tpye = UI_Graph_Char;
	graph->Graph_Control.operate_tpye = Graph_Operate;
	graph->Graph_Control.layer = Graph_Layer;
	graph->Graph_Control.color = Graph_Color;

	graph->Graph_Control.width = Graph_Width;
	graph->Graph_Control.start_x = Start_x;
	graph->Graph_Control.start_y = Start_y;
	graph->Graph_Control.start_angle = Graph_Size;
	graph->Graph_Control.radius = 0;
	graph->Graph_Control.end_x = 0;
	graph->Graph_Control.end_y = 0;

	va_list ap;
	va_start(ap, fmt);
	vsprintf((char *)graph->show_Data, fmt, ap); // ʹ�ò����б���и�ʽ����������ַ���
	va_end(ap);
	graph->Graph_Control.end_angle = strlen((const char *)graph->show_Data);
}

/* UI���ͺ�����ʹ������Ч��
   ������ cnt   ͼ�θ���
			...   ͼ�α�������
   Tips�����ú���ֻ������1��2��5��7��ͼ�Σ�������ĿЭ��δ�漰
 */
void UIGraphRefresh(referee_id_t *_id, int cnt, ...)
{
	UI_GraphReFresh_t UI_GraphReFresh_data;
	Graph_Data_t graphData;

	uint8_t temp_datalength = LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * cnt + LEN_TAIL; // ���㽻�����ݳ���

	static uint8_t buffer[512]; // �������ݻ���

	va_list ap;		   // ����һ�� va_list ���ͱ���
	va_start(ap, cnt); // ��ʼ�� va_list ����Ϊһ�������б�

	UI_GraphReFresh_data.FrameHeader.SOF = REFEREE_SOF;
	UI_GraphReFresh_data.FrameHeader.DataLength = Interactive_Data_LEN_Head + cnt * UI_Operate_LEN_PerDraw;
	UI_GraphReFresh_data.FrameHeader.Seq = UI_Seq;
	UI_GraphReFresh_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&UI_GraphReFresh_data, LEN_CRC8, 0xFF);

	UI_GraphReFresh_data.CmdID = ID_student_interactive;

	switch (cnt)
	{
	case 1:
		UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw1;
		break;
	case 2:
		UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw2;
		break;
	case 5:
		UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw5;
		break;
	case 7:
		UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw7;
		break;
	}

	UI_GraphReFresh_data.datahead.receiver_ID = _id->Cilent_ID;
	UI_GraphReFresh_data.datahead.sender_ID = _id->Robot_ID;
	memcpy(buffer, (uint8_t *)&UI_GraphReFresh_data, LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head); // ��֡ͷ�������롢��������֡ͷ�����ָ��Ƶ�������

	for (uint8_t i = 0; i < cnt; i++) // ���ͽ������ݵ�����֡��������CRC16У��ֵ
	{
		graphData = va_arg(ap, Graph_Data_t); // ���ʲ����б��е�ÿ����,�ڶ�����������Ҫ���صĲ���������,��ȡֵʱ��Ҫ����ǿ��ת��Ϊָ�����͵ı���
		memcpy(buffer + (LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * i), (uint8_t *)&graphData, UI_Operate_LEN_PerDraw);
	}
	Append_CRC16_Check_Sum(buffer, temp_datalength);
	RefereeSend(buffer, temp_datalength);

	va_end(ap); // �����ɱ�����Ļ�ȡ
}

/************************************************UI�����ַ���ʹ������Ч��*********************************/
void UICharRefresh(referee_id_t *_id, String_Data_t string_Data)
{
	static UI_CharReFresh_t UI_CharReFresh_data;

	uint8_t temp_datalength = Interactive_Data_LEN_Head + UI_Operate_LEN_DrawChar; // ���㽻�����ݳ���

	UI_CharReFresh_data.FrameHeader.SOF = REFEREE_SOF;
	UI_CharReFresh_data.FrameHeader.DataLength = temp_datalength;
	UI_CharReFresh_data.FrameHeader.Seq = UI_Seq;
	UI_CharReFresh_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&UI_CharReFresh_data, LEN_CRC8, 0xFF);

	UI_CharReFresh_data.CmdID = ID_student_interactive;

	UI_CharReFresh_data.datahead.data_cmd_id = UI_Data_ID_DrawChar;

	UI_CharReFresh_data.datahead.receiver_ID = _id->Cilent_ID;
	UI_CharReFresh_data.datahead.sender_ID = _id->Robot_ID;

	UI_CharReFresh_data.String_Data = string_Data;

	UI_CharReFresh_data.frametail = Get_CRC16_Check_Sum((uint8_t *)&UI_CharReFresh_data, LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);

	RefereeSend((uint8_t *)&UI_CharReFresh_data, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // ����

	UI_Seq++; // �����+1
}
