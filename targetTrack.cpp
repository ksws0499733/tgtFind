// targetTrack.cpp : ¶¨Òå¿ØÖÆÌ¨Ó¦ÓÃ³ÌÐòµÄÈë¿Úµã¡£
//


#include "stdafx.h"

#include <opencv2\opencv.hpp>

#include <stdlib.h>  
#include <time.h>  
#include <algorithm>  
#include <math.h>  
#include <iostream>    
#include <vector>  
#include <fstream>  
#include <cstdlib>  
#include <atltime.h>
#include <direct.h>
#include <io.h>

#include <WinSock2.h> 

#include "kcftracker.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "UAV_CTR.h"

using namespace cv;
using namespace std;

#pragma comment(lib,"WS2_32.lib")  
#define BUF_SIZE 64  
//Rect box;//¾ØÐÎ¶ÔÏó
//bool drawing_box;//¼ÇÂ¼ÊÇ·ñÔÚ»­¾ØÐÎ¶ÔÏó
//void onmouse(int event, int x, int y, int flag, void *img);
//int findTraget();
//void Sample();
//Mat falseROI(Mat frame, Rect trueROI);
//void  OnVideo();
//vector<Ellipse> nonMaxSuppress(vector<Ellipse> data);
//Mat thinImage(const cv::Mat & src, const int maxIterations = -1);
//void filterOver(cv::Mat thinSrc);
//vector<cv::Point> getPoints(const cv::Mat &thinSrc, unsigned int raudis = 4, unsigned int thresholdMax = 6, unsigned int thresholdMin = 4);
//vector<Point> LMM(vector<Point> conter, vector<Point> lines);
//vector<Point2f> LMM_fitline(vector<Point2f> conter);
//vector<Point> lineCore(vector<Point> rawP, vector<Point> P3);

#define MID(a,b,c) a > b ? (b > c ? b : ( a > c ? c : a)) : ( a > c ? a: (b > c ? c : a))


DWORD WINAPI CMDSend(LPVOID pPara);	
DWORD WINAPI CMDRecv(LPVOID pPara);	
DWORD WINAPI WaveRecv(LPVOID pPara);
DWORD WINAPI TargerTrack(LPVOID pPara);	
DWORD WINAPI VideoRecv(LPVOID pPara);	
DWORD WINAPI StateChange(LPVOID pPara);
DWORD WINAPI VideoRecv_sock(LPVOID pPara);

UAV_CTR uCtr;
CaCtr *cCtr;
Mat frame;

vector<string>cameraMSG;
vector<string>ctrlerMSG;

static CRITICAL_SECTION cs; // ¶¨ÒåÁÙ½çÇø¶ÔÏó
static CRITICAL_SECTION cs_camera; // ¶¨ÒåÁÙ½çÇø¶ÔÏó
static CRITICAL_SECTION cs_ctrlor; // ¶¨ÒåÁÙ½çÇø¶ÔÏó


CONFIG cfg;

bool isNewFrame;

int cmmd;
short roll, pitch;
unsigned short yaw;
short cm_pitch;
short cm_yaw;
int ldist,wdist;
vector<double>wdists;
DWORD t0;
short spdN, spdE, spdG;
int pposN, pposE, pposD;

void writeMSG(uchar* msg, double ctrX, double ctrY, int len);
bool readMSG(uchar* msg, int len);
bool readConfig(CONFIG* cfg, char* path);
string get_usableName(const char* dirname);

void writeMSG42(uchar* msg, double ctrX, double ctrY, int len=42) {
	msg[0] = 0xEB;
	msg[1] = 0x90;

	static bool aa = 1;

	int state = !(uCtr.m_state == ST_IDLE);
	switch (uCtr.m_state) {

	case ST_IDLE: {
		//---空闲
		state = 0;
	}break;
		//-------------------------------寻找标靶，相机缓慢摆动
	case ST_F_FIND:
	case ST_FIND:
	case ST_FIND_A:
	case ST_FIND_B:
	case ST_F_FIND_A:
	case ST_FIND_D:
	case ST_FIND_C: {
		state = 1;
	}break;

		//-------------- 锁定标靶至相机中心，报告标靶位置和朝向

	case ST_TRACK:
	case ST_TRACK_A: {
		state = 2;
	}break;

		//------------- 锁定相机向下，报告标靶位置（中心点）
	case ST_N_TRACK:
	case ST_N_TRACK_A: {
		state = 3;
	}break;
		//------------- 拉高无人机至2米以上，报告丢失信息
	case ST_N_LOSE: {
	}break;

	default:break;
	}


	int find = uCtr.isfindFar | uCtr.isfindtarget | uCtr.isfindcenter;
	bool stb = uCtr.isStable;
	msg[2] = 0x00| (stb<<3) | (find << 2) | state;


	int pixX = uCtr.diffX;
	int pixY = uCtr.diffY;
	msg[3] = pixX;
	msg[4] = pixX >> 8;
	msg[5] = pixY;
	msg[6] = pixY >> 8;

	int ang = uCtr.tgt.angelT * 180 / CV_PI * 100;
	msg[7] = ang;
	msg[8] = ang >> 8;

	int yawC = 0;//偏航 方位控制
	int pitchC = 0;

	yawC = round(ctrX) * 100;
	pitchC = round(ctrY) * 100;


	msg[9] = pitchC;
	msg[10] = pitchC >> 8;
	msg[11] = yawC;
	msg[12] = yawC >> 8;

	//int TX = uCtr.t_posX *100;
	//int TY = uCtr.t_posY *100;
	//int TZ = uCtr.t_posZ *100;
	//msg[13] = TX;
	//msg[14] = TX >> 8;
	//msg[15] = TY;
	//msg[16] = TY >> 8;
	//msg[17] = TZ;
	//msg[18] = TZ >> 8;

	int TvN = (uCtr.t_spdN) * 100;
	int TvE = (uCtr.t_spdE) * 100;
	int TvD = 0 * 100;

	//TN = 4096;

	msg[13] = TvN;
	msg[14] = TvN >> 8;


	msg[15] = TvE;
	msg[16] = TvE >> 8;


	msg[17] = TvD;
	msg[18] = TvD >> 8;



	int TN = (uCtr.t_posN) * 100;
	int TE = (uCtr.t_posE) * 100;
	int TD = (uCtr.t_posD) * 100;

	//TN = 4096;

	msg[19] = TN;
	msg[20] = TN >> 8;
	msg[21] = TN >> 16;
	msg[22] = TN >> 24;

	msg[23] = TE;
	msg[24] = TE >> 8;
	msg[25] = TE >> 16;
	msg[26] = TE >> 24;

	msg[27] = TD;
	msg[28] = TD >> 8;
	msg[29] = TD >> 16;
	msg[30] = TD >> 24;

	int angNE = (uCtr.t_head)* 100; // target head N = 0 0~360
	//if (angNE < 0)angNE += 36000;
	msg[31] = angNE;
	msg[32] = angNE >> 8;


	unsigned int pc_ag = uCtr.t_height * 100;
	//pc_ag = 1234;
	int pc_vx = 0;
	int pc_vy = 0;
	int pc_vz = 0;
	msg[33] = pc_ag;
	msg[34] = pc_ag >> 8;
	msg[35] = pc_vx;
	msg[36] = pc_vx >> 8;
	msg[37] = pc_vy;
	msg[38] = pc_vy >> 8;
	msg[39] = pc_vz;
	msg[40] = pc_vz >> 8;

	int s = 0;
	for (int ii = 0; ii < 41; ii++) {
		s += (msg[ii] & 0xff);
	}
	msg[41] = s % 256;

}

bool readMSG53(uchar* msg, int len=53) {

	if (msg[0] == 0xEB && msg[1] == 0x90) {
	}
	else {
		return false;
	}

	//---- 校验和
	int s = 0;
	for (int ii = 0; ii < 52; ii++) {
		s += (msg[ii] & 0xff);
	}
	s %= 256;
	if (s != msg[52]) {
		return false;
	}


	cmmd = msg[2];


	roll = (msg[3] & 0xff) | ((msg[4]) << 8);//0.01°
	pitch = (msg[5] & 0xff) | ((msg[6]) << 8);//0.01°
	yaw = (msg[7] & 0xff) | ((msg[8] & 0xff) << 8);//0.01°


	cm_pitch = (msg[9] & 0xff) | (msg[10] << 8);//0.01°
	cm_yaw = (msg[11] & 0xff) | (msg[12] << 8);//0.01°


	ldist = (msg[13] | (msg[14] << 8)) & 0xffff;//cm
	if (ldist == 0xffff) {
		ldist = -1;
	}

	spdN = (msg[15] & 0xff) | (msg[16] << 8);
	spdE = (msg[17] & 0xff) | (msg[18] << 8);
	spdG = (msg[19] & 0xff) | (msg[20] << 8);

	pposN = (msg[21] & 0xff) | ((msg[22] & 0xff) << 8) | ((msg[23] & 0xff) << 16) | (msg[24] << 24);
	pposE = (msg[25] & 0xff) | ((msg[26] & 0xff) << 8) | ((msg[27] & 0xff) << 16) | (msg[28] << 24);
	pposD = (msg[29] & 0xff) | ((msg[30] & 0xff) << 8) | ((msg[31] & 0xff) << 16) | (msg[32] << 24);

	bool tdata_va;
	tdata_va = msg[33];

	int tspdN, tspdE, tspdD;
	tspdN = (msg[34] & 0xff) | ((msg[35]) << 8);//0.01 m/s
	tspdE = (msg[36] & 0xff) | ((msg[37]) << 8);//0.01 m/s
	tspdD = (msg[38] & 0xff) | ((msg[39]) << 8);//0.01 m/s

	int tposN, tposE, tposD;
	tposN = (msg[40] & 0xff) | ((msg[41] & 0xff) << 8) | ((msg[42] & 0xff) << 16) | (msg[43] << 24);
	tposE = (msg[44] & 0xff) | ((msg[45] & 0xff) << 8) | ((msg[46] & 0xff) << 16) | (msg[47] << 24);
	tposD = (msg[48] & 0xff) | ((msg[49] & 0xff) << 8) | ((msg[50] & 0xff) << 16) | (msg[51] << 24);


	return true;
}

bool readWave(uchar* msg, int len = 4) {

	if (msg[0] == 0xFF) {
	}
	else {
		return false;
	}

	//---- 校验和
	int s = 0;
	for (int ii = 0; ii < 3; ii++) {
		s += (msg[ii] & 0xff);
	}
	s %= 256;
	if (s != msg[3]) {
		return false;
	}
	wdist = (msg[2] & 0xff) | ((msg[1]) << 8);
	return true;
}

void writeMSG(uchar* msg, double ctrX, double ctrY, int len) {
	msg[0] = 0xEB;
	msg[1] = 0x90;

	static bool aa = 1;

	int state = !(uCtr.m_state == ST_IDLE);
	switch (uCtr.m_state) {

	case ST_IDLE: {
		//---空闲
		state = 0;
	}break;
		//-------------------------------寻找标靶，相机缓慢摆动
	case ST_F_FIND:
	case ST_FIND:
	case ST_FIND_A:
	case ST_FIND_B:
	case ST_F_FIND_A:
	case ST_FIND_D:
	case ST_FIND_C: {
		state = 1;
	}break;

		//-------------- 锁定标靶至相机中心，报告标靶位置和朝向

	case ST_TRACK:
	case ST_TRACK_A: {
		state = 2;
	}break;

		//------------- 锁定相机向下，报告标靶位置（中心点）
	case ST_N_TRACK:
	case ST_N_TRACK_A: {
		state = 3;
	}break;
		//------------- 拉高无人机至2米以上，报告丢失信息
	case ST_N_LOSE: {
	}break;

	default:break;
	}


	int find = uCtr.isfindFar | uCtr.isfindtarget | uCtr.isfindcenter;
	//if (uCtr.p_height < 1) find = 0;

	msg[2] = 0x00 | (find << 2) | state;


	int pixX = uCtr.diffX;
	int pixY = uCtr.diffY;
	msg[3] = pixX;
	msg[4] = pixX >> 8;
	msg[5] = pixY;
	msg[6] = pixY >> 8;

	int ang = uCtr.tgt.angelT * 180 / CV_PI * 100;
	msg[7] = ang;
	msg[8] = ang >> 8;

	int yawC = 0;//偏航 方位控制
	int pitchC = 0;

	yawC = round(ctrX) * 100;
	pitchC = round(ctrY) * 100;


	msg[9] = pitchC;
	msg[10] = pitchC >> 8;
	msg[11] = yawC;
	msg[12] = yawC >> 8;

	//int TX = uCtr.t_posX *100;
	//int TY = uCtr.t_posY *100;
	//int TZ = uCtr.t_posZ *100;
	//msg[13] = TX;
	//msg[14] = TX >> 8;
	//msg[15] = TY;
	//msg[16] = TY >> 8;
	//msg[17] = TZ;
	//msg[18] = TZ >> 8;

	int TN = (uCtr.t_posN + uCtr.p_posN0) * 100;
	int TE = (uCtr.t_posE + uCtr.p_posE0) * 100;
	int TD = (uCtr.t_posD) * 100;

	//TN = 4096;

	msg[13] = TN;
	msg[14] = TN >> 8;
	msg[15] = TN >> 16;
	msg[16] = TN >> 24;

	msg[17] = TE;
	msg[18] = TE >> 8;
	msg[19] = TE >> 16;
	msg[20] = TE >> 24;

	msg[21] = TD;
	msg[22] = TD >> 8;
	msg[23] = TD >> 16;
	msg[24] = TD >> 24;

	int pc_ag = 0;
	int pc_vx = 0;
	int pc_vy = 0;
	int pc_vz = 0;
	msg[25] = pc_ag;
	msg[26] = pc_ag >> 8;
	msg[27] = pc_vx;
	msg[28] = pc_vx >> 8;
	msg[29] = pc_vy;
	msg[30] = pc_vy >> 8;
	msg[31] = pc_vz;
	msg[32] = pc_vz >> 8;

	int s = 0;
	for (int ii = 0;ii < 33;ii++) {
		s += (msg[ii] & 0xff);
	}
	msg[33] = s % 256;

}

bool readMSG(uchar* msg, int len) {

	if (msg[0] == 0xEB && msg[1] == 0x90) {
	}
	else {
		return false;
	}

	//---- 校验和
	int s = 0;
	for (int ii = 0;ii < 33;ii++) {
		s += (msg[ii] & 0xff);
	}
	s %= 256;
	if (s != msg[33]) {
		return false;
	}
	cmmd = msg[2];


	roll = (msg[3] & 0xff) | ((msg[4]) << 8);//0.01°
	pitch = (msg[5] & 0xff) | ((msg[6]) << 8);//0.01°
	yaw = (msg[7] & 0xff) | ((msg[8] & 0xff) << 8);//0.01°


	cm_pitch = (msg[9] & 0xff) | (msg[10] << 8);//0.01°
	cm_yaw = (msg[11] & 0xff) | (msg[12] << 8);//0.01°


	ldist = (msg[13] | (msg[14] << 8)) & 0xffff;//cm
	if (ldist == 0xffff) {
		ldist = -1;
	}

	spdN = (msg[15] & 0xff) | (msg[16] << 8);
	spdE = (msg[17] & 0xff) | (msg[18] << 8);
	spdG = (msg[19] & 0xff) | (msg[20] << 8);

	pposN = (msg[21] & 0xff) | ((msg[22] & 0xff) << 8) | ((msg[23] & 0xff) << 16) | (msg[24] << 24);
	pposE = (msg[25] & 0xff) | ((msg[26] & 0xff) << 8) | ((msg[27] & 0xff) << 16) | (msg[28] << 24);
	pposD = (msg[29] & 0xff) | ((msg[30] & 0xff) << 8) | ((msg[31] & 0xff) << 16) | (msg[32] << 24);
	return true;
}
SYSTEMTIME pcTime;
//CTime PCtime;

Demo demo;
bool isEnd = false;
bool isReleased = false;

int main(int argc, char* argv[]) {
	//Mat fff;
	//VideoCapture vcap(0);
	//while (1) {
	//	vcap >> fff;
	//	imshow("ddd", fff);
	//	if (waitKey(30) == ' ')
	//		break;
	//}
	//cout << "............." << endl;
	//vcap.release();
	//vcap.open(0);
	//while (1) {
	//	vcap >> fff;
	//	imshow("ddd", fff);
	//	if (waitKey(30) == ' ')
	//		break;
	//}

	//vcap.release();
	//return 0;



	demo.setup();	
	isNewFrame = false;
	InitializeCriticalSection(&cs);
	InitializeCriticalSection(&cs_camera);
	InitializeCriticalSection(&cs_ctrlor);
		
	GetLocalTime(&pcTime);
	
	cfg = CONFIG();
	if (readConfig(&cfg, "D:\\configure.txt")) {

	}
	else {
		readConfig(&cfg, "configure.txt");
	}

	//readConfig(&cfg, "C:\\Users\\LattePanda\\Desktop\\configure.txt");
	
	uCtr.Camera.Open(cfg.ctr_port, cfg.ctr_baud);
	//uCtr.Wave.Open(9,9600);
	uCtr.setTagPara(cfg.tag_position, cfg.tag_len);
	uCtr.init();
	cCtr = &uCtr.cct;
	t0 = GetTickCount();
	if (uCtr.frd == NULL) {
		char fname[108];
		sprintf(fname, "D:\\record\\Record%d.txt", t0);
		uCtr.frd = fopen(fname, "w+");
	}
	uCtr.T0 = t0;

	//CreateThread(NULL, 0, CMDSend, NULL, 0, NULL);  //---- uart send
	//CreateThread(NULL, 0, CMDRecv, NULL, 0, NULL);	//---- uart recv
	//CreateThread(NULL, 0, WaveRecv, NULL, 0, NULL);	//---- uart recv
	CreateThread(NULL, 0, TargerTrack, NULL, 0, NULL); //--- picture recognise & show
	CreateThread(NULL, 0, StateChange, NULL, 0, NULL);	//--- m_state change
	//if (cfg.camera_socket) {
	//	CreateThread(NULL, 0, VideoRecv_sock, NULL, 0, NULL);  //--- picture recv from socket
	//}
	//else {
	CreateThread(NULL, 0, VideoRecv, NULL, 0, NULL);	//--- picture recv from camera
	//}
	
	
	if (cfg.isDebugMode ) {
		Sleep(cfg.startTime);
		uCtr.m_state = cfg.startST;
	}
	int ii = 0;
	int tt = 0;
	
	//Mat tmmp = imread("tmmp.png", 0);
	//Mat rrr = uCtr.RotateMat(tmmp, 10);
	//imshow("rrr", rrr);
	//waitKey(1);
	while (1) {
		if (getchar() == '\n') {
			isEnd = true;
			break;
		}
		else {
			cout << "continue\n";
		}
	}
	cout << "...........releas\n";
	while (!isReleased)
	{
		Sleep(100);
	}

	return 0;
}

//----- °´ÃüÁîÐòÁÐ·¢ËÍÐÅÏ¢
DWORD WINAPI CMDSend(LPVOID pPara) {
	//--init 	
	SYSTEMTIME tp;
	vector<string>MSG;
	FILE * fp,*fpr;
	fp = fopen("SendRecord.txt", "w+");

	uchar msg[1024];
	char fname1[108],fname2[108];
	sprintf(fname1, "D:\\record\\SendRecord%04d%02d%02d_%02d%02d%02d.txt", pcTime.wYear, pcTime.wMonth, pcTime.wDay, pcTime.wHour, pcTime.wMinute, pcTime.wSecond);
	sprintf(fname2, "C:\\Users\\LattePanda\\Desktop\\record\\SendRecord%04d%02d%02d_%02d%02d%02d.txt", pcTime.wYear, pcTime.wMonth, pcTime.wDay, pcTime.wHour, pcTime.wMinute, pcTime.wSecond);
	
	if ((fpr = fopen(fname1, "w+")) == NULL) {
		fpr = fopen(fname2, "w+");
	}
	if (fpr != NULL) {
		fprintf(fpr, "time  find  posX posY posZ   cmctr_yaw cmctr_pitch\n");
	}

	static char cont=0;

	DWORD t1, t2;
	t1 = GetTickCount();
	int num = 0;

	double cYaw, cPitch;

	double dPitch, dYaw, dPitch_last, dYaw_last;
	double dPitch_P, dYaw_P;
	double dPitch_I, dYaw_I;
	double dPitch_D, dYaw_D;
	dPitch_P = dYaw_P = dPitch_I = dYaw_I = dPitch_D = dYaw_D = 0;
	dPitch = dYaw = dPitch_last = dYaw_last = 0;

	int pii = 0;
	bool isfind = false;
	bool isfind_last;
	double Yaw0, Pitch0;

	// loops
	while (1)
	{
		num++;
		t2 = GetTickCount();
		if (t2 - t1 > 1000 && cfg.isDebugMode) {
			//cout << "send fps:= " << num << endl;
			t1 = t2;
			num = 0;
		}

		switch (uCtr.m_state) {

		case ST_IDLE: {
			//---¿ÕÏÐ
			cYaw = 0;
			cPitch = 0;
			isfind = false;
		}break;
			//-------------------------------Ñ°ÕÒ±ê°Ð£¬Ïà»ú»ºÂý°Ú¶¯
		case ST_F_FIND: 			
		case ST_FIND:
		case ST_FIND_A:
		case ST_FIND_B:
		case ST_FIND_C: {

			cYaw = uCtr.cst.yaw;
			cPitch = uCtr.cst.pitch;

			pii += 1;
		}break;

			//-------------- Ëø¶¨±ê°ÐÖÁÏà»úÖÐÐÄ£¬±¨¸æ±ê°ÐÎ»ÖÃºÍ³¯Ïò
		case ST_F_FIND_A:
		case ST_FIND_D: 		
		case ST_TRACK:
		case ST_TRACK_A: {

		}break;

			//------------- Ëø¶¨Ïà»úÏòÏÂ£¬±¨¸æ±ê°ÐÎ»ÖÃ£¨ÖÐÐÄµã£©
		case ST_N_TRACK: 
		case ST_N_TRACK_A: {

		}break;
			//------------- À­¸ßÎÞÈË»úÖÁ2Ã×ÒÔÉÏ£¬±¨¸æ¶ªÊ§ÐÅÏ¢
		case ST_N_LOSE: {
			cYaw = 0;
			cPitch = 0;
		}break;

		default:break;
		}

		writeMSG42(msg, cYaw, cPitch);

		if (uCtr.Camera.IsOpen()) {
			uCtr.Camera.Write(msg, 42);
		}
		
		if (fpr) {
			GetLocalTime(&tp);
			fprintf(fpr, "%02d %02d %02d %03d  %02x ",tp.wHour,tp.wMinute,tp.wSecond, tp.wMilliseconds, msg[2]);
			fprintf(fpr, "%.2lf %.2lf %.2lf   %.2lf %.2lf %.2lf  ",uCtr.t_posX, uCtr.t_posY, uCtr.t_posZ, uCtr.p_yaw, uCtr.p_pitch, uCtr.p_roll);
			fprintf(fpr, " %.2lf %.2lf %.2lf \n",uCtr.p_posN, uCtr.p_posE, uCtr.p_posD);
			fflush(fpr);
		}
		Sleep(80);
	}

	return 0;
}

//------ ½ÓÊÕ·É¿ØÐÅÏ¢£¬¸üÐÂÏà»úÊý¾ÝºÍ¿ØÖÆÃüÁî
DWORD WINAPI CMDRecv(LPVOID pPara) {
	char buffer[1024];
	int len;
	FILE * fp;
	fp = fopen("RecvRecord.txt", "w+");
	FILE * fpr;
	char fname[108];


	sprintf(fname, "D:\\record\\RecvRecord%04d%02d%02d_%02d%02d%02d.txt", pcTime.wYear, pcTime.wMonth, pcTime.wDay, pcTime.wHour, pcTime.wMinute, pcTime.wSecond);
	if ((fpr = fopen(fname, "w+")) == NULL) {
		sprintf(fname, "C:\\Users\\LattePanda\\Desktop\\record\\RecvRecord%04d%02d%02d_%02d%02d%02d.txt", pcTime.wYear, pcTime.wMonth, pcTime.wDay, pcTime.wHour, pcTime.wMinute, pcTime.wSecond);
		fpr = fopen(fname, "w+");
	}
	if (fpr != NULL) {
		fprintf(fpr, "time state  tposN tposE tposD   tposNp tposEp   pposN pposE pposD   dposN dposE   dposNp dposEp   height\n");
	}
	SYSTEMTIME tp;
	int last_cmmd = 0;
	DWORD t1, t2;
	t1 = GetTickCount();
	int num = 0;
	int tt = 0;
	while (1)
	{		
		t2 = GetTickCount();
		if (t2 - t1 > 1000 && cfg.isDebugMode) {
			//cout << "recv fps:= " << num << endl;
			t1 = t2;
			num = 0;
		}
		if(uCtr.Camera.IsOpen())
			len = uCtr.Camera.Read(buffer, 53); //½ÓÊÕbuffer
		else
			uCtr.Camera.Open(cfg.ctr_port, cfg.ctr_baud);

		if (len > 0) {
			if (fp) {
				for (int ii = 0; ii < len; ii++) {
					fprintf(fp, "%02x ", (buffer[ii] & 0xff));
				}
				fprintf(fp, "\n");
				fflush(fp);
			}


			if (readMSG53((uchar*)buffer)) {
				
				//uCtr.updataCamera(-90 / 100.0, 0 / 100.0, 300 / 100.0);	
				uCtr.updataPose(roll / 100.0, pitch / 100.0, yaw / 100.0, pposN / 100.0, pposE / 100.0, pposD / 100.0);
				//uCtr.updataCamera(-90,0, ldist/100.0);
				//if(cfg.isDebugMode)cout << "ldist:= " << ldist << endl;
				if (cmmd == 0x01 && last_cmmd != 0x01) {
					uCtr.m_state = ST_FIND;
				}
				if (cmmd == 0x02 && last_cmmd != 0x02) {
					uCtr.m_state = ST_IDLE;
					uCtr.isfindcenter = false;
					uCtr.isfindFar = false;
					uCtr.isfindtarget = false;
					uCtr.trackBox.width = 0;
					uCtr.rROI = Rect(0, 0, 0, 0);
					//destroyAllWindows();
				}
				last_cmmd = cmmd;
				num++;
				if (fpr) {
					GetLocalTime(&tp);
					fprintf(fpr, "%02d %02d %02d %03d  %02d   ", tp.wHour,tp.wMinute,tp.wSecond,tp.wMilliseconds, uCtr.m_state);
					fprintf(fpr, "%.2lf %.2lf    %.2lf %.2lf   ", uCtr.t_posN, uCtr.t_posE, uCtr.t_posN_p, uCtr.t_posE_p);
					fprintf(fpr, "%.2lf %.2lf %.2lf    %d %d    ", uCtr.p_posN, uCtr.p_posE, uCtr.p_posD, uCtr.diffX, uCtr.diffY);
					fprintf(fpr, "%.2lf %.2lf   ", uCtr.t_posN - uCtr.p_posN, uCtr.t_posE - uCtr.p_posE);
					fprintf(fpr, "%.2lf %.2lf   %.2lf\n", uCtr.t_posN_p - uCtr.p_posN, uCtr.t_posE_p - uCtr.p_posE, uCtr.p_height);
					fflush(fpr);
				}				
			}
		}
		Sleep(10);
	}
	return 0;
}

DWORD WINAPI WaveRecv(LPVOID pPara) {
	char buffer[1024];
	int len;
	
	FILE *ffp = fopen("wave.txt", "w+");
	SYSTEMTIME tp;
	DWORD t1, t2;
	t1 = GetTickCount();
	int num = 0;
	int tt = 0;
	while (1)
	{
		t2 = GetTickCount();
		if (t2 - t1 > 1000 && cfg.isDebugMode) {
			//cout << "recv fps:= " << num << endl;
			t1 = t2;
			num = 0;
		}
		if (uCtr.Wave.IsOpen())
			len = uCtr.Wave.Read(buffer, 4); //½ÓÊÕbuffer
		else
			uCtr.Wave.Open(9,9600);

		if (len > 0) {
			
			if (readWave((uchar*)buffer)) {
				if (ffp) {
					fprintf(ffp, "%.3lf\n", wdist / 1000.0);
					fflush(ffp);
				}
				if (wdist != 0xAAAA) {
					uCtr.updataCamera(-90, 0, wdist / 1000.0);
					wdists.push_back(wdist / 1000.0);
					if (wdists.size() > 300) {
						wdists.erase(wdists.begin());
					}
				}
				

				if (cfg.isDebugMode)cout << "wdist:= " << wdist << endl;
				num++;
				
			}
		}
		Sleep(10);
	}
	return 0;
}

DWORD WINAPI TargerTrack(LPVOID pPara) {
		
	char picName[100];
	int pid = 0;
	//VideoWriter vwrt("record.avi",CV_FOURCC('X','V','I','D'),25.0,Size(1780,1000));
	VideoWriter vwrt;
	while (1) {	
		
		//while (isNewFrame == false) { Sleep(10); }
		EnterCriticalSection(&cs);
		if (!frame.empty()) {
			uCtr.updataFrame(frame);
			isNewFrame = false;
		}
		LeaveCriticalSection(&cs);

		switch (uCtr.m_state) {

		case ST_IDLE: {
			
		}break;
			//-------------------------------Ñ°ÕÒÔ¶¾àÀë±ê°ÐÏà¹Ø
		case ST_F_FIND: 
		case ST_F_FIND_A: {
			uCtr.isfindcenter = false;
			uCtr.isfindtarget = false;
			//uCtr.isfindFar = false;
			uCtr.pPoseRecord();
			uCtr.farFindTarget();
		}break;
			//-------------------------------Ñ°ÕÒ±ê°ÐÏà¹Ø
		case ST_FIND:
		case ST_FIND_A: 
		case ST_FIND_B: 
		case ST_FIND_C: 
		case ST_FIND_D: {
			uCtr.isfindcenter = false;
			//uCtr.isfindtarget = false;
			uCtr.isfindFar = false;
			uCtr.pPoseRecord();
			uCtr.findTarget();
		}break;

			//----------------------------- ¸ú×Ù±ê°ÐÏà¹Ø
		case ST_TRACK: 
		case ST_TRACK_A: {
			uCtr.isfindcenter = false;
			//uCtr.isfindtarget = false;
			uCtr.isfindFar = false;
			if (cfg.trackMode) {
				uCtr.pPoseRecord();
				uCtr.trackTarget(Mat());
			}
			else {
				uCtr.pPoseRecord();
				uCtr.findTarget();
			}


		}break;

		case ST_N_TRACK: 
		case ST_N_TRACK_A: {
			//uCtr.isfindcenter = false;
			uCtr.isfindtarget = false;
			uCtr.isfindFar = false;
			if (uCtr.rframe.empty()) {
				continue;
			}
			Mat image;
			resize(uCtr.rframe, image, Size(640,360));
			Mat image_gray;
			cv::cvtColor(image, image_gray, CV_BGR2GRAY);
			uCtr.pPoseRecord();

			uCtr.detections = demo.m_tagDetector->extractTags(image_gray);
			if (uCtr.detections.size()) {			
				uCtr.isfindcenter = uCtr.targetPoseT();
			}
			else {
				uCtr.isfindcenter = false;	
				uCtr.nofindtimes++;
				uCtr.findtimes = 0;
				//uCtr.pPoseRecord();
				//uCtr.findTarget();
			}
		}break;
		case ST_N_LOSE: {
			uCtr.isfindcenter = false;
			uCtr.isfindtarget = false;
			uCtr.isfindFar = false;
			
		}break;
		default:break;				
		}

		uCtr.show();
		static char name[100];
		if (uCtr.m_state != ST_N_LOSE && uCtr.m_state != ST_IDLE) {	
			if (!vwrt.isOpened()) {
				string videoname = get_usableName(cfg.videoPath.data());
				vwrt.open(videoname, CV_FOURCC('X', 'V', 'I', 'D'), 25.0, Size(1780, 1000));
			}
			if (vwrt.isOpened() && !uCtr.UI.empty()){
				vwrt << uCtr.UI;
			}				
		}
		else {
			if(vwrt.isOpened())
				vwrt.release();			
		}
/*		if (uCtr.m_state >= ST_N_TRACK && uCtr.m_state <= ST_N_TRACK_A && cfg.isSavePic) {
			GetLocalTime(&pcTime);			
			sprintf(picName, "%02d_%02d_%02d_%03d.jpg", pcTime.wHour, pcTime.wMinute, pcTime.wSecond, pcTime.wMilliseconds);
			imwrite(picName, uCtr.sframe);
		}*/		
	}

	return 0;
}

DWORD WINAPI StateChange(LPVOID pPara) {
	
	int fnum = 0;

	bool isNear = false;
	double sPitch, sYaw;
	int ts = 0;
	DWORD ts1, ts2;
	DWORD t1, t2;
	uchar SMSG[100];
	char picName[100];

	t1 = GetTickCount();
	fnum = 0;
	while (1) {
		switch (uCtr.m_state) {

		case ST_IDLE: {

		}break;

		case ST_F_FIND: {			
			if (uCtr.isNearFind()) {
				uCtr.m_state = ST_FIND;
			}			
		}break;

		case ST_F_FIND_A: {

			uCtr.m_state = ST_F_FIND;
		}break;

		case ST_FIND: {
			if (uCtr.findtimes > 2) {
				uCtr.m_state = ST_TRACK;
				cout << "Find to Track Mode\n";
			}
			else if (( uCtr.t_height<2.4 | uCtr.targetBox.area() > 300 * 300 ) && cfg.isNearModeUsed) {
				uCtr.targetBox.width = 0;
				uCtr.m_state = ST_N_TRACK;
				cout << "Find to Near Mode\n";
			}
			else if (uCtr.nofindtimes > 5 && uCtr.p_height > 8 && cfg.isFarModeUsed) { 
				uCtr.m_state = ST_F_FIND;
			}
		}break;
			

		case ST_FIND_D: {

			uCtr.m_state = ST_FIND;

		}break;
			//----------------------------- ¸ú×Ù±ê°ÐÏà¹Ø
		case ST_TRACK: {

			if (uCtr.nofindtimes > 5) {
				uCtr.m_state = ST_FIND;
			}
			if ((uCtr.t_height<2.4 | uCtr.targetBox.area() > 300*300) && cfg.isNearModeUsed) {
				uCtr.targetBox.height = 0;
				uCtr.m_state = ST_N_TRACK;
				cout << "Track to Near Mode\n";
			}
		}break;

		case ST_N_TRACK: 
		case ST_N_TRACK_A: {

			static int nnn = 0;

			if ( uCtr.nofindtimes > 10) {
				uCtr.t_height = 2.5;
				uCtr.m_state = ST_FIND;
				cout << "Near to FIND Mode\n";
			}

			if (uCtr.t_height < 0.85) {
				nnn ++;
			}
			else {
				nnn = 0;
			}
			if (nnn > 30 && cfg.isEnterEnd) {
				uCtr.m_state = ST_N_LOSE;
			}
			if (uCtr.isStable && uCtr.t_height < 1) {
				uCtr.m_state = ST_N_LOSE;
			}


		}break;
		case ST_N_LOSE: {

			//if (uCtr.p_height >3) {
			//	uCtr.m_state = ST_FIND;
			//}
			if (uCtr.p_height >3) {
				uCtr.m_state = ST_IDLE;
			}
		}break;

		default:break;
		}

		Sleep(10);
	}

	return 0;
}

//------ video 
DWORD WINAPI VideoRecv_sock(LPVOID pPara) {
	WSAData wsd;           //初始化信息  
	SOCKET soRecv;              //接收SOCKET  
	char * buffer = NULL; //接收数据的数据缓冲区指针  
	int nRet = 0;
	int i = 0;
	int dwSendSize = 0;
	SOCKADDR_IN siRemote, siLocal;    //远程发送机地址和本机接收机地址  

									  //启动Winsock  
	if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) {
		cout << "WSAStartup Error = " << WSAGetLastError() << endl;
		return 0;
	}

	//创建socket  
	soRecv = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (soRecv == SOCKET_ERROR) {
		cout << "socket Error = " << WSAGetLastError() << endl;
		return 1;
	}

	//设置端口号  
	int nPort = 20102;

	//int nPort = 1234;  
	siLocal.sin_family = AF_INET;
	siLocal.sin_port = htons(cfg.local_port);
	siLocal.sin_addr.s_addr = inet_addr(cfg.local_ip.c_str());

	//绑定本地地址到socket  
	nRet = ::bind(soRecv, (SOCKADDR*)&siLocal, sizeof(siLocal));
	if (nRet == SOCKET_ERROR) {
		cout << "bind Error = " << WSAGetLastError() << endl;
		return 1;
	}

	//申请内存  
	buffer = new char[65535];
	if (buffer == NULL) {
		cout << "pszRecv new char Error " << endl;
		return 0;
	}

	int t1, t2;
	t1 = GetTickCount();
	int llen = 0;
	std::vector<uchar> data;
	bool init = false;

	double Scan = cfg.camera_scan;
	int wid = cfg.camera_wid;
	int hei = cfg.camera_hei;

	//int wid = 1280;
	//int hei = 720;
	//double Scan = 88.2;
	uCtr.cst.setPara(wid, hei, Scan / 180 * CV_PI, Scan / wid*hei / 180 * CV_PI);
	while (true) {
		dwSendSize = sizeof(siRemote);
		//开始接受数据  
		nRet = recv(soRecv, buffer, 65535, 0);
		llen += nRet;
		t2 = GetTickCount();
		if (t2 - t1 > 1000) {
			cout << llen << endl;
			llen = 0;
			t1 = t2;
		}


		if (nRet == SOCKET_ERROR) {
			cout << "recvfrom Error " << WSAGetLastError() << endl;
			break;
		}
		else if (nRet == 0) {
			break;
		}
		else {
			for (int ii = 4;ii < nRet;ii++) {
				data.push_back(buffer[ii]);
			}

			if (buffer[0] == 0 && buffer[1] == 0 && buffer[2] == 0 && buffer[3] == 0 ) {
			
					Mat ff = imdecode(data, CV_LOAD_IMAGE_COLOR);
					data.clear();
					if (!ff.empty()) {
					/*	Mat ffg;
						flip(ff.t(), ffg, 1);*/

						EnterCriticalSection(&cs);
						//flip(ffg, frame, 0);
						ff.copyTo(frame);
						isNewFrame = true;
						LeaveCriticalSection(&cs);


						/*imshow("pic", ffg);
						waitKey(10);*/
					}					
			}

		}
		Sleep(10);
	}
	//关闭socket连接  
	closesocket(soRecv);
	delete[] buffer;

	//清理  
	WSACleanup();
	//system("pause");
	return 0;

}


DWORD WINAPI VideoRecv(LPVOID pPara) {
	
	VideoCapture vcap;
	
	int cnm = cfg.camera_Number;
	double Scan = 88.2;//cfg.camera_scan;
	int wid = 1280;//cfg.camera_wid;
	int hei = 720;//cfg.camera_hei;


	VideoCapture vcap2;
	int cnm2 = cfg.camera_Number2;
	double Scan2 = cfg.camera_scan2;
	int wid2 = cfg.camera_wid2;
	int hei2 = cfg.camera_hei2;

	
	vcap.open(cnm);
	//vcap.release();
	//vcap.open(cnm);

	vcap.set(CV_CAP_PROP_FRAME_WIDTH, wid);
	vcap.set(CV_CAP_PROP_FRAME_HEIGHT, hei);
	uCtr.cst.setPara(wid, hei, Scan / 180 * CV_PI, Scan / wid*hei / 180 * CV_PI);
	cout << "camera para:= " << wid << "x" << hei << "  " << Scan << endl;
	
	Mat ff,ffg;
	Mat ff2, ffg2;
	//int no_frame_num;

	int camern = 0;
	bool cg00 = 1;
	bool cg01 = 1;
	bool cg02 = 1;
	bool cg10 = 1;
	bool cg11 = 1;
	bool cg12 = 1;
	DWORD t1, t2;
	t1 = GetTickCount();


	while (!isEnd) {
		if (vcap.isOpened()) {

			t2 = GetTickCount();
			if (t2 - t1 > 1000 && cfg.isDebugMode ) {
				// cout << "fps := " << camern << endl;
				// cout << "wid:= "<< wid << "   hei:= " << hei <<"scan:= "<< cameraScan<< endl;
				camern = 0;
				t1 = t2;
			}

			vcap >> ff;		// 1080 far
			ffg = ff;

			EnterCriticalSection(&cs);
				//flip(ffg, frame, 0);
				ffg.copyTo(frame);
				isNewFrame = true;
				camern++;
			LeaveCriticalSection(&cs);
			ff.release();

		/*	cv::imshow("ff",ffg);
			waitKey(10);*/
		}
		else {
			vcap.release();
			vcap.open(cnm);
			vcap.set(CV_CAP_PROP_FRAME_WIDTH, wid);
			vcap.set(CV_CAP_PROP_FRAME_HEIGHT, hei);
			uCtr.cst.setPara(wid, hei, Scan / 180 * CV_PI, Scan / wid*hei / 180 * CV_PI);
		}
		Sleep(10);
	}
	vcap.release();
	isReleased = true;
	return 0;
}

void showConfig(CONFIG* cfg) {
	cout << endl;
	cout << "debug mode := " << cfg->isDebugMode << endl;
	cout << "start status := " << cfg->startST << endl;
	cout << "start time := " << cfg->startTime << endl;
	cout << "uart port := " << cfg->ctr_port << endl;
	cout << "uart baud := " << cfg->ctr_baud << endl;
	cout << "tag number := " << cfg->tag_len << endl;
	cout << endl<<endl;
	
}

void readcfg(const string s, const char* para, int len, bool* cfg) {
	const char* sc = s.data();
	if (s.compare(0, len, para, 0, len) == 0) {
		const char* ss = sc + len + 3;
		if (ss[0] == '1')
			*cfg = true;
		else
			*cfg = false;
	}
}
void readcfg(const string s, const char* para, int len, int* cfg, int min = 0, int max = MAXINT) {
	const char* sc = s.data();
	if (s.compare(0, len, para, 0, len) == 0) {
		const char* ss = sc + len + 3;
		sscanf(ss, "%d", cfg);
	}
	*cfg = MID(*cfg, min, max);
}
void readcfg(const string s, const char* para, int len, double* cfg) {
	const char* sc = s.data();
	if (s.compare(0, len, para, 0, len) == 0) {
		const char* ss = sc + len + 3;
		sscanf(ss, "%lf", cfg);
	}
}
void readcfg(const string s, const char* para, int len, string &cfg) {
	const char* sc = s.data();
	if (s.compare(0, len, para, 0, len) == 0) {
		const char* ss = sc + len + 3;
		cfg = ss;
	}
}

bool readConfig(CONFIG* cfg, char* path)
{
	string s;
	ifstream fp(path);
	//char sc[64];
	const char* sc;
	const char *ss = NULL;
	
	if (!fp)
	{
		cout << "open congfig file:= "<< path<<" failed"<<endl;
		cfg->ready = false;
		return false;
	}
	else {
		cout << "open congfig file := " << path <<" success" << endl;
	}

	int idx = 0;
	while (getline(fp, s))
	{
		//cout << s << endl;
		sc = s.data();
		//--------  debug set

		readcfg(s, "DebugMode", 9,&(cfg->isDebugMode));
		readcfg(s, "start_ST", 8, &(cfg->startST));
		readcfg(s, "start_time", 10, &(cfg->startST),2000);
		
		
		//-------------- uart set

		readcfg(s, "PLAN_PORT", 9, &(cfg->ctr_port));
		readcfg(s, "PLAN_BAUD", 9, &(cfg->ctr_baud));
		if (cfg->ctr_baud < 1)
			cfg->ctr_baud = 115200;

		readcfg(s, "SendMesg", 8, &(cfg->isSendMsg));


		//-------camera set
		readcfg(s, "UseTwoCamera", 12, &(cfg->isUseTwoCamera));

				//-------camera 1
		readcfg(s, "CameraNo1", 9, &(cfg->camera_Number));
		readcfg(s, "CameraWid1", 10, &(cfg->camera_wid));
		readcfg(s, "CameraHei1", 10, &(cfg->camera_hei));
		readcfg(s, "CameraScan1", 11, &(cfg->camera_scan));
		readcfg(s, "CameraMV1", 9, &(cfg->camera_movable));

		if (s.compare(0, 10, "CameraPos ", 0, 10) == 0) {
			ss = sc + 12;
			sscanf(ss, "[%f,%f]", &(cfg->camera_position.x), &(cfg->camera_position.y));
		}
		if (s.compare(0, 11, "CameraPose ", 0, 11) == 0) {
			ss = sc + 13;
			sscanf(ss, "[%f,%f,%f]", &(cfg->camera_pose[0]), &(cfg->camera_pose[1]), &(cfg->camera_pose[2]));
		}


				//-------camera 2
		readcfg(s, "CameraNo2", 9, &(cfg->camera_Number2));
		readcfg(s, "CameraWid2", 10, &(cfg->camera_wid2));
		readcfg(s, "CameraHei2", 10, &(cfg->camera_hei2));
		readcfg(s, "CameraScan2", 11, &(cfg->camera_scan2));
		readcfg(s, "CameraMV2", 9, &(cfg->camera_movable2));
		
		if (s.compare(0, 11, "CameraPos2 ", 0, 11) == 0) {
			ss = sc + 13;
			sscanf(ss, "[%f,%f]", &(cfg->camera_position2.x), &(cfg->camera_position2.y));
		}
		if (s.compare(0, 12, "CameraPose2 ", 0, 12) == 0) {
			ss = sc + 14;
			sscanf(ss, "[%f,%f,%f]", &(cfg->camera_pose2[0]), &(cfg->camera_pose2[1]), &(cfg->camera_pose2[2]));
		}

		// socket camera
		readcfg(s, "CameraScoket", 12, &(cfg->camera_socket));
		readcfg(s, "ScoketIP", 8, (cfg->local_ip));

		readcfg(s, "ScoketPort", 10, &(cfg->local_port));

		//------- picture set
		readcfg(s, "InpaintPic", 10, &(cfg->isInpaintPic));
		readcfg(s, "SavePic", 7, &(cfg->isSavePic));

		readcfg(s, "ViedoRecord", 11, &(cfg->isSaveVideo));
		readcfg(s, "RecordPath", 10, (cfg->videoPath));

		//------------ status set
		readcfg(s, "Use_FarMode", 11, &(cfg->isFarModeUsed));
		readcfg(s, "Use_NrMode", 10, &(cfg->isNearModeUsed));
		readcfg(s, "trackMode", 9, &(cfg->trackMode));
		readcfg(s, "EndStop", 7, &(cfg->isEnterEnd));		

		//------ tag para
		readcfg(s, "Use_tag", 7, &(cfg->tag_len));

		if (s.compare(0, 1, "[", 0, 1) == 0) {
			double x, y;
			int id;
			ss = sc;
			sscanf(ss, "[%d: %lf,%lf]", &id,&x,&y);
			if(id<cfg->tag_position.size() && id>=0)
				cfg->tag_position[id] = Vec2f(x, y);
		}
		
	}
	cfg->ready = true;
	fp.close();
	cout << "read congfig file success!!" << endl;
	showConfig(cfg);
	return true;
}

string get_usableName(const char* dirname) {
	//char dirname[100] = "video";
	if (_access(dirname, 0) == -1) {
		_mkdir(dirname);
	}
	char filename[100];
	static int num = 0;
	do {
		sprintf(filename, "%s\\video%d.avi",dirname,num);
		if (_access(filename, 0) == -1) {
			break;
		}
		else {
			num++;
		}
	} while (1);

	return filename;
}
