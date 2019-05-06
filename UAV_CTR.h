#pragma once
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

#include "kcftracker.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include "cncomm.h"

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"


using namespace cv;
using namespace std;

Rect operator+(Rect tgt, Rect roi);
Rect operator*(Rect tgt, double n);
Point2f operator/ (Point2f p1, double div);
Size2i operator*(Size2i sz, double nn);
Point2f operator*(Point p1, Point2f ap);
void rotateMatR(double yaw, double pitch, double roll, Mat& output, int tpye = 0);

enum {
	ST_IDLE,
	ST_F_FIND,
	ST_F_FIND_A,
	ST_FIND,
	ST_FIND_A,
	ST_FIND_B,
	ST_FIND_C,
	ST_FIND_D,
	ST_TRACK,
	ST_TRACK_A,
	ST_N_TRACK,
	ST_N_TRACK_A,
	ST_N_LOSE
};

typedef struct configData {
	configData() {
		ready = false;
		isDebugMode = true;
		startST = ST_FIND;
		startTime = 2000;
		DebugPara = 0xff;

		ctr_port = 6;
		ctr_baud = 115200;
		isSendMsg = true;

		isUseTwoCamera = false;
		camera_Number = 0;
		camera_wid = 1280;
		camera_hei = 720;
		camera_scan = 42.0;
		camera_position = Point2f(0, 0.12);
		camera_movable = false;
		camera_pose = Vec3f(0, -90, 0);

		camera_Number2 = 1;
		camera_wid2 = 1280;
		camera_hei2 = 720;
		camera_scan2 = 42.0;
		camera_position2 = Point2f(0, 0);
		camera_movable2 = false;
		camera_pose2 = Vec3f(0, -90, 0);

		trackMode = 0;
		isNearModeUsed = false;
		isFarModeUsed = true;
		isEnterEnd = false;

		isSaveVideo = true;
		videoPath = "";

		isInpaintPic = true;
		isSavePic = true;
		tag_position = vector<Vec2f>(30);

		camera_socket = false;
		local_ip = "192.168.1.2";
		local_port = 20102;

	}
	bool ready;

	bool isDebugMode;
	int startST;
	int startTime;
	uint32_t DebugPara;

	int ctr_port;
	int ctr_baud;

	bool isUseTwoCamera;
	int camera_Number;
	int camera_wid;
	int camera_hei;
	double camera_scan;
	Point2f camera_position;
	bool camera_movable;
	Vec3f camera_pose;
	int camera_delay;

	int camera_Number2;
	int camera_wid2;
	int camera_hei2;
	double camera_scan2;
	Point2f camera_position2;
	bool camera_movable2;
	Vec3f camera_pose2;
	int camera_delay2;

	int trackMode;

	bool isInpaintPic;
	bool isSendMsg;
	bool isNearModeUsed;
	bool isFarModeUsed;
	bool isSavePic;
	bool isEnterEnd;
	bool isSaveVideo;

	string videoPath;

	vector<Vec2f> tag_position;
	int tag_len;

	bool camera_socket;
	string local_ip;
	int local_port;

}CONFIG;
const int DB_SHOW_TIME = 0x01;
const int DB_SHOW_FRAME = 0x02;

class Demo {
public:
	AprilTags::TagDetector* m_tagDetector;
	AprilTags::TagCodes m_tagCodes;

	bool m_draw; // draw image and April tag detections?
				 //  bool m_arduino; // send tag detections to serial port?
	bool m_timing; // print timing information for each tag extraction call

	int m_width; // image size in pixels
	int m_height;
	double m_tagSize; // April tag side length in meters of square black frame
	double m_fx; // camera focal length in pixels
	double m_fy;
	double m_px; // camera principal point
	double m_py;

	int m_deviceId; // camera id (in case of multiple cameras) 
	list<string> m_imgNames;
	cv::VideoCapture m_cap;

	int m_exposure;
	int m_gain;
	int m_brightness;

public:
	// default constructor
	Demo() :
		// default settings, most can be modified through command line options (see below)
		m_tagDetector(NULL),
		m_tagCodes(AprilTags::tagCodes16h5),
		m_draw(true),
		m_timing(false),
		m_width(640),
		m_height(480),
		m_tagSize(0.074 * 76 * 1000),
		m_fx(600),
		m_fy(600),
		m_px(m_width / 2),
		m_py(m_height / 2),

		m_exposure(-1),
		m_gain(-1),
		m_brightness(-1),

		m_deviceId(0)
	{}



	void setup() {
		m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
	}
	void setupVideo() {
		// find and open a USB camera (built in laptop camera, web cam etc)
		m_cap = cv::VideoCapture(m_deviceId);
		if (!m_cap.isOpened()) {
			cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
			exit(1);
		}
		m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
		m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
		cout << "Camera successfully opened (ignore error messages above...)" << endl;
		cout << "Actual resolution: "
			<< m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
			<< m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

	}

	void print_detection(AprilTags::TagDetection& detection) const {
		cout << "  Id: " << detection.id
			<< " (Hamming: " << detection.hammingDistance << ")\n";

		// recovering the relative pose of a tag:
		// NOTE: for this to be accurate, it is necessary to use the
		// actual camera parameters here as well as the actual tag size
		// (m_fx, m_fy, m_px, m_py, m_tagSize)

		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;
		detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py, translation, rotation);
		std::cout << "MT:= " << translation << std::endl;
	}

	void processImage(cv::Mat& image, cv::Mat& image_gray) {

		cv::cvtColor(image, image_gray, CV_BGR2GRAY);
		vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
		cout << detections.size() << endl;
	}

	// Load and process a single image
	void loadImages() {
		cv::Mat image;
		cv::Mat image_gray;

		for (list<string>::iterator it = m_imgNames.begin(); it != m_imgNames.end(); it++) {
			image = cv::imread(*it); // load image with opencv
			processImage(image, image_gray);
			while (cv::waitKey(100) == -1) {}
		}
	}

	// Video or image processing?
	bool isVideo() {
		return m_imgNames.empty();
	}

	// The processing loop where images are retrieved, tags detected,
	// and information about detections generated
	void loop() {
		cv::Mat image;
		cv::Mat image_gray;

		int frame = 0;
		while (true) {
			// capture frame
			m_cap >> image;

			processImage(image, image_gray);
			// print out the frame rate at which image frames are being processed
			frame++;
			// exit if any key is pressed
			if (cv::waitKey(1) >= 0) break;
		}
	}
}; // Demo



typedef class cameraState {
public:
	cameraState();
	~cameraState();

public:
	double angHz;	//
	double angVt;	//
	double yaw;		//Ë®Æ½½Ç¶È£¨·½Î»½Ç£©------ÄæÊ±ÕëÕý¡¾×ó±ß¡¿ µ¥Î» ¡ã
	double pitch;	//´¹Ö±½Ç¶È£¨¸©Ñö½Ç£©------ÄæÊ±ÕëÕý¡¾ÉÏ±ß¡¿ µ¥Î» ¡ã
	double LDist;   //¼¤¹âÀ×´ï²â¾à m

	//------- Ïà»ú²ÎÊý
	double fx, fy;
	double cx, cy;
	double pixHz, pixVt;  //ÏñËØ¿í¶È
	double aPxVt, aPxHz;  //ÊÓ½Ç·¶Î§ »¡¶È
	double corHz, corVt;  //ÏñËØ-½Ç¶È ×ª»»ÏµÊý

	//------- Ïà¹Øº¯Êý
	void setPara(double ph, double pv, double ah, double av);
	void updata(double aHz, double aVt);
	void updataD(double ya, double pi);

}CaSt;


typedef class cameraControl {
public:
	cameraControl();
	~cameraControl();

	enum cameraTpye{
		VESIBLE_LIGHT,
		RED_WHITE,
		RED_BLACK
	};
	enum PanState {
		PS_IDLE,
		PS_MANU_TRACK,
		PS_AUTO_TRACK,
		PS_LOCK,
		PS_DIGITAL_GUIDE,
		PS_SCAN,
		PS_PROTECT,
		PS_INIT
	};

	enum TrackerState {
		TS_IDLE,
		TS_NORMAL,
		TS_ABNORMAL,
		TS_LOST,
		TS_KCF,
		TS_FOCUS_BLACK,
		TS_FOCUS_WHITE
	};

public:
	
	CaSt camera;
	CnComm cn;
	int com_port;
	int com_baud;
	bool open(int port, int baud = 115200);
	uchar SEND_MSG[110];
	uchar RECV_MSG[50];

	int ctrType;//¿ª¹ØÖ¸Áî
	double focalLength; // ½¹¾à mm
	unsigned int timeMark;//Ê±Âë
	double distant;//¼¤¹âÀ×´ï¾àÀë

	int cameraTpye;// ÔØºÉÀàÐÍ£¬ 0=¿É¼û¹â¡¢1=ºìÍâ°×ÈÈ¡¢2=ºìÍâºÚÈÈ
	int compressMode; //01

	bool redState;
	bool visibleState;

	int PanState;//Æ½Ì¨×´Ì¬
	int TrackerState;//¸ú×ÙÆ÷×´Ì¬
	int fieldSize; //ÊÓ³¡´óÐ¡£¬1/2/4µµ±ä±¶
	
	bool recvRead(uchar* data);
	bool moveCamera(double yaw, double pitch);
	void moveCamera(uchar* msg, double yaw, double pitch);
	bool cmd_scan();
	void cmd_scan(uchar* msg);
	bool stopMove();
	void stopMove(uchar* msg);
	void moveCenter(uchar* msg, double dx, double dy);

	bool stateCheck(double yaw, double pitch);


}CaCtr;


class TargetInfo {
public:
	TargetInfo();
	~TargetInfo();

public:
	double posX;
	double posY;
	double posZ;

	double dist;	//¾àÀë
	double angHz;	//Ë®Æ½½Ç¶È
	double angVt;	//´¹Ö±½Ç¶È

	double angelT; //±ê°Ð³¯Ïò¡¾±ê°ÐÕýÏòÓëÎÞÈË»úÕýÏòµÄ¼Ð½Ç£¬-180 µ½ 180£¬ ÄæÊ±ÕëÕý£¬Ë³Ê±Õë¸º¡¿

	void updata(CaSt cs, Rect ct,vector<Point> lineT);
	void TargetInfo::updata(CaSt cs, Rect ct, double T);
	friend ostream & operator<<(ostream &out, TargetInfo &obj);


};

class TargetRecord {
public:
	TargetRecord();
	~TargetRecord();

public:
	TargetInfo target;
	CaSt camera;
	Mat frame;
	DWORD tim0;

};


class UAV_CTR
{
public:
	UAV_CTR();
	~UAV_CTR();

public:
	Mat rframe; //Ô­Í¼£¬1080p
	Size rsize;
	Mat frame;  
	Size fsize;
	Mat tframe;
	Mat sframe, mMap;
	Mat UI;
	double scale;
	bool isUpdata;
	double scale_tk;
	int fps;
	int T0;

	FILE* frd;

	Mat gray; //frame 的灰度图
	Mat gray2; //gray 的自适应滤波图

	Rect rROI; //算法处理的感兴趣区域
	Rect tROI;

	vector<int> time0;
	vector<double> p_yaws, p_pitchs, p_rolls;	// UAV pose
	vector<Vec3f> t_posNEDs;		// target position (NED)
	Point2f t_pos_bias;
	vector<Vec3f> t_pos_to_UAV;		// target position (correct to UAV)
	

	int diffX, diffY;
	double p_roll, p_yaw, p_pitch;
	double p_posN, p_posE, p_posD;
	double p_roll2, p_yaw2, p_pitch2;
	double p_posN2, p_posE2, p_posD2;
	double p_posN0, p_posE0, p_posD0;
	vector<Point2f> p_posNEs;
	double p_height;// 离地高度
	double t_height;

	double c_pointX, c_pointY, c_pointZ;
	double c_yaw, c_pitch;	//相机光轴方向，相对于水平坐标的方位角和俯仰角。
	vector<double> c_yaws, c_pitchs;	// UAV pose
	bool c_isPanel;
	double c_posX, c_posY;
	double c_yaw0, c_pitch0,c_roll0;
	void setCamera(bool isPanel, double yaw, double pitch, double roll, double posX,double posY);

	double t_posX, t_posY, t_posZ;
	vector<double> t_posXs, t_posYs;
	double t_posN, t_posE, t_posD;
	double t_posN_p, t_posE_p;
	double t_spdN, t_spdE;
	double t_head;
	int stb_cont;
	void CheckStable(){
		static double lastPosX = 0;
		static double lastPosY = 0;
		static int cont = 0;

		double dPosX = fabs(t_posX - lastPosX);
		double dPosY = fabs(t_posY - lastPosY);

		if (dPosX < 0.05 && dPosY < 0.05) {
			cont = MIN(30, cont++);
		}
		else {
			cont = MAX(0, cont - 10);
		}
		isStable = (cont == 30 && t_height<0.55);
		lastPosX = t_posX;
		lastPosY = t_posY;
		stb_cont = cont;

	};
	bool isStable;

	double in_p_posN, in_p_posE, in_p_posD;
	double in_p_yaw, in_p_pitch, in_p_roll;
	void pPoseRecord();
	
	int tag_total;
	vector<Vec2f> tag_position;
	void setTagPara(vector<Vec2f> position,int len);

	bool in_isRotMat(Mat &R);
	Vec3f rotMat2Euler(Mat &R);
	vector<Mat> tmmp;

	KalmanFilter KF;
	KalmanFilter KF_H;
	void cacuHeight(double pRoll,double pPithc,double pYaw,double cPitch,double cYaw,double dist);
	Mat fny;	
	Mat ftgt;	
		
	vector<RotatedRect> fp_Ellips;
	vector<double> fp_weight;

	vector<Rect> ep_Rects;//-----ºòÑ¡ÍÖÔ²ËùÔÚµÄÎ»ÖÃ
	vector<RotatedRect> ep_ellips;//-----ºòÑ¡ÍÖÔ²µÄ±íÊ¾
	vector<double> ep_diffs;

	vector<Rect> tp_Rects;//-----ºòÑ¡ÍÖÔ²ËùÔÚµÄÎ»ÖÃ
	vector<RotatedRect> tp_ellips;//-----ºòÑ¡ÍÖÔ²µÄ±íÊ¾
	vector<vector<Point>> tp_contours;//-----ÄÚ²¿TÐÍµÄ±íÊ¾
	vector<vector<Point>> tp_Tlines;//-----ÄÚ²¿TÐÍµÄ±íÊ¾
	vector<double> tp_corrs;//-----Ïà¹Ø¶È

	vector<Point> conterT; //----	×îÓÅTÐÎÏßÂÖÀª
	vector<Point> conterTC;	//----- ×îÓÅÍ¹°ü
	vector<Point> conterTCA; //----- ×îÓÅÈý½Ç½üËÆ
	vector<Point> conterLines; //----- ×îÓÅTÐÎÏß¶Î¶Ëµã
	double linesDiff;

	Rect targetBox;	 //--- ±ê°ÐµÄÎ»ÖÃ
	Rect tTrackBox;
	Rect trackBox;  //---- ¸ú×ÙÎ»ÖÃ
	double trackBL;

	Rect centerBox;
	vector<Point> targetT;	//--- ±ê°ÐµÄTÐÍÏß¶Î
	vector<Point> targetContour;
	RotatedRect targetEllp;  //--- ±ê°ÐÍâÎ§ÍÖÔ²

	double ca_yaw;
	double ca_pitch;
	double ld_dist;

	bool isfindtarget;
	bool isfindFar;
	int findtimes;
	int nofindtimes;
	bool istrackTarget;
	bool isfindcenter;
	Point center_cp;
	double center_rd;
	int center_findtimes;
	int center_nofindtimes;

	bool isTypeChanged; //---- ÅÐ¶ÏÍ¼Ïñ×´Ì¬ÊÇ·ñ·¢Éú±ä»¯£¨½¹¾à¡¢ÊÓ³¡½Ç¡¢Í¼Ïñ³ß´ç£©
	void typeChanged();
	
	CaCtr cct;//----- Ïà»ú¿ØÖÆÖ¸ÁîÀà¡£
	CaSt cst; //----- ÊµÊ±Ïà»úÐÅÏ¢
	
	TargetInfo tgt;
	int m_state;

	vector<AprilTags::TagDetection> detections;
	//void targetPoseT();

	//------------------------ ÊäÈëÏà¹Ø
	void updataFrame(Mat ff);
	void updataCamera(double pitch, double yaw, double dist = 0);
	void updataPose(double roll, double pitch, double yaw, double posN = 0, double posE = 0, double posD = 0);
	void reshape(double para);
	void drawMap(Mat& map, int tpye = 0);
	void drawPosMap(Mat& map, int tpye = 0);
	void setrRoi(Rect rt,double para = 0.5);

	//------------------------ Ñ°ÕÒ±ê°Ð
	bool findTarget();
	bool findTarget_half();
		bool findCandidateEllps(Mat1b ff);	//----- Ñ°ÕÒºòÑ¡ÍÖÔ²
		bool findInnerT(Mat1b ff);			//-----   Ñ°ÕÒTÐÎ½á¹¹£¨ÂÖÀªÆ¥Åä£©
		bool findInnerTP(Mat1b ff);			//-----   Ñ°ÕÒTÐÎ½á¹¹£¨ÐÎ×´Æ¥Åä£©
		vector<Point> lineT(vector<Point> rawP, vector<Point> P3, double* diff);//---¼ì²âTÐÎµÄÁ½ÌõÏß
		vector<Point> LMM(vector<Point> conter, vector<Point> lines,bool flag=true, double* df = NULL);
		vector<Point2f> LMM_fitline(vector<Point2f> conter);
		bool readyTrack();
		double isEllps(vector<Point> pp, RotatedRect ellp);

	//----------------------- ¸ú×Ù
	KCFTracker tracker;
	bool trackTarget(Mat ff);
	bool trackTarget_init();
	void checkTarget();

	bool farFindTarget();
	bool isNearFind();

	bool nearTrackTarget();
	bool nearTrackTarget_init();
	bool isNearTarget();
	bool isNearTrack;
	void checkCenter(Mat ff);
	bool isReTrack();
	void resetTrackBox();
	bool isLoseNTrack();
	//----------------------- ×ËÌ¬½âËã

	void targetPose();
	void targetPose(Rect rt);
	bool targetPoseT();

	//----------------------- ÐÐÎª¿ØÖÆ
	void cameraCtr();
	void planeCtr();
	//------------------------ Êä³öÏÔÊ¾
	void show(int para=0);

	CnComm Camera;
	CnComm Wave;
	void init();
	bool isInit;

	vector<Vec4i> nonMaxDpress(vector<Vec4i> lines);

};

