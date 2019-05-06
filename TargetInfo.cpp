#include "stdafx.h"
#include "UAV_CTR.h"
using namespace cv;

/////----------------------  cameraState
extern vector<string>cameraMSG;
extern vector<string>ctrlerMSG;

cameraState::cameraState() {
	angHz = 0;
	angVt = 0;
	LDist = 0;
}
cameraState::~cameraState() {

}
void cameraState::setPara(double ph, double pv, double ah, double av) {
	pixHz = ph;
	pixVt = pv;
	aPxHz = ah;
	aPxVt = av;
	corHz = aPxHz / pixHz; //---- Ã¿ÏñËØ´ú±íµÄ¶ÈÊý
	corVt = aPxVt / pixVt;
	cx = ph / 2;
	cy = pv / 2;
	fx = ph / tan(ah / 2) / 2;
	fy = fx;
	//cout << "ph:= " << ph << " ah:= " << ah * 180 / CV_PI << "fx:= " << fx << endl;
}
void  cameraState::updata(double aHz, double aVt) {
	angVt = aVt;
	pitch = angVt * 180 / CV_PI;
	angHz = aHz;
	yaw = angHz * 180 / CV_PI;
}
void cameraState::updataD(double ya, double pi) {
	yaw = ya;
	angHz = yaw*CV_PI / 180;
	pitch = pi;
	angVt = pitch*CV_PI / 180;
}



/////----------------------  TargetInfo


TargetInfo::TargetInfo() {
	posX = 0;
	posY = 0;
	posZ = 0;
	dist = 0;
	angHz = 0;
	angVt = 0;
	angelT = 0;
}

TargetInfo::~TargetInfo() {


}

void TargetInfo::updata(CaSt cs, Rect ct, vector<Point> lineT) {
	double dse = MAX(ct.width*cs.corHz, ct.height*cs.corVt);
	double daHz = (ct.x + ct.width / 2 - cs.pixHz / 2)*cs.corHz;
	double daVt = (ct.y + ct.height / 2 - cs.pixVt / 2)*cs.corVt;

		double tanx = tan(daHz);
		double tany = tan(daVt);

	dist = 0.85 / dse *sqrt(1 + tanx*tanx + tany*tany); //---- 距离，单位m
	float cx = dist * cos(daVt)*cos(daHz);
	float cy = dist * cos(daVt)*sin(daHz);
	float cz = dist * sin(daVt);
	Mat rot;
	rotateMatR(cs.yaw, cs.pitch, 0, rot,0);// 0 == degree
	Mat cX = (Mat_<float>(3, 1) << cx, cy, cz);
	Mat pX = rot*cX;
	posX = pX.at<float>(0);
	posY = pX.at<float>(1);
	posZ = pX.at<float>(2);
		

	//angHz = cs.angHz - daHz;
	//angVt = cs.angVt - daVt;

	//posX = dist * cos(angVt) * cos(angHz);
	//posY = dist * cos(angVt) * sin(angHz);
	//posZ = dist * sin(angVt);


	Point gg = lineT[3] - lineT[2];
	Point pc = (lineT[0] + lineT[1]) / 2;
	if (norm(lineT[3] - pc) < norm(lineT[2] - pc))
		gg = -gg;
	angelT = -atan2(gg.x, gg.y);
}

void TargetInfo::updata(CaSt cs, Rect ct, double T) {
	double dse = MAX(ct.width*cs.corHz, ct.height*cs.corVt);
	double daHz = (ct.x + ct.width / 2 - cs.pixHz / 2)*cs.corHz;
	double daVt = (ct.y + ct.height / 2 - cs.pixVt / 2)*cs.corVt;

	dist = 0.88 / dse; //---- ¾àÀë£¬µ¥Î»mm
	angHz = cs.angHz - daHz;
	angVt = cs.angVt - daVt;

	posX = dist * cos(angVt) * cos(angHz);
	posY = dist * cos(angVt) * sin(angHz);
	posZ = dist * sin(angVt);
	
	angelT = T;
}

ostream & operator<<(ostream &out, TargetInfo &obj) {
	out << "X:= " << obj.posX << "  Y:= " << obj.posY << "  Z:= " << obj.posZ << endl;
	out << "r:= " << obj.dist << "  h:= " << obj.angHz / CV_PI * 180 << "  v:= " << obj.angVt / CV_PI * 180 << endl;
	out << "head:= " << obj.angelT /CV_PI*180 << endl;
	return out;
}

cameraControl::cameraControl() {

}

cameraControl::~cameraControl() {

}

bool cameraControl::open(int port, int baud) {
	return cn.Open(port, baud);
}

bool cameraControl::recvRead(uchar* buffer) {
	
	return true;
}
