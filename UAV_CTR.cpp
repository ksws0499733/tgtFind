#include "stdafx.h"
#include "UAV_CTR.h"
using namespace cv;

extern CONFIG cfg;

UAV_CTR::UAV_CTR()
{
	isfindtarget = false;
	istrackTarget = false;
	isfindcenter = false;
	isTypeChanged = false;
	isUpdata = false;
	isStable = false;
	stb_cont = 0;

	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool LAB = false;
	frd = NULL;
	tracker = KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	findtimes = 0;
	nofindtimes = 0;

	cst = CaSt();
	//	cst.setPara(1920, 1080, 71.4/ 180 * CV_PI, 40.16 / 180 * CV_PI);
	cst.setPara(640, 480, 71.4 / 180 * CV_PI / 1920 * 640, 71.4 / 180 * CV_PI / 1920 * 480);
	tROI = Rect(960 - 320, 540 - 240, 640, 480);
	tgt = TargetInfo();
	scale = 1;
	UI = Mat(1000, 1780, CV_8UC3,Scalar(0,0,0));
	scale_tk = 1;
	fps = 0;

	diffX = 0;
	diffY = 0;
	ld_dist = -1;
	t_height = 10;

	p_roll = p_yaw = p_pitch = 0;
	p_roll2 = p_yaw2 = p_pitch2 = 0;
	c_yaw = c_pitch = 0;

	m_state = ST_IDLE;
	T0 = GetTickCount();
	p_posN0 = 0;
	p_posE0 = 0;
	p_posD0 = 0;
	p_posN = 0;
	p_posE = 0;
	p_posD = 0;
	p_posN2 = 0;
	p_posE2 = 0;
	p_posD2 = 0;

	float dt = 0.1;
	KF = KalmanFilter(2, 2);
	KF.transitionMatrix = (Mat_<float>(2, 2) << 1, 0,  0, 1);  //转移矩阵A
	setIdentity(KF.measurementMatrix);                                             //测量矩阵H
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
	setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
	randn(KF.statePost, 0, 0.1);

	KF_H = KalmanFilter(2, 1);
	KF_H.transitionMatrix = (Mat_<float>(2, 2) << 1, 0.08,   0, 1);  //转移矩阵A
	setIdentity(KF_H.measurementMatrix);                                             //测量矩阵H
	setIdentity(KF_H.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
	setIdentity(KF_H.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
	setIdentity(KF_H.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
	randn(KF_H.statePost, 0, 0.1);

	tmmp = vector<Mat>(36);
	char fn[20];
	Mat tmp, tmp2;
	for (int ii = 0;ii < 9;ii++) {
		sprintf(fn, "temps\\tmp%d.png", ii);
		tmp = imread(fn, 0);
		cout << "temp " << ii << " read success..." << endl;
		tmmp[ii] = tmp.clone();// 0~80°

		transpose(tmp, tmp2);
		flip(tmp2, tmp2, 1);
		tmmp[ii + 9] = tmp2.clone();// 90~170°

		flip(tmp, tmp2, -1);
		tmmp[ii + 18] = tmp2.clone();// 180~260°

		transpose(tmp, tmp2);
		flip(tmp2, tmp2, 0);
		tmmp[ii + 27] = tmp2.clone();// 270~350°		
	}

}


UAV_CTR::~UAV_CTR()
{
}

void UAV_CTR::updataFrame(Mat ff) {
	
	rframe = ff.clone();

	frame = rframe;
	rsize = frame.size();
	//resize(rframe, tframe, rsize * scale_tk);

	if (isTypeChanged) {
		m_state = ST_FIND;
		rROI = Rect(0, 0, ff.cols, ff.rows);
		isTypeChanged = false;
	}	
	if (rROI.width == 0) rROI = Rect(0, 0, ff.cols, ff.rows)*scale;

	isUpdata = true;
}
void UAV_CTR::updataCamera(double pitch, double yaw, double dist) {
	cst.updataD(yaw, pitch);
	c_pitch = pitch;
	c_pitchs.push_back(pitch);
	c_yaw = yaw;
	c_yaws.push_back(yaw);
	if (dist > 0) {
		//KF_H.predict();
		//Mat mr = (Mat_<float>(1, 1) << dist);
		//Mat rst = KF_H.correct(mr);
		cst.LDist = dist;//rst.at<float>(0);
		p_height = dist;//rst.at<float>(0);
	}
	cacuHeight(p_roll, p_pitch, p_yaw, c_pitch, c_yaw, p_height);
}

void UAV_CTR::updataPose(double roll, double pitch, double yaw, double posN, double posE, double posD) {
	p_roll2 = p_roll;
	p_pitch2 = p_pitch;
	p_yaw2 = p_yaw;
	
	p_roll = roll;
	p_pitch = pitch;
	p_yaw = yaw;

	//p_rolls.push_back(roll);
	//p_pitchs.push_back(pitch);
	//p_yaws.push_back(yaw);
	//if (p_rolls.size() > 100 && c_pitchs.size()>100) {

	//	p_rolls.erase(p_rolls.begin());
	//	p_pitchs.erase(p_pitchs.begin());
	//	p_yaws.erase(p_yaws.begin());
	//}

	p_posN2 = p_posN;
	p_posE2 = p_posE;
	p_posD2 = p_posD;

	p_posN = posN ;
	p_posE = posE ;
	p_posD = posD ;

	//	cout << "p_posN:= " << p_posN << "  p_posE:= " << p_posE << endl;
	p_posNEs.push_back(Point2f(p_posN, p_posE));
	if (p_posNEs.size() > 100) {
		p_posNEs.erase(p_posNEs.begin());
	}
}

void UAV_CTR::setCamera(bool isMovable, double yaw, double pitch, double roll, double posX, double posY) {
	c_isPanel = isMovable;
	c_yaw0 = yaw;
	c_pitch0 = pitch;
	c_roll0 = roll;
	c_posX = posX;
	c_posY = posY;
}

void UAV_CTR::reshape(double para) {


}
void UAV_CTR::setrRoi(Rect rn, double para) {
	Rect rt;
	if (rROI.height == 0) {
		rROI = Rect(0, 0, frame.cols, frame.rows);
	}
	if (isfindtarget) {
		rt = rn;
		rt.x = rt.x + rROI.x;
		rt.y = rt.y + rROI.y;
	}
	else {
		rt = rROI;
	}

	double x1 = rt.x - para* rt.width;
	double x2 = rt.x + (1 + para)* rt.width;
	double y1 = rt.y - para* rt.height;
	double y2 = rt.y + (1 + para)* rt.height;
	x1 = MAX(0, x1);
	y1 = MAX(0, y1);
	x2 = MIN(frame.cols, x2);
	y2 = MIN(frame.rows, y2);
	rROI = Rect(x1, y1, x2 - x1, y2 - y1);
}

bool UAV_CTR::farFindTarget() {
	

	double in_ca_yaw = cst.yaw;
	double in_ca_pitch = cst.pitch;


	if (frame.empty())
		return false;

	//setrRoi(targetBox,1);
	cvtColor(frame, gray, CV_BGR2GRAY);
	
	double mr = 0.88 / (2 * p_height * tan(cst.aPxHz / 2)) * cst.pixHz;
	if (mr <= 0 || mr > 128) {
		isfindFar = false;
		return false;
	}

	Mat tmr;	
	Mat result;
	double mmax, mmin;
	Point mmaxID, mminID;

	double maxmax = 0;
	Point maxmaxID;
	int maxii;


	for (int ii = 0;ii < 36;ii++) {
		resize(tmmp[ii], tmr, Size(mr, mr));
		matchTemplate(gray, tmr, result, TM_CCOEFF_NORMED);
		minMaxLoc(result, &mmin, &mmax, &mminID, &mmaxID);
		if (mmax > maxmax) {
			maxmax = mmax;
			maxmaxID = mmaxID;
			maxii = ii;
		}
	}
	if (maxmax > 0.6) {
		Rect rt;
		rt.x = maxmaxID.x - mr / 2;
		rt.y = maxmaxID.y - mr / 2;
		rt.width = mr;
		rt.height = mr;
		targetPose(rt);
		findtimes++;
		nofindtimes = 0;
		targetEllp.center = maxmaxID;
		targetEllp.size = Size(mr, mr);
		targetEllp.angle = maxii * 10;

		isfindFar = true;
	}
	else {

		nofindtimes++;
		findtimes = 0;
		isfindFar = false;
	}
	return true;
}

bool UAV_CTR::isNearFind() {
	if ((targetBox.area() > 48 * 48 & findtimes > 3 )| p_height < 6) {
		isfindFar = false;
		return true;
	}
	else
		return false;
}

bool UAV_CTR::findTarget_half() {
	double t1, t2, t3, t4;
	t1 = GetTickCount();

	isfindFar = false;

	if (rframe.empty())
		return false;

	setrRoi(targetBox);

	cvtColor(rframe(rROI), gray, CV_RGB2GRAY);
	Mat grayh;
	resize(gray, grayh, gray.size() / 2);

	adaptiveThreshold(grayh, gray2, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 33, 5);
	//dilate(gray2, gray2, Mat());


	t2 = GetTickCount();
	//cout << "=========t1:= " << (t2 - t1) / 1000.0 << endl;

	findCandidateEllps(gray2); //--- find candidate ellips	
	findInnerTP(gray2);  //---- find T in ellips

						 //Mat cframe = rframe.clone();
						 //for (int ii = 0; ii < ep_ellips.size(); ii++) {
						 //	ellipse(cframe(rROI), ep_ellips[ii], Scalar(100,255,255), 1);
						 //}
						 //imshow("eee", cframe);
						 //imshow("gray", gray2);

						 //------------------------------ output
	double ms = 0;
	double mSa = 0;
	double mDs = 65535;
	int mmii = 0;

	vector<Point> in_targetT;
	vector<Point> in_targetContour;
	RotatedRect in_targetEllp;

	Rect in_targetBox;
	for (int ii = 0; ii < tp_ellips.size(); ii++) {
		Rect roic = tp_Rects[ii];
		double sR = roic.area();
		double dS = norm(Point(roic.x + roic.width / 2, roic.y + roic.height / 2) - Point(frame.cols / 2, frame.rows / 2));
		if (mSa < sR && mDs > dS*0.9) {
			mDs = MIN(mDs, dS);
			mSa = MAX(mSa, sR);
			mmii = ii;
			in_targetBox = roic;		//×îÓÅ¾ØÐÎ¿ò	
		}
	}

	if (in_targetBox.height != 0 && in_targetBox.height != 0) {

		targetBox = in_targetBox*2;		//×îÓÅ¾ØÐÎ¿ò				
		targetContour = tp_contours[mmii];	//×îÓÅTÂÖÀª
		targetT = LMM(tp_contours[mmii], tp_Tlines[mmii]); //×îÓÅTÏß¶Î	
		targetEllp = tp_ellips[mmii];
			targetEllp.center.x *= 2;
			targetEllp.center.y *= 2;
			targetEllp.size.height *= 2;
			targetEllp.size.width *= 2;
		isfindtarget = true;
		findtimes++;
		nofindtimes = 0;
		targetPose();
		diffX = rROI.x + targetBox.x + targetBox.width / 2 - rsize.width / 2;
		diffY = rROI.y + targetBox.y + targetBox.height / 2 - rsize.height / 2;
		//	cout << "rsize:=" << rsize << endl;
	}
	else {
		isfindtarget = false;
		findtimes = 0;
		nofindtimes++;
	}

	t4 = GetTickCount();
	//cout << "=========t3:= " << (t4 - t3) / 1000.0 << endl;
	//cout << "=========t0:= " << (t4 - t1) / 1000.0 << endl;

	return targetT.size();

}


bool UAV_CTR::findTarget() {
	
	if (targetBox.width > 200) {
		return findTarget_half();
	}
		
	double t1, t2, t3,t4;
	t1 = GetTickCount();

	isfindFar = false;

	if (rframe.empty())
		return false;

	setrRoi(targetBox);

	cvtColor(rframe(rROI), gray, CV_RGB2GRAY);

	adaptiveThreshold(gray, gray2, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 33, 5);
	//dilate(gray2, gray2, Mat());


	t2 = GetTickCount();
	//cout << "=========t1:= " << (t2 - t1) / 1000.0 << endl;

	findCandidateEllps(gray2); //--- find candidate ellips	
	findInnerTP(gray2);  //---- find T in ellips

	//Mat cframe = rframe.clone();
	//for (int ii = 0; ii < ep_ellips.size(); ii++) {
	//	ellipse(cframe(rROI), ep_ellips[ii], Scalar(100,255,255), 1);
	//}
	//imshow("eee", cframe);
	//imshow("gray", gray2);

	//------------------------------ output
	double ms = 0;
	double mSa = 0;
	double mDs = 65535;
	int mmii = 0;
	
	vector<Point> in_targetT;
	vector<Point> in_targetContour;
	RotatedRect in_targetEllp;

	Rect in_targetBox;
	for (int ii = 0; ii < tp_ellips.size(); ii++) {
		Rect roic = tp_Rects[ii];
		double sR = roic.area();
		double dS = norm(Point(roic.x + roic.width / 2, roic.y + roic.height / 2) - Point(frame.cols / 2, frame.rows / 2));
		if (mSa < sR && mDs > dS*0.9) {
			mDs = MIN(mDs, dS);
			mSa = MAX(mSa, sR);
			mmii = ii;
			in_targetBox = roic;		//×îÓÅ¾ØÐÎ¿ò	
		}
	}
	
	if (in_targetBox.height != 0 && in_targetBox.height != 0) {

		targetBox = in_targetBox;		//×îÓÅ¾ØÐÎ¿ò				
		targetContour = tp_contours[mmii];	//×îÓÅTÂÖÀª
		targetT = LMM(tp_contours[mmii], tp_Tlines[mmii]); //×îÓÅTÏß¶Î	
		targetEllp = tp_ellips[mmii];
		isfindtarget = true;
		findtimes++;
		nofindtimes = 0;
		targetPose();
		diffX = rROI.x + targetBox.x + targetBox.width / 2 - rsize.width / 2;
		diffY = rROI.y + targetBox.y + targetBox.height / 2 - rsize.height / 2;
	//	cout << "rsize:=" << rsize << endl;
	}
	else {
		isfindtarget = false;
		findtimes = 0;
		nofindtimes++;
	}

	t4 = GetTickCount();
	//cout << "=========t3:= " << (t4 - t3) / 1000.0 << endl;
	//cout << "=========t0:= " << (t4 - t1) / 1000.0 << endl;

	return targetT.size();


}
//------------- Ñ°ÕÒºòÑ¡ÍÖÔ²
bool UAV_CTR::findCandidateEllps(Mat1b ff) {
	
	Mat temp(ff.size(), CV_8UC3);
	temp.setTo(0);
	vector<vector<Point>> conters;
	vector<Vec4i> parents;
	findContours(ff.clone(), conters, parents, RETR_LIST, CHAIN_APPROX_NONE);
	
	vector<Point> cnt_cvx;
	double S, Sc;//, Se, Sb, Si;	//Ãæ»ý Í¹°üÃæ»ý	ÍÖÔ²Ãæ»ý	×îÐ¡°üÎ§¾ØÐÎÃæ»ý	Ïà½»Ãæ»ý
	RotatedRect ellp, bd;
	ep_Rects.clear();
	ep_ellips.clear();
	ep_diffs.clear();

	vector<Rect> in_ep_Rects;//-----ºòÑ¡ÍÖÔ²ËùÔÚµÄÎ»ÖÃ
	vector<RotatedRect> in_ep_ellips;//-----ºòÑ¡ÍÖÔ²µÄ±íÊ¾
	vector<double> in_ep_diffs;
	vector<Point> in_ep_cnts;

	for (int ii = 0; ii < conters.size(); ii++) {

		vector<Point> pp = conters[ii];

		Rect nrt = boundingRect(pp);
		if (nrt.x<3 || nrt.y <3 || (nrt.x + nrt.width)>(ff.cols - 3) || (nrt.y + nrt.height)>(ff.rows - 3)) continue;

		convexHull(pp, cnt_cvx);
		if (cnt_cvx.size() < 8)continue;
		
		S = contourArea(pp);
		Sc = contourArea(cnt_cvx);		
		//if (Sc > ff.rows*ff.cols*3.3 )continue;	//-------É¸³ýÃæ»ý´óµÄ
		if (S < 16*16 )continue;	//-------É¸³ýÃæ»ýÐ¡µÄ£¬°¼µÄ
		

		ellp = fitEllipse(cnt_cvx);
		//double ddd = fabs(ellp.size.width - ellp.size.height);
		//double dds = (ellp.size.width + ellp.size.height);
		//if (ddd / dds < 0.01) {
		//	ellp.size.width = dds / 2;
		//	ellp.size.height = dds / 2;
		//	ellp.angle = 0;
		//}
			


		double bbll = ellp.size.height / ellp.size.width;
		double Se = ellp.size.area() * CV_PI / 4;

	//	cout <<"Se:="<<Se<<"  S:="<<S<<"  bl:=" << Se / S << endl;

		if (bbll > 2 || bbll < 0.5) continue;				
		if (Se / S < 90.0/100 || Se / S > 100.0/90) continue;

		//polylines(temp, pp, 1, Scalar(0, 0, 255));
		//polylines(temp, cnt_cvx, 1, Scalar(255, 0, 255));
		//ellipse(temp, ellp, Scalar(0, 255, 255));

		double ss = isEllps(cnt_cvx, ellp);
//		cout << ss << endl;

		if( ss< 0.96){
			continue;
		}
		
		
		//drawContours(temp, conters, ii, Scalar(0, 0, 255));

		//------------------- 
		Rect roic = boundingRect(pp);

		ep_Rects.push_back(roic);
		ep_ellips.push_back(ellp);
		ep_diffs.push_back(ss);
	}
	//cout << "-------\n";
	//---------------- 

	//imshow("cnt", temp);

	double ds0, ds1;
	Rect rt0 ,rt1;
	Point p0, p1;
	RotatedRect rot0, rot1;
	double s0, s1;
	

	if (ep_Rects.size() > 1) {

		for (int ii = 0; ii < ep_Rects.size(); ii++) {

			in_ep_Rects.clear();
			in_ep_ellips.clear();
			in_ep_diffs.clear();

			ds0 = ep_diffs[0];
			rt0 = ep_Rects[0];
			rot0 = ep_ellips[0];
			s0 = rt0.area();
			for (int jj = 0; jj < ep_Rects.size(); jj++) {

				ds1 = ep_diffs[jj];
				rt1 = ep_Rects[jj];
				rot1 = ep_ellips[jj];
				//   IUO of  rt0 & rt1

				double IUO = 0;
				double aa = MIN(rt0.x + rt0.width, rt1.x + rt1.width) - MAX(rt0.x, rt1.x);
				double bb = MIN(rt0.y + rt0.height, rt1.y + rt1.height) - MAX(rt0.y, rt1.y);
				s1 = rt1.area();

				if (aa <= 0 | bb <= 0) {
					IUO = 0;
				}
				else {
					double ssI = aa*bb;
					double ssU = s0 + s1 - ssI;
					IUO = ssI / ssU;
				}


				if (IUO > 0.6) {
				    // choose the bigger one.
					if (s1 > s0) {
						ds0 = ds1;
						rt0 = rt1;
						rot0 = rot1;
					}
				}
				else {
					in_ep_Rects.push_back(rt1);
					in_ep_ellips.push_back(rot1);
					in_ep_diffs.push_back(ds1);
				}
			}
			in_ep_Rects.push_back(rt0);
			in_ep_ellips.push_back(rot0);
			in_ep_diffs.push_back(ds0);

			ep_Rects = in_ep_Rects;
			ep_ellips = in_ep_ellips;
			ep_diffs = in_ep_diffs;
		}

	}

	
	//cout << ep_Rects.size() << endl;


	return ep_Rects.size();
}

double UAV_CTR::isEllps(vector<Point> pp, RotatedRect ellp) {
	double diff = 0;
	double diff2 = 0;
	double len = pp.size();

	double sxy = 0, sx = 0, sy = 0, sx2 = 0, sy2 = 0;
	double Ex, Ex2, Ey, Ey2, Exy;
	double Var_x, Var_y, Cov_xy, Corr_xy;
	double eb = ellp.size.width/2;	//¶Ì±ß
	double ea = ellp.size.height/2; //³¤±ß
	double eg = ellp.angle /180*CV_PI;
	double ex = ellp.center.x;
	double ey = ellp.center.y;

	string st;
	for (int ii = 0; ii < pp.size(); ii++) {
		Point p0 = pp[ii];
		double px = p0.x - ex;
		double py = p0.y - ey;
		double pg = atan2(py, px) - eg + CV_PI/2;
		double xx = ea*cos(pg);
		double yy = eb*sin(pg);

		double td = sqrt(xx*xx + yy*yy);
		double dd = sqrt(px*px + py*py);

		diff += (td - dd);
		diff2 += (td - dd)*(td - dd);

		sxy += td*dd;
		sx  += td;
		sx2 += td*td;
		sy  += dd;
		sy2 += dd*dd;				
	}
	Ex = sx / len;
	Ey = sy / len;
	Exy = sxy / len;
	Ex2 = sx2 / len;
	Ey2 = sy2 / len;

	Corr_xy = (Exy - Ex*Ey) / (sqrt((Ex2 - Ex*Ex)*(Ey2 - Ey*Ey))+1e-5);
	if ((Exy - Ex*Ey) < 1)Corr_xy = 0.999;

	return Corr_xy;
}

//------------- find far circles in Gray
bool UAV_CTR::findInnerT(Mat1b ff) {  //»Ò¶ÈÍ¼
	vector<vector<Point>> in_conters;
	vector<Vec4i> in_parents;
	Mat temp(ff.size(), CV_8UC1);
	
	Point2f p1, p2, p3, p4, p12, p34;
	Mat result;
	vector<Point> conterC;
	vector<Point> conterCA;

	static RotatedRect last_rtr;
	//static Mat tmmp = imread("tmmp.png",0);

	tp_Tlines.clear();
	tp_corrs.clear();
	tp_contours.clear();
	tp_Rects.clear();
	tp_ellips.clear();
	targetT.clear();
	double mii, mjj;
	double mmcorr = 0;

	int nn = 0;

	Mat temps(ff.size(), CV_8UC1);
	temps.setTo(255);
	for (int ii = 0; ii < ep_Rects.size(); ii++) {
		Rect roic = ep_Rects[ii];

		RotatedRect rtr = ep_ellips[ii];

		double lw = round((rtr.size.width + rtr.size.height) / 2 / 10);

		p1 = rtr.center;
		rtr.size.width *= 0.82;
		rtr.size.height *= 0.82;



		double dnd = (rtr.size.width + rtr.size.height) / 4;

		double mcorr = 0;

		for (int jj = 0; jj < 18; jj++) {
			temp.setTo(255);
			ellipse(temp, rtr, Scalar(0, 0, 0), MAX(2, lw));
			p12 = Point2f(sin(jj*10.0*CV_PI / 180), cos(jj*10.0*CV_PI / 180));
			line(temp, p1 - dnd * p12, p1 + dnd * p12, Scalar(0, 0, 0), MAX(2, lw));
			matchTemplate(ff(roic), temp(roic), result, TM_CCOEFF_NORMED);
			double corr = result.at<float>(0);

			if (corr > mcorr) {
				mcorr = corr;
				mjj = jj;
			}
		}
		//°Ñ´óÓÚ0.5µÄ¼ÇÂ¼ÏÂÀ´
		if (mcorr > 0.5) {		
			fp_Ellips.push_back(rtr);//ÐÎ×´ÊÇµÚÒ»Î»
			nn++;
			ellipse(temps, rtr, Scalar(0, 0, 0), MAX(2, lw));
		}

		if (mcorr > mmcorr) {
			mmcorr = mcorr;
			mii = ii;
		}
	}

	while (fp_Ellips.size() > 20) {
		fp_Ellips.erase(fp_Ellips.begin());
	}
	//double wt[20] = { 0 };
	//for (int ii = fp_Ellips.size() - 1; ii > -1; ii--) {
	//	RotatedRect r1 = fp_Ellips[ii];
	//	double s1 = r1.size.area();
	//	double b1 = r1.size.width / r1.size.height;

	//	for (int jj = ii - 1; ii > -1; jj--) {
	//		RotatedRect r2 = fp_Ellips[jj];
	//		double s2 = r2.size.area();
	//		double b2 = r2.size.width / r2.size.height;

	//		bool a1 = (s1*0.9 < s2 && s2 < s1 / 0.9);
	//		bool a2 = (b1*0.9 < b2 && b2 < b1 / 0.9);
	//		
	//		if (a1 && a2) {
	//			wt[ii]++;
	//			wt[jj]++;
	//		}
	//	}
	//	nn--;
	//	if (nn < 0) {
	//		break;
	//	}
	//}

	

	//imshow("sdf", temps);
	//cout << mmcorr << endl;
	if (mmcorr < 0.5) {
		return 0;
	}
	
		// get the biggist corr
		Rect mroic = ep_Rects[mii];
		RotatedRect mrtr = ep_ellips[mii];
		p1 = mrtr.center;
		mrtr.size.width *= 0.84;
		mrtr.size.height *= 0.84;

		targetEllp = mrtr;
		targetBox = mroic;
		tgt.updata(cst, targetBox + rROI, 0);




	return 1;
}

//--------- find T in a ROI
bool UAV_CTR::findInnerTP(Mat1b ff) {  
		
	vector<vector<Point>> in_conters;
	vector<Vec4i> in_parents;
	Mat temp(128, 128, CV_8UC1); Mat temp2(128, 128, CV_8UC1);
	Point2f p1, p2, p3, p4, p12, p34;
	Mat result;
	vector<Point> conterC;
	vector<Point> conterCA;

	tp_Tlines.clear();
	tp_corrs.clear();
	tp_contours.clear();
	tp_Rects.clear();
	tp_ellips.clear();

	for (int ii = 0; ii < ep_Rects.size(); ii++) {
		Rect roic = ep_Rects[ii];

	//	rectangle(rframe, roic+rROI, Scalar(0, 155, 155), 3);

		Mat roi = ff(roic).clone(); //----- template
		resize(roi, roi, Size(128, 128));
		roi(Rect(64 - 6, 64 - 6, 12, 12)).setTo(0);

		

		findContours(roi.clone(), in_conters, in_parents, RETR_LIST, CHAIN_APPROX_NONE);

		for (int kk = 0; kk < in_conters.size(); kk++) {
			vector<Point> pp = in_conters[kk];
				//------------ 
			Rect brt = boundingRect(pp);
			if (!brt.contains(Point2f(64, 64))) {
				continue;
			}				

			convexHull(pp, conterC);	//----	 Í¹°ü
			approxPolyDP(conterC, conterCA, 25, 1);		//--- Èý½Ç½üËÆ
			approxPolyDP(conterCA, conterCA, 25, 1);		//--- Èý½Ç½üËÆ

			if (conterCA.size() != 3)continue;  //---------- triangle

			conterLines = lineT(pp, conterCA, &linesDiff); //-----
			conterLines = LMM(pp, conterLines, 1, &linesDiff);
				
			if (linesDiff > 3) continue;

			//--------- Á½ÌõÏß¶ÎµÄ¼Ð½Ç
			p1 = conterLines[0];
			p2 = conterLines[1];
			p3 = conterLines[2];
			p4 = conterLines[3];
			p12 = p2 - p1;
			p34 = p4 - p3;
			double d12 = norm(p12);
			double d34 = norm(p34);
			double dd = p12.dot(p34) / d12 / d34;
			if (fabs(dd) > 0.5)continue;

			//--------- Ä£°åÆ¥Åä¶È
			temp.setTo(255);
			line(temp, p1, p2, 0, 10);
			line(temp, p3, p4, 0, 10);
			circle(temp, Point(64, 64), 64 - 8, 0, 10);

			temp2.setTo(255);
			line(temp2, p1, p2, 0, 10);
			line(temp2, p3, p4, 0, 10);
			circle(temp2, Point(64, 64), 64 + 8, 0, 10);

			//	imshow("temp", temp);
			//	imshow("ff", roi);

			matchTemplate(roi, temp, result, TM_CCOEFF_NORMED);
			double corr1 = result.at<float>(0);

			matchTemplate(roi, temp2, result, TM_CCOEFF_NORMED);
			double corr2 = result.at<float>(0);

			double corr = MAX(corr1, corr2);

			//cout << corr <<" Rect:="<< ep_Rects[ii]<< endl;

			//tp_Tlines.push_back(conterLines);
			//tp_corrs.push_back(corr);
			//tp_contours.push_back(pp);
			//tp_Rects.push_back(ep_Rects[ii]);
			//tp_ellips.push_back(ep_ellips[ii]);
			//break;


			if (corr < 0.3) { 
				continue; 
			}
			else {
				//if (corr2 > corr1) {
				//	ep_Rects[ii].height *= 1.25;
				//	ep_Rects[ii].width *= 1.25;
				//	ep_Rects[ii].x -= ep_Rects[ii].width*0.125;
				//	ep_Rects[ii].y -= ep_Rects[ii].height*0.125;
				//}

				

				tp_Tlines.push_back(conterLines);
				tp_corrs.push_back(corr);
				tp_contours.push_back(pp);
				tp_Rects.push_back(ep_Rects[ii]);
				tp_ellips.push_back(ep_ellips[ii]);
				break;
			}
		}

		imshow("rrot", roi);
	}
	return 1;
}

void UAV_CTR::init() {
	isInit = true;
}

bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
{
	//Number of key points
	int N = key_point.size();

	//构造矩阵X
	cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
		for (int j = 0; j < n + 1; j++)
		{
			for (int k = 0; k < N; k++)
			{
				X.at<double>(i, j) = X.at<double>(i, j) +
					std::pow(key_point[k].x, i + j);
			}
		}
	}

	//构造矩阵Y
	cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
		for (int k = 0; k < N; k++)
		{
			Y.at<double>(i, 0) = Y.at<double>(i, 0) +
				std::pow(key_point[k].x, i) * key_point[k].y;
		}
	}

	A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
	//求解矩阵A
	cv::solve(X, Y, A, cv::DECOMP_LU);
	return true;
}
int maxPoint(vector<Point> rawA, double dowmBd = 0, double upBd = 0) {
	if (dowmBd > upBd)
		upBd += 360;
	int N = rawA.size();
	int maxV = 0, maxI = -1;
	for (int ii = 0; ii < N; ii++) {
		int ss = rawA[ii].x;
		int dd = rawA[ii].y;
		if ((ss < upBd && ss > dowmBd) || ((ss - 360) < upBd && (ss - 360) > dowmBd) || ((ss + 360) < upBd && (ss + 360) > dowmBd)) {
			if (maxV < dd) {
				maxV = dd;
				maxI = ii;
			}
		}				
	}
	return maxI;
}

int maxPoint2(vector<Point> rawA, double dowmBd = 0, double upBd = 0) {
	if (dowmBd > upBd)
		upBd += 360;
	int N = rawA.size();
	int maxV = 0, maxI = -1,maxS = 0;
	for (int ii = 0; ii < N; ii++) {
		int ss = rawA[ii].x;
		int dd = rawA[ii].y;
		if ((ss <= upBd && ss > dowmBd) || ((ss - 360) <= upBd && (ss - 360) > dowmBd) || ((ss + 360) <= upBd && (ss + 360) > dowmBd)) {
			if (maxV < dd) {
				maxV = dd;
				maxI = ii;
				maxS = ss;
			}
		}
	}
	return (maxS+3600)%360;
}


vector<Point> ineT(vector<Point> rawP) {
	int N = rawP.size();
	Scalar center_s = mean(rawP);
	Point center = Point(center_s[0], center_s[1]);
	vector<Point> rawA(N);
	vector<int> rawI(360);
	for (int i = 0; i < N; i++) {
		double dx = rawP[i].x - center_s[0];
		double dy = rawP[i].y - center_s[1];
		double dr = sqrt(dx*dx + dy*dy);
		double ds = atan2(dy, dx) * 180 / CV_PI;
		rawA[i] = Point(ds, dr);		
	}
	for (int i = 0; i < 360; i++) {
		rawI[i] = 0;
	}


	//cout << "-----------------" << endl;
	//cout << rawA << endl;	
	//cout << "-----------------" << endl;
	for (int ii = 0; ii < 120; ii = ii+20) {
		//int a1 = maxPoint2(rawA, ii, ii + 120);
		//int a2 = maxPoint2(rawA, ii + 120, ii + 240);
		//int a3 = maxPoint2(rawA, ii + 240, ii + 360);

		//cout << ii << " - " << ii + 120 << "   " << a1 << endl;
		//cout << ii+120 << " - " << ii + 240 << "   " << a2 << endl;
		//cout << ii+240 << " - " << ii + 360 << "   " << a3 << endl;

		rawI[maxPoint2(rawA, ii, ii + 120)]++;
		rawI[maxPoint2(rawA, ii+120, ii + 240)]++;
		rawI[maxPoint2(rawA, ii+240, ii + 360)]++;	

	}
	cout << "................." << endl;
	for (int i = 0; i < 360; i++) {
		if(rawI[i])
			cout << i << ":= " << rawI[i] << endl;
	}
	cout << "................." << endl;
	

	return rawP;
}

//----------
vector<Point> UAV_CTR::lineT(vector<Point> rawP, vector<Point> P3, double* diff) {
	
	ineT(rawP);
	//------
	Point2f p1, p2, p3, p4, p01, p02, p03, p04;
	double  S = contourArea(P3);
	double d12 = norm(p1 - p2);
	double d23 = norm(p2 - p3);
	double d13 = norm(p1 - p3);
	int cor, maxcor = 0;
	//double a, b, c;
	vector<Point2f> in_line;
	vector<Point2f> out_line;
	double dd12, dd34, dds, mindds;
	Point2f p_12, p_34;
	Point2f p_12T, p_34T;
	mindds = 65535;
	for (int i = 0; i < 3; i++) {
		cor = 0;
		dd12 = 0;
		dd34 = 0;
		vector<Point2f> in_line0;
		vector<Point2f> out_line0;

		p1 = P3[i];
		p2 = P3[(i + 1) % 3];
		p3 = P3[(i + 2) % 3];
		p4 = (p1 + p2) / 2;

		p_12 = p2 - p1;
		p_34 = p4 - p3;
		p_12T = Point2f(-p_12.y, p_12.x) / norm(p_12);
		p_34T = Point2f(-p_34.y, p_34.x) / norm(p_34);

		for (int jj = 0; jj < rawP.size(); jj++) {

			Point2f p0 = rawP[jj];
			Point2f p_01 = Point2f(p0.x - p1.x, p0.y - p1.y);
			Point2f p_03 = Point2f(p0.x - p3.x, p0.y - p3.y);
			double h1 = fabs(p_01.dot(p_12T));
			double h2 = fabs(p_03.dot(p_34T));

			if (h1 > h2) {
				out_line0.push_back(p0);
				dd34 += h2*h2;
			}
			else {
				in_line0.push_back(p0);
				dd12 += h1*h1;
			}
		}
		dds = sqrt(dd12) / in_line0.size() + sqrt(dd34) / out_line0.size();
		if (dds < mindds) {
			mindds = dds;
			p01 = p1;
			p02 = p2;
			p03 = p3;
			p04 = p4;
			in_line = in_line0;
			out_line = out_line0;
		}
	}
	vector<Point> lines;
	lines.push_back(p01);
	lines.push_back(p02);
	lines.push_back(p03);
	lines.push_back(p04);

	*diff = mindds;

	return lines;
}



vector<Point> UAV_CTR::LMM(vector<Point> conter, vector<Point> lines,bool flag, double* df) {
	// ---------- »ùÓÚÏßÐÔ»ìºÏÄ£ÐÍµÄÖ±Ïß¾«Ï¸»¯
	Point2f p1, p2, p3, p4;
	p1 = lines[0];
	p2 = lines[1];
	p3 = lines[2];
	p4 = lines[3];

	double dd12, dd34, dds, mindds;
	Point2f p_12, p_34;
	Point2f p_12T, p_34T;
	mindds = 65535;

	vector<Point2f> in_line;
	vector<Point2f> out_line;

	for (int ii = 0; ii < 100;ii++) {

		p_12 = p2 - p1;
		p_34 = p4 - p3;
		p_12T = Point2f(-p_12.y, p_12.x) / norm(p_12);
		p_34T = Point2f(-p_34.y, p_34.x) / norm(p_34);

		dd12 = 0;
		dd34 = 0;
		for (int ii = 0; ii < conter.size(); ii++) {
			Point2f p0 = conter[ii];

			Point2f p_01 = Point2f(p0.x - p1.x, p0.y - p1.y);
			Point2f p_03 = Point2f(p0.x - p3.x, p0.y - p3.y);
			double h1 = fabs(p_01.dot(p_12T));
			double h2 = fabs(p_03.dot(p_34T));

			if (h1 > h2) {
				out_line.push_back(p0);
				dd34 += h2;
			}
			else {
				in_line.push_back(p0);
				dd12 += h1;
			}
		}
		dds = (dd12) / in_line.size() + (dd34) / out_line.size();
		//	cout << dds << endl;
		if (fabs(mindds - dds) < 0.1) {
			break;
		}
		else {
			mindds = dds;
		}

		vector<Point2f> lin = LMM_fitline(in_line);
		vector<Point2f> out = LMM_fitline(out_line);

		p1 = lin[0];
		p2 = lin[1];
		p3 = out[0];
		p4 = out[1];

	}
	if(flag) lines.clear();

	lines.push_back(p1);
	lines.push_back(p2);
	lines.push_back(p3);
	lines.push_back(p4);

	if(df)	*df = mindds;

	return lines;
}

vector<Point2f> UAV_CTR::LMM_fitline(vector<Point2f> conter) {

	vector<Point2f> lin;
	if (conter.size() < 2) {
		lin.push_back(Point(0, 0));
		lin.push_back(Point(0, 0));
		return lin;
	}
		

	Vec4f flin;
	fitLine(conter, flin, CV_DIST_L2, 0, 0.01, 0.01);

	Point2f po = Point2f(flin[2], flin[3]);
	Point2f pn = Point2f(flin[0], flin[1]);
	double m1 = 65535, m2 = -65535;

	for (int ii = 0; ii < conter.size(); ii++) {
		Point2f pp = conter[ii];
		double mm = pn.dot(pp - po);
		m1 = MIN(mm, m1);
		m2 = MAX(mm, m2);
	}
	
	lin.push_back(po + m1*pn);
	lin.push_back(po + m2*pn);
	return lin;
}

bool UAV_CTR::readyTrack() {
	Rect rt = targetBox + rROI;
	Point ctr = Point(rsize.width / 2, rsize.height / 2);
	//if (rt.contains(ctr) && findtimes > 3)
	if ( findtimes > 3)
		return true;
	else
		return false;
}

bool UAV_CTR::trackTarget(Mat ff) {
	if (!istrackTarget)
		trackTarget_init();
	//findTarget();
	trackBox = tracker.update(rframe);

	//Mat temp(rframe.size(), CV_8UC1);
	checkTarget();
	//checkTargetP();
	
	if (nofindtimes > 10) {
		istrackTarget = false;
		trackBox.width = 0;
	}
	return true;
}

bool UAV_CTR::trackTarget_init() {

	Rect rt = targetBox;
	double dw = 0.2*rt.width;
	double dh = 0.2*rt.height;

	dw = MIN(dw, 50);
	dw = MAX(dw, 15);
	dh = MIN(dh, 50);
	dh = MAX(dh, 15);
	double x1 = rt.x + rROI.x - dw;
	double x2 = rt.x + rROI.x + rt.width + dw;
	double y1 = rt.y + rROI.y - dh;
	double y2 = rt.y + rROI.y + rt.height + dh;
	x1 = MAX(0, x1) * scale_tk;
	y1 = MAX(0, y1) * scale_tk;
	x2 = MIN(rframe.cols, x2) ;
	y2 = MIN(rframe.rows, y2) ;
	trackBox = Rect(x1, y1, x2 - x1, y2 - y1);
	tracker.init(trackBox, rframe);
	istrackTarget = true;
	return true;
}

void UAV_CTR::checkTarget() {
	
	Rect rt = trackBox;
	double x1 = rt.x;
	double x2 = rt.x +  rt.width;
	double y1 = rt.y;
	double y2 = rt.y +  rt.height;
	x1 = MAX(0, x1) ;
	y1 = MAX(0, y1) ;
	x2 = MIN(rframe.cols, x2) ;
	y2 = MIN(rframe.rows, y2) ;
	rROI = Rect(x1, y1, x2 - x1, y2 - y1);

//	rROI = Rect(0, 0, rframe.cols, rframe.rows);


	cvtColor(rframe(rROI), gray, CV_RGB2GRAY);
	adaptiveThreshold(gray, gray2, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 33, 3);
	findCandidateEllps(gray2); //--- find candidate ellips

	findInnerTP(gray2);

	double ms = 0;
	double mSa = 0;
	double mDs = 65535;
	Rect in_targetBox;
	vector<Point> in_targetT;
	vector<Point> in_targetContour;
	RotatedRect in_targetEllp;

	for (int ii = 0; ii < tp_ellips.size(); ii++) {
		Rect roic = tp_Rects[ii];
		double sR = roic.area();
		double dS = norm(Point(roic.x + roic.width / 2, roic.y + roic.height / 2) - Point(frame.cols / 2, frame.rows / 2));
		if (mSa < sR && mDs > dS*0.9) {
			mDs = MIN(mDs, dS);
			mSa = MAX(mSa, sR);

			in_targetBox = roic;		//×îÓÅ¾ØÐÎ¿ò				
			in_targetContour = tp_contours[ii];	//×îÓÅTÂÖÀª
			in_targetT = tp_Tlines[ii];	//×îÓÅTÏß¶Î	
			in_targetEllp = tp_ellips[ii];
		}
	}

	if (in_targetBox.height != 0 && in_targetBox.height != 0) {
		targetBox = in_targetBox;		//×îÓÅ¾ØÐÎ¿ò				
		targetContour = in_targetContour;	//×îÓÅTÂÖÀª
		targetT = LMM(in_targetContour, in_targetT); //×îÓÅTÏß¶Î	
		targetEllp = in_targetEllp;
		isfindtarget = true;
		findtimes++;
		diffX = rROI.x + targetBox.x + targetBox.width / 2 - rsize.width / 2;
		diffY = rROI.y + targetBox.y + targetBox.height / 2 - rsize.height / 2;
		nofindtimes = 0;
		targetPose();
	}
	else {
		isfindtarget = false;
		findtimes = 0;
		nofindtimes++;
	}
}

void UAV_CTR::checkCenter(Mat ff) {
	vector<vector<Point>> in_conters;
	vector<Vec4i> in_parents;

	Point pcnt;
	pcnt = Point(trackBox.x + trackBox.width / 2, trackBox.y + trackBox.height / 2) * 2;

	vector<Point> nn;
	Mat3b temp(ff.size());
	temp.setTo(255);
	Mat tmm(ff.size(), CV_8UC1);
	tmm.setTo(0);
	circle(tmm, pcnt, 120, 255, -1);
	double cx, cy;
	cx = 0; cy = 0;

	Mat tmm2(Size(32, 32), CV_8UC1);
	tmm2.setTo(0);
	circle(tmm2, Point(16, 16), 16, 255, -1);
	Mat rroir;

	Mat dst, cdst, dstg;
	ff.copyTo(dst, tmm);

	cvtColor(dst, cdst, CV_GRAY2BGR);
	vector<Vec3f> circles;

	HoughCircles(dst, circles, CV_HOUGH_GRADIENT, 1, 1, 40, 40, 0, 60);
	int minR = 200;
	int mi;
	double maxCorr = 0;

	if (circles.size() > 0) {

		for (size_t i = 0; i < circles.size(); i++)
		{
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);

			Mat rroi = ff(Rect(center.x - radius, center.y - radius, 2 * radius, 2 * radius));
			resize(rroi, rroir, Size(32, 32));
			Mat result;
			matchTemplate(rroir, tmm2, result, TM_CCOEFF_NORMED);
			double corr = result.at<float>(0);
			//if (minR > radius) {
			//	minR = radius;
			//	mi = i;
			//}
			if (maxCorr < corr) {
				maxCorr = corr;
				mi = i;
			}
			//cout << "circle corr:= " << corr << endl;
		}
	}
	if (maxCorr > 0.6) {
		center_cp = Point(cvRound(circles[mi][0]), cvRound(circles[mi][1]));
		center_rd = circles[mi][2];

		//circle(cdst, center_cp, 3, Scalar(0, 255, 0), -1, 8, 0);
		//circle(cdst, center_cp, center_rd, Scalar(0, 0, 255), 3, 8, 0);

		diffX = center_cp.x - rsize.width / 2;
		diffY = center_cp.y - rsize.height / 2;
		targetPose(Rect(center_cp, Size(1, 1)*center_rd * 20));
		isfindcenter = true;
		center_findtimes++;
		center_nofindtimes = 0;
	}
	else {
		diffX = trackBox.x + trackBox.width / 2 - rsize.width / 2;
		diffY = trackBox.y + trackBox.height / 2 - rsize.height / 2;
		//targetPose(trackBox);
		isfindcenter = false;
		center_nofindtimes++;
		center_findtimes = 0;
	}

	//imshow("qwe", cdst);
}
bool UAV_CTR::isReTrack() {
	bool a1 = (center_findtimes > 3);
	Rect rt = trackBox;
	Point ct(rt.x + rt.width / 2, rt.y + rt.height / 2);
	double dd = norm(ct - center_cp);
	bool a2 = (dd > 20);
//	cout <<"track dis:= "<< dd << endl;

	return a1&a2;

	//bool a2 = center_cp ;
}

void UAV_CTR::resetTrackBox() {
	double x1 = MAX(0, center_cp.x - 60);
	double y1 = MAX(0, center_cp.y - 60);
	double x2 = MIN(rframe.cols,center_cp.x + 60);
	double y2 = MIN(rframe.rows,center_cp.y + 60);

	targetBox = Rect(x1, y1, x2 - x1, y2 - y1);
}

bool UAV_CTR::isLoseNTrack() {
	//double s1 = rsize.area();
	//double s2 = trackBox.area();

	//if (s2 / s1 > 0.5)
	//	return true;
	//else
	//	return false;
	return false;
}

vector<Vec4i> UAV_CTR::nonMaxDpress(vector<Vec4i> lines) {
	int N = lines.size();
	vector<Vec4i> lines0;
	vector<Point> mm;

	Point n1,n1t, n2, n3, n4, n5, n6;
	Point p1, p2, p3, p4;
	double d1, d2;

	for (int ii = 0; ii < N; ii++) {
		Vec4i ln = lines[0];
		Point p11(ln[0], ln[1]);
		Point p12(ln[2], ln[3]);
		n1 = p11 - p12;
		n1t = Point(-n1.y, n1.x)/norm(n1);
		mm.clear();
		mm.push_back(p11);
		mm.push_back(p12);

		lines0.clear();
		for (int jj = 1; jj < N; jj++) {
			Vec4i Lm = lines[jj];
			Point p21(Lm[0], Lm[1]);
			Point p22(Lm[2], Lm[3]);
			n2 = p21 - p22;

			d1 = fabs(n1t.dot(p21 - p12));
			d2 = fabs(n1t.dot(p22 - p12));
			double cs = fabs(n1.dot(n2) / norm(n1) / norm(n2));

			if (cs > 0.98 && d1<10 && d2<10) {
				mm.push_back(p21);
				mm.push_back(p22);
			}
			else {
				lines0.push_back(Lm);
			}
		}
		double ds = 0;
		int mjj, mkk;
		for (int jj = 0; jj < mm.size(); jj++) {
			for (int kk = jj + 1; kk < mm.size(); kk++) {
				double dd = norm(mm[jj] - mm[kk]);
				if (ds < dd) {
					ds = dd;
					mjj = jj;
					mkk = kk;
				}
			}
		}
		Vec4i L0(mm[mjj].x, mm[mjj].y, mm[mkk].x, mm[mkk].y);
		lines0.push_back(L0);
		lines = lines0;
		lines0.clear();
	}
	return lines;
}

void UAV_CTR::targetPose() {
	targetPose(targetBox + rROI);
}

void UAV_CTR::targetPose(Rect rt) {
	double pyaw = in_p_yaw;
	double ppch = in_p_pitch;
	double prol = in_p_roll;
	double cyaw = 90;
	double cpch = -90;
	double pposN = p_posN;
	double pposE = p_posE;
	double pposD = p_posD;
	int p_delay = 1;

	//if (p_yaws.size()) {
	//	int kk = MAX(p_yaws.size() - 1 - p_delay, 0);
	//	pyaw = p_yaws[kk];
	//	ppch = p_pitchs[kk];
	//	prol = p_rolls[kk];
	//	pposN = p_posNEs[kk].x;
	//	pposE = p_posNEs[kk].y;
	//	cyaw = 0;
	//	cpch = -90;
	//}
	//tgt.updata(cst, rt, targetT);

	Point gg = targetT[3] - targetT[2];
	Point pc = (targetT[0] + targetT[1]) / 2;
	if (norm(targetT[3] - pc) < norm(targetT[2] - pc))
		gg = -gg;
	t_head = -atan2(gg.x, gg.y) * 180 / CV_PI + pyaw +cyaw;
	while (t_head > 360)t_head -= 360;
	while (t_head < 0)t_head += 360;

	double tag_size = 0.88;
	double fx = cst.fx;
	double fy = cst.fy;
	double px = cst.cx;
	double py = cst.cy;
	//cout << "fxy:= " << fx << " " << fy << "  pxy:= " << px << " " << py << endl;

	std::vector<cv::Point3f> objPts;
	std::vector<cv::Point2f> imgPts;
	double s = tag_size / 2.;
	objPts.push_back(cv::Point3f(-s, s, 0));
	objPts.push_back(cv::Point3f(s, s, 0));
	objPts.push_back(cv::Point3f(s, -s, 0));
	objPts.push_back(cv::Point3f(-s, -s, 0));

	imgPts.push_back(cv::Point2f(rt.x, rt.y));
	imgPts.push_back(cv::Point2f(rt.x + rt.width, rt.y));
	imgPts.push_back(cv::Point2f(rt.x + rt.width, rt.y + rt.height));
	imgPts.push_back(cv::Point2f(rt.x, rt.y + rt.height));

	cv::Mat rvec, tvec;
	cv::Matx33f cameraMatrix(
		fx, 0, px,
		0, fy, py,
		0, 0, 1);
	cv::Vec4f distParam(0, 0, 0, 0); // all 0?
	cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);

	float cx = tvec.at<float>(2);
	float cy = tvec.at<float>(0);
	float cz = tvec.at<float>(1);

	//cout << "cxyz:= " << cx << " " << cy << " " << cz<< endl;

	Mat R0;
	rotateMatR(cyaw, cpch, 0, R0, 0);// 0 == degree
	Mat cX = (Mat_<float>(3, 1) << cx, cy, cz);

	Mat R1, R2;
	rotateMatR(0, p_pitch, p_roll, R1, 0);// 0 == degree
	Mat pX1 = R1*R0*cX;

	t_posX = pX1.at<float>(0);
	t_posY = pX1.at<float>(1);
	t_posZ = pX1.at<float>(2);

	t_height = t_posZ;

	//Mat tX = (Mat_<float>(3, 1) << t_posX, t_posY, t_posZ);
	rotateMatR(p_yaw, 0, 0, R2, 0);// 0 == degree
	Mat pX2 = R2*pX1;

	//cout << "pX2:= " << pX2 << endl;

	t_posN = pX2.at<float>(0)+pposN;
	t_posE = pX2.at<float>(1)+pposE;
	t_posD = pX2.at<float>(2)+pposD;

	//--------  KalmanFiter
	Mat pt = KF.predict();

	Mat measure = (Mat_<float>(2, 1) << t_posN, t_posE);
	Mat ct = KF.correct(measure);

	t_posN_p = ct.at<float>(0);
	t_posE_p = ct.at<float>(1);
	
	CheckStable();

	int t0 = GetTickCount();
	time0.push_back(t0 - T0);
	t_pos_to_UAV.push_back(Vec3f(t_posX, t_posY, t_posZ));
	t_posNEDs.push_back(Vec3f(t_posN, t_posE, t_posD));
	//t_posXs.push_back(t_posX);
	//t_posYs.push_back(t_posY);

	if (time0.size() > 100) {
		time0.erase(time0.begin());
		t_pos_to_UAV.erase(t_pos_to_UAV.begin());
		t_posNEDs.erase(t_posNEDs.begin());
		//t_posXs.erase(t_posXs.begin());
		//t_posYs.erase(t_posYs.begin());
	}
}

void UAV_CTR::pPoseRecord() {
	in_p_pitch = p_pitch2;
	in_p_posD = p_posD2;
	in_p_posE = p_posE2;
	in_p_posN = p_posN2;
	in_p_yaw = p_yaw2;
	in_p_roll = p_roll2;
}
void UAV_CTR::setTagPara(vector<Vec2f> position, int len) {
	tag_position = position;
	tag_total = len;
}
bool UAV_CTR::targetPoseT() {
	static double deff[30][3] = { 
		{0,			0,			0.9},		//0
		{ 0.075,	0.075,		0.09},		//1
		{ -0.225,	0.075,		0.09},		//2
		{ -0.075,  -0.075,		0.09 },	//3
		{ 0.225,   -0.075,		0.09 }		//4
	};
	
	
	double pyaw = in_p_yaw;
	double ppch = in_p_pitch;
	double prol = in_p_roll;
	double cyaw = 90;
	double cpch = -90;
	double pposN = in_p_posN;
	double pposE = in_p_posE;
	double pposD = in_p_posD;
	int p_delay = 1;



	Mat R0, R1, R2;
	rotateMatR(cyaw, cpch, 0, R0, 0);// 0 == degree
	rotateMatR(0, ppch, prol, R1, 0);// 0 == degree
	rotateMatR(pyaw, 0, 0, R2, 0);// 0 == degree
	
	double m_psize = 0.074;
	double m_fx = 348;
	double m_fy = 348;
	double m_px = 320;
	double m_py = 180;

	Mat cX;
	Mat dX;
	double pNs = 0;
	double pEs = 0;
	double pDs = 0;
	int dii = 0;
	int sss = 0;

	for ( dii = 0;dii < detections.size();dii++) {

		if (!detections[dii].good || detections[dii].id>tag_total-1) {
			continue;
		}
		vector<Point2f> ctr;
		for (int kk = 0;kk < 4;kk++) {
			ctr.push_back(Point2f(detections[dii].p[kk].first, detections[dii].p[kk].second));
		}
		double area = contourArea(ctr);
		if (area < 10 * 10)
			continue;
		
	
		double deffx = deff[detections[dii].id][0];
		double deffy = deff[detections[dii].id][1];
		double deffs = deff[detections[dii].id][2];

		Eigen::Matrix4d dd = detections[dii].getRelativeTransform(deffs, m_fx, m_fy, m_px, m_py, deffx, deffy);
		
		double picx = dd(0, 3);// in picture  right = +
		double picy = dd(1, 3);//			  down = +
		double picz = dd(2, 3);	//	

	//	cout << "px:= "<<picx<<"  py:= "<<picy<<"  pz:= "<<picz << endl;

		cX = (Mat_<float>(3, 1) << picz, picx, picy);
		Mat pX1 = R1*R0*cX;

		pX1.at<float>(0) += 0.2;
		Mat pX2 = R2*pX1;

		if (pX2.at<float>(2) < 0.2) {
			continue;
		}
	//	cout << pX1 << endl;
	//	cout << pX2 << endl << endl;
		pNs += (pX2.at<float>(0));
		pEs += (pX2.at<float>(1));
		pDs += pX2.at<float>(2);
		sss++;
	}

	t_head = pyaw;
	if (sss > 0) {
		t_posN = pNs / sss + pposN;
		t_posE = pEs / sss + pposE;
		t_posD = pDs / sss + pposD;
		t_height = pDs / sss;

		dX = (Mat_<float>(3, 1) << pNs / sss, pEs / sss, pDs / sss);
		Mat pX = R2.t()*dX;
		t_posX = pX.at<float>(0);
		t_posY = pX.at<float>(1);
		t_posZ = pX.at<float>(2);
	}
	else {
		nofindtimes++;
		findtimes = 0;
		return false;
	}

	CheckStable();

	//--------  KalmanFiter
	Mat pt = KF.predict();

	Mat measure = (Mat_<float>(2, 1) << t_posN, t_posE);
	Mat ct = KF.correct(measure);

	t_posN_p = ct.at<float>(0);
	t_posE_p = ct.at<float>(1);
	
	int t0 = GetTickCount();
	time0.push_back(t0 - T0);
	t_pos_to_UAV.push_back(Vec3f(t_posX, t_posY, t_posZ));
	t_posNEDs.push_back(Vec3f(t_posN, t_posE, t_posD));
	//t_posXs.push_back(t_posX);
	//t_posYs.push_back(t_posY);

	if (time0.size() > 100) {
		time0.erase(time0.begin());
		t_pos_to_UAV.erase(t_pos_to_UAV.begin());
		t_posNEDs.erase(t_posNEDs.begin());
		//t_posXs.erase(t_posXs.begin());
		//t_posYs.erase(t_posYs.begin());
	}
	static FILE *ffp = fopen("rsdf.txt", "w+");
	static SYSTEMTIME tp;
	if (ffp) {
		GetLocalTime(&tp);
		fprintf(ffp, "%02d %02d %02d %03d  %02d   ", tp.wHour, tp.wMinute, tp.wSecond, tp.wMilliseconds,m_state);
		fprintf(ffp, "%.2lf   %.2lf   %.2lf\n", pNs / sss, pEs / sss, pDs / sss);
		fflush(ffp);
	}
	nofindtimes = 0;
	findtimes ++;
	return true;
}

bool UAV_CTR::in_isRotMat(Mat &R) {
	Mat Rt;
	transpose(R, Rt);
	Mat SB = Rt *R;
	Mat I = Mat::eye(3, 3, R.type());
	return norm(I, SB) < 1e-6;
}
Vec3f UAV_CTR::rotMat2Euler(Mat &R) {
	assert(in_isRotMat(R));
	float sy = sqrt(R.at<float>(0, 0)*R.at<float>(0, 0) + R.at<float>(1, 0)*R.at<float>(1, 0));
	bool singular = sy < 1e-6;

	float x, y, z;
	if (!singular) {
		x = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
		y = atan2(-R.at<float>(2, 0), sy);
		z = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
	}
	else {
		x = atan2(-R.at<float>(1, 2), R.at<float>(1,1));
		y = atan2(-R.at<float>(2, 0), sy);
		z = 0;
	}
	float pp = 1 / CV_PI * 180;
	x *= pp;
	y *= pp;
	z *= pp;
	return Vec3f(x, y, z);
}

bool UAV_CTR::nearTrackTarget_init() {
	Rect rt = targetBox;
	double x1 = center_cp.x - 120;
	double x2 = center_cp.x + 120;
	double y1 = center_cp.y - 120;
	double y2 = center_cp.y + 120;
	x1 = MAX(0, x1);
	y1 = MAX(0, y1);
	x2 = MIN(rsize.width, x2);
	y2 = MIN(rsize.height, y2);

	trackBL = 320.0 / rsize.width;

	trackBox = Rect(x1, y1, x2 - x1, y2 - y1);
	tTrackBox = trackBox * trackBL;
	resize(rframe, tframe, Size(320, 180));
	tracker.init(tTrackBox, tframe);
	istrackTarget = true;
	isNearTrack = true;
	return true;

}

bool UAV_CTR::nearTrackTarget() {
	if(!isNearTrack)
		nearTrackTarget_init();
	static int nn = 0;
	nn++;

	//if (nn % 10 == 0) {
	//	nearTrackTarget_init();
	//}	
	
	resize(rframe, tframe, Size(320, 180));
	tTrackBox = tracker.update(tframe);
	trackBox = tTrackBox*(1 / trackBL);
	isfindtarget = false;
	//checkTarget();
	cvtColor(rframe, gray, CV_RGB2GRAY);
	checkCenter(gray);

	return 1;
}
bool UAV_CTR::isNearTarget() {

	//if ((ld_dist < 3 | targetBox.area()> rframe.rows*rframe.cols /2) & fabs(tgt.angelT) < 5) {
	if (p_height < 2.4 | t_height < 2.4 | targetBox.area() > rframe.rows*rframe.cols / 8) {
		center_cp = Point(targetBox.x + rROI.x + targetBox.width / 2, targetBox.y + rROI.y + targetBox.height / 2);
		isfindtarget = false;
		return true;
	}
	return false;
}

void UAV_CTR::typeChanged() {
	if (m_state != ST_IDLE) {
		isTypeChanged = true;
		isfindtarget = false;
		isfindFar = false;
		isfindcenter = false;
	}		
}

Point2f operator/ (Point2f p1, double div) {
	if (div == 0)
		return Point2f(0, 0);
	else
		return Point2f(p1.x / div, p1.y / div);

}
Rect operator+(Rect tgt, Rect roi) {
	tgt.x += roi.x;
	tgt.y += roi.y;
	return tgt;
}
Rect operator*(Rect tgt, double n) {
	tgt.x *= n;
	tgt.y *= n;
	tgt.width *= n;
	tgt.height *= n;
	return tgt;
}
Size2i operator*(Size2i sz, double nn) {
	int a = round(sz.width*nn);
	int b = round(sz.height*nn);
	return Size2i(a, b);
}
Point2f operator*(Point p1, Point2f ap) {
	return Point2f(p1.x*ap.x, p1.y*ap.y);
}

void rotateMatR(double yaw, double pitch, double roll, Mat& output, int tpye) {
	if (tpye == 0) {
		roll = roll *CV_PI / 180;
		pitch = pitch *CV_PI / 180;
		yaw = yaw *CV_PI / 180;
	}
	double s1 = sin(yaw);
	double s2 = sin(pitch);
	double s3 = sin(roll);
	double c1 = cos(yaw);
	double c2 = cos(pitch);
	double c3 = cos(roll);

	double a11 = c1*c2;
	double a12 = c1*s2*s3 - s1*c3;
	double a13 = c1*s2*c3 + s1*s3;

	double a21 = s1*c2;
	double a22 = s1*s2*s3 + c1*c3;
	double a23 = s1*s2*c3 - c1*s3;

	double a31 = -s2;
	double a32 = c2*s3;
	double a33 = c2*c3;

	Mat A = (Mat_<float>(3, 3) << a11, a12, a13, a21, a22, a23, a31, a32, a33);
	output = A;
}

void UAV_CTR::cacuHeight(double pRoll, double pPitch, double pYaw, double cPitch, double cYaw, double dist) {
	Mat R1, R2;
	rotateMatR(cYaw, cPitch, 0, R1);
	rotateMatR(0, pPitch, pRoll, R2);
	Mat cX = (Mat_<float>(3, 1) << 1, 0, 0);
	Mat pX = R2*R1*cX;
	
	double bl = fabs(dist / (pX.at<float>(2) + 0.0001));
	c_pointX = pX.at<float>(0) * bl;
	c_pointY = pX.at<float>(1) * bl;
	c_pointZ = pX.at<float>(2) * bl;
	ld_dist = bl;

	c_pitch = atan2(-c_pointZ, sqrt(c_pointX*c_pointX + c_pointY*c_pointY)) * 180 / CV_PI;
	c_yaw = atan2(c_pointY, c_pointX) * 180 / CV_PI;

}

void UAV_CTR::show(int para) {
	int t1, t2, t3, t4;
	t1 = GetTickCount();
	if (frame.empty())return;

	sframe = rframe.clone();

	if (isfindFar) {
		//	rectangle(sframe(rROI), targetBox, Scalar(255, 0, 255), 1);	//粉色
		ellipse(sframe, targetEllp, Scalar(255, 0, 0), 2);		//蓝色
		

	}

	if (isfindtarget && m_state < ST_N_TRACK) {

		if (targetT.size() > 0) {
			Point2f ap(targetBox.width / 128.0, targetBox.height / 128.0);
			Point2f p1 = targetT[0] * ap;
			Point2f p2 = targetT[1] * ap;
			Point2f p3 = targetT[2] * ap;
			Point2f p4 = targetT[3] * ap;
			line(sframe(rROI)(targetBox), p1, p2, Scalar(0, 0, 255), 2);	//红色
			line(sframe(rROI)(targetBox), p3, p4, Scalar(0, 255, 255), 2);	//黄色
		}

		rectangle(sframe(rROI), targetBox, Scalar(255, 0, 255), 1);	//粉色
		ellipse(sframe(rROI), targetEllp, Scalar(255, 0, 0), 2);		//蓝色


	}
	if (trackBox.width != 0 && trackBox.height != 0) {
		rectangle(sframe, trackBox, Scalar(255, 255, 0), 4);	//粉色
	}
	//if (m_state >= ST_N_TRACK) {
	//	circle(rframe, Point2f(trackBox.x + trackBox.width / 2, trackBox.y + trackBox.height / 2), 10, Scalar(0, 255, 0), 2);
	//	if (isfindcenter) {
	//		circle(sframe, center_cp, 3, Scalar(0, 255, 0), -1, 8, 0);
	//		circle(sframe, center_cp, center_rd, Scalar(0, 0, 255), 3, 8, 0);
	//	}
	//}




	//t2 = GetTickCount();
	//cout << "=============| st1:= " << (t2 - t1) / 1000.0 << endl;


	//resize(sframe, sframe, Size(640, 360));
	resize(sframe, sframe, Size(1280, 720));
	for (int ii = 0; ii < detections.size(); ii++) {
		//Eigen::Vector3d translation;
		//Eigen::Matrix3d rotation;
		//detections[ii].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py, translation, rotation);
		if (detections[ii].good)
			detections[ii].draw(sframe);
	}

	//imshow("frame", sframe);


	//----------- draw map
	static Mat map1(500, 500, CV_8UC3);
	static Mat map2(500, 500, CV_8UC3);
	//static Mat map(500, 1000, CV_8UC3);
	static bool aa = 1;
	
	drawMap(map1, 0);

	//t3 = GetTickCount();
	//cout << "=============| st2:= " << (t3 - t2) / 1000.0 << endl;

	drawPosMap(map2, 0);
	//hconcat(map1, map2, mMap);
	//vconcat(map1, map2, mMap);


	
	static Mat UI_map1 = UI(Range(0, 500), Range(0, 500));
	static Mat UI_map2 = UI(Range(500, 1000), Range(0, 500));
	static Mat UI_frame = UI(Range(140, 140+720), Range(500, 500+1280));
	map1.copyTo(UI_map1);
	map2.copyTo(UI_map2);
	sframe.copyTo(UI_frame);
	imshow("UI", UI);

	//t4 = GetTickCount();
	//cout << "=============| st3:= " << (t4 - t3) / 1000.0 << endl;
	//cout << "=============| st0:= " << (t4 - t1) / 1000.0 << endl << endl;

	//imshow("map", mMap);
	static int fpp = 0;
	static DWORD tt1 = GetTickCount();
	static DWORD tt2 = GetTickCount();
	fpp++;
	tt2 = GetTickCount();
	if (tt2 - tt1 > 1000) {
		tt1 = tt2;
		fps = fpp;
		fpp = 0;
	}
	//cout << "============!! fps:= " << fps << endl << endl;
	waitKey(10);
}

void UAV_CTR::drawMap(Mat& map, int tpye) {
	map.setTo(Scalar(255, 255, 255));
	double tx = t_posX;
	double ty = t_posY;
	double tz = t_posZ;
	double pitch = cst.pitch;
	double yaw = cst.yaw;
	double scan = cst.aPxHz / CV_PI * 180;
	Point2f nnp(sin(pitch*CV_PI / 180), -cos(pitch*CV_PI / 180));
	Point2f cp(250, 250);
	static int dd = 100;
	dd = 100 - (GetTickCount() % 3500) / 50 ;

	double pyaw = in_p_yaw;
	double ppch = in_p_pitch;
	double prol = in_p_roll;
	double cyaw = 0;
	double cpch = -45;
	

	if (tpye == 0) {
		yaw = c_yaw;
		pitch = c_pitch;
		//// --- camera Scan
		//nnp = Point2f(sin(yaw*CV_PI / 180), -cos(yaw*CV_PI / 180));
		//ellipse(map, Point(250, 250), Size(50, 50), -90, yaw - scan / 2, yaw + scan / 2, Scalar(200, 200, 100), -1);


		Mat R0, R1;
		rotateMatR(cyaw, cpch, 0, R0, 0);// 0 == degree
		rotateMatR(0, ppch, prol, R1, 0);// 0 == degree		


		const double a = tan(44.1 /180* CV_PI);
		const double b = tan(27.0 / 180 * CV_PI);
		const double d = 250 / sqrt(1 + a*a + b*b);

		Point3f P1(d, b*d, a*d);
		Point3f P2(d, b*d, -a*d);
		Point3f P3(d, -b*d, a*d);
		Point3f P4(d, -b*d,-a*d);

		const double b1 = sqrt(1 + a*a + b*b) / sqrt(1 + a*a);
		const double b2 = sqrt(1 + a*a + b*b) / sqrt(1 + b*b);
		Point3f P5(d*b1,0, a*d*b1);
		Point3f P6(d*b1,0, -a*d*b1);
		Point3f P7(d*b2, b*d*b2,0 );
		Point3f P8(d*b2, -b*d*b2,0 );

		vector<Point3f> oPoints;
		vector<Point> ePoints;
		vector<Point> cPoints;
		vector <vector<Point>> ccPoints;
		oPoints.push_back(P1);
		oPoints.push_back(P2);
		oPoints.push_back(P3);
		oPoints.push_back(P4);
		oPoints.push_back(P5);
		oPoints.push_back(P6);
		oPoints.push_back(P7);
		oPoints.push_back(P8);

		Mat ddd(oPoints);
		Mat eee = ddd.reshape(1,8).clone();
		Mat ccc = R1 * R0 * eee.t();
		ePoints.push_back(Point(250, 250));
		for (int ii = 0; ii < ccc.cols; ii++) {
			ePoints.push_back(Point(ccc.at<float>(0,ii), ccc.at<float>(1, ii))+Point(250,250));
		}
		convexHull(ePoints, cPoints);	//---
		//polylines(map, cPoints, true, Scalar(0, 122, 122), 10);
		//fillPoly(map, cPoints, Scalar(50, 50, 0));

		fillConvexPoly(map, cPoints, Scalar(200, 200, 100));

		ccPoints.push_back(cPoints);
		ccPoints.push_back(ePoints);
		fillPoly(map, ccPoints, Scalar(250, 250, 0));






		// --- background
		line(map, Point(250, 0), Point(250, 500), Scalar(20, 20, 20));
		line(map, Point(0, 250), Point(500, 250), Scalar(20, 20, 20));
		for (int ii = 1; ii<6; ii++)
			circle(map, Point(250, 250), 50 * ii, Scalar(40, 40, 40));

		// --- target position
		line(map, cp, cp + ld_dist * 10 * nnp, Scalar(0, 0, 255), 1);//50pix = 5m
		line(map, cp, cp + Point2f(ty, -tx) * 10, Scalar(0, 255, 255), 2);




		//for (int ii = 0;ii <=t_posXs.size();ii++) {
		//	Point2f pp = Point2f(t_posXs[ii], t_posYs[ii]);
		//	circle(map, cp + (Point2f(pp.y, -pp.x)) * 10, 2, Scalar(200, 0, 200), 1);
		//}

		circle(map, cp + Point2f(ty, -tx) * 10, dd / 10, Scalar(0, 0, 255), -1);
	}
	else {
		
	}

	dd--;
	if (dd < 30) dd = 100;
	char msg[100];
	sprintf(msg, "dis:= %.2lf", t_height);
	putText(map, msg, Point(20, 50), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 1);
	sprintf(msg, "dx:=  %.2lf", t_posX);
	putText(map, msg, Point(20, 100), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 1);
	sprintf(msg, "dy:=  %.2lf", t_posY);
	putText(map, msg, Point(20, 150), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 1);
	sprintf(msg, "dz:=  %.2lf", t_posZ);
	putText(map, msg, Point(20, 200), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 1);
	sprintf(msg, "head:=  %.2lf", t_head);
	putText(map, msg, Point(20, 250), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 1);

	sprintf(msg, "cp_yaw:=  %.2lf", c_yaw);
	putText(map, msg, Point(20, 280), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "cp_ptch:=  %.2lf", c_pitch);
	putText(map, msg, Point(20, 300), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);

	sprintf(msg, "find:= %d", findtimes);
	putText(map, msg, Point(20, 320), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "nofind:= %d", nofindtimes);
	putText(map, msg, Point(20, 340), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "stb_cnt:= %d", stb_cont);
	putText(map, msg, Point(20, 360), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);

	switch (m_state)
	{
	case ST_IDLE:
		sprintf(msg, "state:= Idle"); break;
	case 	ST_F_FIND:
		sprintf(msg, "state:= Far"); break;
	case	ST_F_FIND_A:
		sprintf(msg, "state:= Far_A"); break;
	case	ST_FIND:
		sprintf(msg, "state:= Find"); break;
	case	ST_FIND_A:
	case	ST_FIND_B:
	case	ST_FIND_C:
	case	ST_FIND_D:
		sprintf(msg, "state:= Find_D"); break;
	case	ST_TRACK:
	case	ST_TRACK_A:
		sprintf(msg, "state:= Track"); break;
	case	ST_N_TRACK:
	case	ST_N_TRACK_A:
		sprintf(msg, "state:= Near"); break;
	case ST_N_LOSE:
		sprintf(msg, "state:= END"); break;
	default:
		sprintf(msg, "state:= No state"); break;
	}
	putText(map, msg, Point(20, 450), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 2);

	static int fpp = 0;
	static DWORD t1 = GetTickCount();
	static DWORD t2 = GetTickCount();

	fpp++;
	t2 = GetTickCount();
	if (t2 - t1 > 1000) {
		t1 = t2;
		fps = fpp;
		fpp = 0;
	}
	sprintf(msg, "fps:=  %d", fps);
	putText(map, msg, Point(400, 30), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "tgt_wid:=  %d", targetBox.width);
	putText(map, msg, Point(350, 70), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "tgt_hei:=  %d", targetBox.height);
	putText(map, msg, Point(350, 90), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "diffX:=  %d", diffX);
	putText(map, msg, Point(350, 110), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "diffY:=  %d", diffY);
	putText(map, msg, Point(350, 130), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);


	sprintf(msg, "high:= %.2lf", p_height);
	putText(map, msg, Point(300, 240), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "cm_yaw:=  %.2lf", cst.yaw);
	putText(map, msg, Point(300, 260), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "cm_ptch:=  %.2lf", cst.pitch);
	putText(map, msg, Point(300, 280), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "pl_roll:=  %.2lf", p_roll);
	putText(map, msg, Point(300, 300), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "pl_yaw:=  %.2lf", p_yaw);
	putText(map, msg, Point(300, 320), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "pl_ptch:=  %.2lf", p_pitch);
	putText(map, msg, Point(300, 340), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);

	if (isfindFar | isfindtarget | isfindcenter) {
		circle(map, Point(460, 460), 15, Scalar(0, 255, 0), -1);
	}
	else {
		circle(map, Point(460, 460), 15, Scalar(0, 0, 255), -1);
	}
	if (isStable) {
		circle(map, Point(460, 460), 10, Scalar(255, 100, 100), -1);
	}
}
void UAV_CTR::drawPosMap(Mat& map, int tpye) {
	static Point2f pc(250, 250);
	map.setTo(Scalar(255, 255, 255));
	int para = 25;
	for (int ii = 0;ii < 500;ii += 50) {
		int tk = (ii == 250) ? 2 : 1;
		line(map, Point(ii, 0), Point(ii, 500), Scalar(60, 60, 60), tk);
		line(map, Point(0, ii), Point(500, ii), Scalar(60, 60, 60), tk);
	}
	putText(map, "N", Point(250, 20), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 2);
	putText(map, "E", Point(480, 250), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 2);
	putText(map, "S", Point(250, 500), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 2);
	putText(map, "W", Point(0, 250), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 2);

	char msg[108];
	sprintf(msg, "N0:= %.2lf", p_posN0);
	putText(map, msg, Point(0, 20), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "E0:= %.2lf", p_posE0);
	putText(map, msg, Point(0, 40), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);

	sprintf(msg, "pposN:= %.2lf", p_posN);
	putText(map, msg, Point(0, 70), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "pposE:= %.2lf", p_posE);
	putText(map, msg, Point(0, 90), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "pposD:= %.2lf", p_posD);
	putText(map, msg, Point(0, 110), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);

	sprintf(msg, "dposN:= %.2lf", t_posN - p_posN);
	putText(map, msg, Point(20, 130), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "dposE:= %.2lf", t_posE - p_posE);
	putText(map, msg, Point(20, 150), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);

	sprintf(msg, "tposN:= %.2lf", t_posN);
	putText(map, msg, Point(350, 70), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "tposE:= %.2lf", t_posE);
	putText(map, msg, Point(350, 90), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "tposD:= %.2lf", t_posD);
	putText(map, msg, Point(350, 110), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);

	sprintf(msg, "grid:= %.2lfm", 50.0/para);
	putText(map, msg, Point(350, 480), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);

	sprintf(msg, "spdN:= %.2lf", t_spdN);
	putText(map, msg, Point(0, 190), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	sprintf(msg, "spdE:= %.2lf", t_spdE);
	putText(map, msg, Point(0, 210), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
	static int dd = 100;
	dd = 100 - (GetTickCount() % 3500) / 50;
	if (time0.size() == 0)
		return;
	int t0;
	for (int ii = time0.size() - 1;ii >= 0;ii--) {
		Vec3f pp1 = t_posNEDs[ii];
		Point2f cnt(p_posE, -p_posN);
		Point2f pnd(pp1[1], -pp1[0]);
		if (ii == time0.size() - 1) {
			t0 = time0[ii];
			circle(map, pc + (pnd - cnt) * para, dd / 10, Scalar(0, 0, 255), -1);
		}
		else if (t0 - time0[ii] > 5000) {
			break;
		}
		else {
			Vec3f pp2 = t_posNEDs[ii + 1];
			Point2f pnd2(pp2[1], -pp2[0]);
			line(map, pc + (pnd - cnt) * para, pc + (pnd2 - cnt) * para, Scalar(255, 0, 0), 1);
			circle(map, pc + (pnd - cnt) * para, 2, Scalar(0, 0, 200), -1);
		}
	}

	for (int ii = 0;ii <p_posNEs.size();ii++) {
		Point2f pp = p_posNEs[ii];
		circle(map, pc + (Point2f(pp.y, -pp.x) - Point2f(p_posE, -p_posN)) * para, 3, Scalar(200, 0, 200), 1);
	}
	circle(map, pc, dd / 8, Scalar(255, 0, 255), -1);
}

TargetRecord::TargetRecord() {

}
TargetRecord::~TargetRecord() {

}

//typedef unsigned char u8; 
//typedef unsigned short u16; 
//typedef unsigned int u32; 
//typedef unsigned long u64;
////Atention at64bitmachineunsignedintis64bitwidth typedefunsignedchar u8; typedefunsignedshortu16; typedefunsignedint u32; typedefunsignedlong u64;
//typedef enum uCRC_Polynomial{ 
//	CRC4_ITU = 0x03,
//	CRC5_EPC = 0X09
//}Crc_Polynomial;
//typedef struct uCRC_Parameter{ 
//	u8 BitWidth; //4,5,6,7,8,16,32oryoudefine 
//	u32 Poly; //Polynomial with BitWidth 
//	u32 Init_Value; //BitWidthbit"0"or"1" 
//	u8 Refin_Bool; //sameTimeRefin=Refout TrueOrFalse 
//	u8 Refout_Bool; u32 XorOut; //Beforelastverifydatatoputout,shuldXORwithXorOut 
//	u32 ValidBit; //BitWidthbit1 
//}CRC_PARAM;
//static unsigned int CRC_table[256]; 
//static unsigned int CRC_tableTT[256];
////Î»Äæ×ª 
//static unsigned int mf_Bitrev(u32 input,u8 bitwidth) { 
//	u8 i; 
//	u32 var; 
//	var=0;
//	for (i = 0; i<bitwidth; i++) { 
//		if (input & 0x01) { 
//			var |= 1 << (bitwidth - 1 - i); 
//		} 
//		input >>= 1; 
//	} 
//	return var;
//}
//void CRC_CreateTable(CRC_PARAM uCRC_PARAM) {
//	u16 i; 
//	u8 j; 
//	u32 tmp_poly; 
//	u32 tmp_CrcReg; 
//	u32 tmp_MaxBit;
//	tmp_poly = uCRC_PARAM.Poly; 
//	tmp_MaxBit = (1 << (uCRC_PARAM.BitWidth - 1));
//	if (1 == uCRC_PARAM.Refin_Bool) { 
//		tmp_poly = mf_Bitrev(tmp_poly, uCRC_PARAM.BitWidth); 
//		for (i = 0; i<256; i++) { 
//			tmp_CrcReg = i; 
//			for (j = 0; j<8; j++) { 
//				if ((tmp_CrcReg & 1) != 0) { 
//					tmp_CrcReg = tmp_poly ^ (tmp_CrcReg >> 1); 
//				} else { 
//					tmp_CrcReg = (tmp_CrcReg >> 1); } 
//			} 
//			CRC_table[i] = (uCRC_PARAM.ValidBit&tmp_CrcReg); 
//		} 
//	}
//	else {
//		if (uCRC_PARAM.BitWidth<8) { 
//			tmp_poly = uCRC_PARAM.Poly << (8 - uCRC_PARAM.BitWidth); 
//			tmp_MaxBit = (1 << 7); 
//		}
//		for (i = 0; i<256; i++) { 
//			tmp_CrcReg = (i << uCRC_PARAM.BitWidth - 8); 
//			if (uCRC_PARAM.BitWidth<8) { 
//				tmp_CrcReg = (i << 0); 
//			} 
//			for (j = 0; j<8; j++) { 
//				if ((tmp_CrcReg&tmp_MaxBit) != 0) { 
//					tmp_CrcReg = (tmp_poly ^ (tmp_CrcReg << 1)); 
//				} else { 
//					tmp_CrcReg = (tmp_CrcReg << 1); 
//				} 
//			}
//			CRC_table[i] = (uCRC_PARAM.ValidBit&tmp_CrcReg); 
//		}
//	}
//}
////¼ÆËã CRC 
//u32 CRC_Calculate(CRC_PARAM uCRC_PARAM,u8 *uInput,u16 len) { 
//	int i; 
//	u8 index; 
//	u32 tmp_crc;
//	u8 tmp_crcHB;
//	tmp_crc = uCRC_PARAM.Init_Value;
//	if (1 == uCRC_PARAM.Refin_Bool) {
//		for (i = 0; i<len; i++) { 
//			index = (uCRC_PARAM.ValidBit&(uInput[i] ^ tmp_crc)); 
//			tmp_crc = (uCRC_PARAM.ValidBit&((tmp_crc >> 8) ^ CRC_table[index])); 
//		} //tmp_crc=mf_Bitrev(tmp_crc,uCRC_PARAM.BitWidth); 
//	}
//	else {
//		if (uCRC_PARAM.BitWidth > 8) {
//			for (i = 0; i < len; i++) {
//				tmp_crcHB = (tmp_crc >> (uCRC_PARAM.BitWidth - 8));
//				tmp_crc = (tmp_crc << 8);
//				tmp_crc = (uCRC_PARAM.ValidBit&(tmp_crc ^CRC_table[tmp_crcHB^uInput[i]]));
//			}
//		}
//		else {
//			if (uCRC_PARAM.BitWidth < 8) {
//				tmp_crc = (tmp_crc << (8 - uCRC_PARAM.BitWidth));
//			}
//			for (i = 0; i < len; i++) {
//				tmp_crc = CRC_table[tmp_crc^uInput[i]];
//			}
//		}
//	}
//	 if ((uCRC_PARAM.BitWidth<8) && (uCRC_PARAM.Refin_Bool == 0)) { 
//		 return(tmp_crc >> (8 - uCRC_PARAM.BitWidth)) ^ uCRC_PARAM.XorOut; 
//	 }else { 
//		 return tmp_crc^uCRC_PARAM.XorOut; 
//	 }
//}
//
//int mainc() {
//	int i; 
//	unsigned int res;
//	CRC_PARAM tmp_CRC_Param;
//	//unsignedcharTestData[]={0xAA,0xAA,0x06,0x01,0x01,0x03,0xFB,0xEC,0x88,0x24,0xEA,0XEF,0X3A}; 
//	unsigned char TestData[]={0xAA,0xAA,0x06,0x01,0x01,0x03,0xFB,0xEC,0x88,0x99};
//	tmp_CRC_Param.BitWidth = 16; 
//	tmp_CRC_Param.Poly = 0x8005; 
//	tmp_CRC_Param.Init_Value = 0x00; 
//	tmp_CRC_Param.Refin_Bool = 1; 
//	tmp_CRC_Param.Refout_Bool = 0; 
//	tmp_CRC_Param.XorOut = 0x00; 
//	tmp_CRC_Param.ValidBit = 0xFFFF;
//	CRC_CreateTable(tmp_CRC_Param); 
//	
//	for (i = 0; i<256; i++) { 
//		if (i % 8 == 0) { 
//			printf("\n"); 
//		} 
//		printf("0x%08X", CRC_table[i]); 
//	}
//	printf("\n");
//	res = CRC_Calculate(tmp_CRC_Param, TestData, 9);
//	printf("\nCRCResult=%08X!\n", res);
//	printf("Helloworld!\n"); 
//	
//	return 0;
//}
