/*
 * CoSLAM.cpp
 *
 *  Created on: 2011-1-3
 *      Author: Danping Zou
 */
#include "SL_CoSLAM.h"
#include "SL_InitMap.h"
#include "SL_NewMapPointsInterCam.h"
#include "SL_GlobParam.h"

#include "slam/SL_CoSLAMHelper.h"
#include "SL_MergeCameraGroup.h"
#include "gui/MyApp.h"

#include "tools/SL_Tictoc.h"
#include "tools/GUI_ImageViewer.h"
#include "tools/SL_AVIReader.h"

#include "geometry/SL_Triangulate.h"
#include "geometry/SL_Geometry.h"
#include "imgproc/SL_ImageIO.h"
#include "calibration/SL_CalibTwoCam.h"
#include <cassert>
#include <set>
#include <cstdio>
#include <sstream>

void enterBACriticalSection() {
	pthread_mutex_lock(&MyApp::s_mutexBA);
}
void leaveBACriticalSection() {
	pthread_mutex_unlock(&MyApp::s_mutexBA);
}

CoSLAM* CoSLAM::ptr = 0;
CoSLAM::CoSLAM():m_nInitFrame(0), m_nFirstFrame(0), numCams(0), curFrame(
				0), numActFrm(250), m_minViewAngleChange(5.0), m_initCamTranslation(
				0.0), m_minCamTranslation(0.0), m_mappedPtsReduceRatio(0.93), m_lastFrmIntraMapping(
				0), m_lastFrmInterMapping(0), m_firstFrmBundle(0), m_lastFrmBundle(
				0), m_lastFrmGroupMerge(-1), m_lastReleaseFrm(-1), m_mergedgid(
				-1), m_nStatic(0), m_nDynamic(0), m_easyToFail(false), m_nKeyFrame(
				0) {

	memset(viewOverlapCost, 0, sizeof(double) * SLAM_MAX_NUM * SLAM_MAX_NUM);
	memset(m_nDynamicFeat, 0, sizeof(int) * SLAM_MAX_NUM);
	memset(m_nStaticFeat, 0, sizeof(int) * SLAM_MAX_NUM);
	memset(m_groupId, 0, sizeof(int) * SLAM_MAX_NUM);

	CoSLAM::ptr = this;
}
CoSLAM::~CoSLAM() {
	//clear the queued BAs
	while (!m_requestBAs.empty()) {
		//start of critical section
		enterBACriticalSection();
		RobustBundleRTS* pBA = m_requestBAs.back();
		m_requestBAs.pop_back();
		leaveBACriticalSection();
		//end of critical section
		delete pBA;
	}
}
void CoSLAM::addInput(const char* videoFilePath, const char* calFilePath, const char* odoFilePath,
		int startFrame,int initFrm) {
	slam[numCams].videoFilePath = videoFilePath;
	slam[numCams].calFilePath = calFilePath;
	slam[numCams].startFrameInVideo = startFrame;
	slam[numCams].odoFilePath = odoFilePath;
	slam[numCams].nInitFrm = initFrm;
	slam[numCams].camId = numCams;
	numCams++;
}

void CoSLAM::init() {
    int minFrmNum = std::numeric_limits<int>::max();
	for (int i = 0; i < numCams; i++) {
		((AVIReader*)slam[i].videoReader)->filePath =slam[i].videoFilePath; 
		slam[i].videoReader->open();
		slam[i].videoReader->skip(slam[i].startFrameInVideo);
		int nfrm = slam[i].videoReader->getTotalFrame();
		if (minFrmNum > nfrm - slam[i].startFrameInVideo)
			minFrmNum = nfrm - slam[i].startFrameInVideo;

		slam[i].videoReader->grabFrame();

		//////////////////////////////////////////////////////////
		slam[i].m_rgb.resize(slam[i].videoReader->_w, slam[i].videoReader->_h);
		slam[i].m_img.resize(slam[i].videoReader->_w, slam[i].videoReader->_h);

		slam[i].K.resize(3, 3);
		slam[i].k_c.resize(5, 1);
		readIntrinDistParam(slam[i].calFilePath.c_str(), slam[i].K,
				slam[i].k_c);
		slam[i].iK.resize(3, 3);
		slam[i].k_ud.resize(7, 1);

		matInv(3, slam[i].K.data, slam[i].iK.data);
		invDistorParam(slam[i].videoReader->_w, slam[i].videoReader->_h,
				slam[i].iK.data, slam[i].k_c.data, slam[i].k_ud.data);
	}
	Param::nTotalFrame = minFrmNum;

	//initialize the camera groups
	m_groupNum = 1;
	for (int i = 0; i < numCams; i++) {
		m_groups[0].addCam(i);
	}
}

void CoSLAM::readFrame() {
	for (int i = 0; i < numCams; i++) {
		slam[i].readFirstFrame();
	}
}
void* _parallelReadNextFrame(void* param) {
	int camId = (intptr_t) param;
	CoSLAM::ptr->slam[camId].grabReadFrame();
	return 0;
}
void CoSLAM::grabReadFrame() {
	TimeMeasurer tm;
	tm.tic();

	pthread_t threads[SLAM_MAX_NUM];
	for (int i = 1; i < numCams; i++) {
		pthread_create(&threads[i], 0, _parallelReadNextFrame, (void*) i);
	}
	slam[0].grabReadFrame();
	for (int i = 1; i < numCams; i++) {
		pthread_join(threads[i], 0);
	}
	curFrame++;
	m_tmReadFrame = tm.toc();
}
void CoSLAM::initMap() {
	if (numCams == 1)
		return initMapSingleCam();
	return initMapMultiCam();
}

void CoSLAM::initMapSingleCam() {
	//1.tracking feature points for a set of frames
	int nFeatPts = slam[0].initTracker(curFrame);
	if (nFeatPts <= 0)
		repErr("not feature point is detected at frame %d.",
				getCurFrameInVideo(0));
	slam[0].grabReadFrame();
    for (int i = 0; i < slam[0].startFrameInVideo; i++) {
		nFeatPts = slam[0].trackFeaturePoints();
		if (nFeatPts <= 0)
			repErr("not feature point is detected at frame %d.",
					getCurFrameInVideo(0));
		slam[0].grabReadFrame();
		curFrame++;
	}

	m_nFirstFrame = curFrame;
	vector<FeaturePoint*> vecPts1, vecPts2;
	for (int i = 0; i < slam[0].m_tracker.m_nMaxCorners; i++) {
		Track2D& tk = slam[0].m_tracker.m_tks[i];
		if (tk.empty())
			continue;
		if (tk.f1 == 0 && tk.f2 == curFrame) {
			vecPts2.push_back(tk.tail->pt);
			vecPts1.push_back(tk.head.next->pt);
		}
	}

	//2.estimate the initial camera poses
	Mat_d matPts1, matPts2;
	featPoint2Mat(vecPts1, matPts1);
	featPoint2Mat(vecPts2, matPts2);

	CalibTwoCam calib;
	calib.setIntrinParam(slam[0].K.data, slam[0].K.data);
	calib.setDistorParam(slam[0].W, slam[0].H, slam[0].k_c, slam[0].k_c);
	calib.setMatchedPoints(matPts1, matPts2);
	calib.estimateEMat();

	vector<int> inlierInd;
	calib.getInlierInd(inlierInd);

	Mat_d R1, t1, R2, t2, pts1, pts2;
	calib.outputInlierNormPoints(pts1, pts2);
	calib.outputRTs(R1, t1, R2, t2);

	int npts = pts1.m;

	//3. triangulate initial map points
	Mat_d Ms(npts, 3);
	binTriangulatePoints(R1, t1, R2, t2, npts, pts1.data, pts2.data, Ms.data);

	for (int i = 0; i < npts; i++) {
		MapPoint* pM = curMapPts.add(Ms.data + 3 * i, curFrame);
		getBinTriangulateCovMat(slam[0].K, R1, t1, slam[0].K, R2, t2, pM->M,
				pM->cov, Const::PIXEL_ERR_VAR);

		int featId = inlierInd[i];
		pM->addFeature(0, vecPts2[featId]);
		for (FeaturePoint* pm = vecPts2[featId]; pm; pm = pm->preFrame)
			pm->mpt = pM;
		pM->nccBlks[0].computeScaled(slam[0].m_smallImg, slam[0].m_smallScale,
				vecPts2[featId]->m[0], vecPts2[featId]->m[1]);
	}


	slam[0].m_camPos.add(curFrame, 0, R2.data, t2.data);
	slam[0].initTracker(curFrame);
	m_minCamTranslation = getCameraDistance(R1, t1, R2, t2) * 0.1;

	//4. add key frame
	//add the first frame as a key frame
	KeyFrame* keyFrame = m_keyFrms.add(curFrame);
	keyFrame->setCamNum(numCams);
	keyFrame->setMapPtsNum(curMapPts.getNum());
	keyFrame->setCamGroups(m_groups, m_groupNum);
	keyFrame->setKeyPose(0, slam[0].addKeyPose(true));

	m_nKeyFrame++;
}

//To be called when the SLAM fails and is needed to restart
void CoSLAM::clearDataPoints()
{
  for(int i = 0; i < numCams; i++)
    {
      //slam[i].m_keyPose.clear();
      slam[i].m_featPts.clear();
      //slam[i].m_camPos.clear();
    }
}

//After calling the clearDataPoints() function, reinit the containers so they are not null
void CoSLAM::initDataPoints()
{
  for(int i = 0; i < numCams; i++)
    {
      slam[i].m_keyPose;
    }
}

void CoSLAM::initMapMultiCam() {
	InitMap initMapper;
	vector<FeaturePoints*> pFeatPts;
	for (int i = 0; i < numCams; i++) {
		initMapper.addCam(slam[i]);
		pFeatPts.push_back(&slam[i].m_featPts);
	}
    //generate 3D map points 
	initMapper.apply(curFrame, pFeatPts, curMapPts);

	cout << "The initial map has been saved!" << endl;

	updateDisplayData();

	//copy the camera orders
	m_groups[0].setCams(initMapper.camOrder);
	m_groupNum = 1;

	RobustBundleRTS bundler;

	//add the first frame as a key frame
	KeyFrame* keyFrame = m_keyFrms.add(curFrame);
	keyFrame->setCamNum(numCams);
	keyFrame->setMapPtsNum(curMapPts.getNum());
	keyFrame->setCamGroups(m_groups, m_groupNum);

	m_nKeyFrame++;

	for (int i = 0; i < numCams; i++) {
		slam[i].getNumMappedStaticPts();
		CamPoseItem* pCamPos = slam[i].m_camPos.add(curFrame, slam[i].camId,
				initMapper.camR[i], initMapper.camT[i]);

		keyFrame->setKeyPose(i, slam[i].addKeyPose(true));
		//for bundle adjustment
		bundler.addKeyCamera(slam[i].K.data, pCamPos);

		//initialize the GPUKLT tracker
		vector<FeaturePoint*> existFeatPts;
		slam[i].m_featPts.getFrame(curFrame, existFeatPts);
		slam[i].initTracker(curFrame, existFeatPts);
	}

	//set the 3D and 2D correspondences for bundle adjustment
	for (int i = 0; i < numCams; i++) {
		FeaturePoint* pFeatHead = slam[i].m_featPts.hd.next;
		FeaturePoint* pFeatTail = slam[i].m_featPts.tail;
		for (FeaturePoint* p = pFeatHead; p != pFeatTail; p = p->next) {
			if (p->mpt && p->mpt->isLocalStatic() && p->mpt->numVisCam > 1)
				bundler.addCorrespondingPoint(p->mpt, p);
		}
	}

	//call bundle adjustment to refine the initial camera poses
	bundler.setCoSLAM(this);
	bundler.apply(2, 1, 5, 50);


	//compute average distance between cameras
	m_initCamTranslation = 0;
	int n = 0;
	for (int i = 0; i < numCams; i++) {
		CamPoseItem* pCamPos1 = slam[i].m_camPos.current();
		for (int j = i + 1; j < numCams; j++) {
			CamPoseItem* pCamPos2 = slam[j].m_camPos.current();
			m_initCamTranslation += getCamDist(pCamPos1, pCamPos2);
			n++;
		}
	}
	m_initCamTranslation /= n;
	m_minCamTranslation = m_initCamTranslation / 4.5;

	for (int i = 0; i < numCams; i++) {
		slam[i].updateCamParamForFeatPts(slam[i].K.data,
				slam[i].m_camPos.current());
	}
	m_nFirstFrame = curFrame;
}
void CoSLAM::featureTracking() {
	TimeMeasurer tm;
	tm.tic();
	for (int i = 0; i < numCams; i++)
		slam[i].trackFeaturePoints();
	m_tmFeatureTracking = tm.toc();
}

#include "SL_InterCamPoseEstimator.h"
bool CoSLAM::interCamPoseUpdate() {
	const int minStaticNum = 40;
	vector<Track2DNode*> nodes[SLAM_MAX_NUM];

	m_easyToFail = false;
	for (int i = 0; i < numCams; i++) {
		m_intraCamPoseUpdateFail[i] = 0;
	}
	for (int i = 0; i < numCams; i++) {
		int num = slam[i].getStaticMappedTrackNodes(nodes[i]);
		if (num < minStaticNum) {
			//if the number of static feature points in this view is small 
			m_easyToFail = true;
			m_intraCamPoseUpdateFail[i] = 2;
		}
		if (m_intraCamPoseUpdateFail[i] == false) {
			//check their bounding box
			double minX = 1e+6;
			double minY = 1e+6;
			double maxX = 0;
			double maxY = 0;
			for (size_t k = 0; k < nodes[i].size(); k++) {
				double x = nodes[i][k]->x;
				double y = nodes[i][k]->y;
				if (x < minX)
					minX = x;
				if (y < minY)
					minY = y;
				if (x > maxX)
					maxX = x;
				if (y > maxY)
					maxY = y;
			}
			if ((maxX - minX) < slam[i].m_img.w * 0.25
					|| (maxY - minY) < slam[i].m_img.h * 0.25) {
				m_easyToFail = true;
				m_intraCamPoseUpdateFail[i] = 1;
			}
		}
	}
	if (!m_easyToFail)
		return false;

	for (int i = 0; i < numCams; i++) {
		slam[i].propagateFeatureStates();
	}
	//inter-camera pose update
	InterCamPoseEstimator poseEstimator;
	poseEstimator.setCoSLAM(this);
	poseEstimator.addMapPoints();
	poseEstimator.apply();

	for (int i = 0; i < numCams; i++) {
		slam[i].detectDynamicFeaturePoints(20, 5, 3, Const::MAX_EPI_ERR);
	}

	return true;
}
void CoSLAM::poseUpdate() {
	TimeMeasurer tm;
	tm.tic();
	enterBACriticalSection();

	if (curFrame >= m_lastFrmGroupMerge && curFrame < m_lastFrmGroupMerge + 60)
		parallelPoseUpdate(true);
	else
		parallelPoseUpdate(false);

	leaveBACriticalSection();
	m_tmPoseUpdate = tm.toc();

	tm.tic();
	enterBACriticalSection();
	mapStateUpdate();
	leaveBACriticalSection();

	enterBACriticalSection();
	
	logInfo("CLASSIFYING POINTS\n");

	mapPointsClassify(12.0);
	leaveBACriticalSection();
	m_tmMapClassify = tm.toc();
}

void* _parallelPoseUpdate(void* param) {
	CoSLAM* pSLAM = CoSLAM::ptr;
	int camId = (intptr_t) param;
	if (pSLAM->slam[camId].poseUpdate3D(false) < 0)
		return (void*) -1;
	pSLAM->slam[camId].detectDynamicFeaturePoints(20, 5, 3, Const::MAX_EPI_ERR);
	return 0;
}
void CoSLAM::parallelPoseUpdate(bool largeErr) {
	if (numCams == 1) {
		slam[0].poseUpdate3D(largeErr);
		slam[0].detectDynamicFeaturePoints(20, 5, 3, Const::MAX_EPI_ERR);
		return;
	}
	for (int i = 0; i < numCams; i++) {
		slam[i].poseUpdate3D(largeErr);
		slam[i].detectDynamicFeaturePoints(20, 5, 3,
				largeErr ? Const::MAX_EPI_ERR * 5 : Const::MAX_EPI_ERR);
	}

#ifdef USE_OPENMP
#pragma omp parallel for num_threads(SLAM_CPU_CORE_NUM)
#endif
	for (int i = 0; i < numCams; i++) {
		CamPoseItem* camPos = slam[i].m_camPos.current();
		slam[i].updateCamParamForFeatPts(slam[i].K.data, camPos);
	}
}
void CoSLAM::mapPointsClassify(double pixelVar) {
	MapPoint* pCurMapHead = curMapPts.getHead();
	if (!pCurMapHead)
		repErr("no map point is found");

	const int FRAME_NUM_FOR_NEWPOINT = 30;
	const int FRAME_NUM_FOR_DONTMOVE = 50;
	const int NUM_FRAME_CHECK_STATIC = 60;

	int numFalse = 0;
	for (MapPoint* p = pCurMapHead; p; p = p->next) {
		if (p->isUncertain() || p->isLocalDynamic()) {
			//check if the map point is a false point or a dynamic point
			if (p->numVisCam == 1) {
				p->setFalse();
				numFalse++;
				continue;
			}
			double M[3], cov[9];
			//check if the map point (with either 'uncertain' or 'dynamic' flag) is a false map point
			//triangulateMultiView(numViews, Rs, Ts, nms, M, 0);
			if (p->isUncertain()) {
				if (p->bNewPt) {
					//dynamic or static
					bool isStatic = isStaticPoint(numCams, p, pixelVar, M, cov,
							NUM_FRAME_CHECK_STATIC);
					if (isStatic) {
						if (p->lastFrame - p->firstFrame
								> FRAME_NUM_FOR_NEWPOINT) {
							p->setLocalStatic();
							p->bNewPt = false;
							p->updatePosition(M, cov);
						}
						//continue; - keep checking 
					} else {
						if (isDynamicPoint(numCams, p, pixelVar, M, cov)) {
							p->setLocalDynamic();
							p->updatePosition(M, cov);
							p->bNewPt = false;
						} else {
							p->setFalse();
						}
					}
				} else {
					//dynamic or false
					if (isDynamicPoint(numCams, p, pixelVar, M, cov)) {
						p->setLocalDynamic();
						p->updatePosition(M, cov);
					} else {
						int outlierViewId = isStaticRemovable(numCams, p,
								pixelVar, M, cov, NUM_FRAME_CHECK_STATIC);
						if (outlierViewId >= 0) {
							FeaturePoint* fp = p->pFeatures[outlierViewId];
							fp->mpt = 0;
							p->pFeatures[outlierViewId] = 0;
							p->numVisCam--;
							p->setLocalStatic();
							p->updatePosition(M, cov);
						} else {
							p->setFalse();
						}
					}
				}
			} else if (p->isLocalDynamic()) {
				//can return to static or be a false point
				if (isDynamicPoint(numCams, p, pixelVar, M, cov)) {
					if (isLittleMove(numCams, p, pixelVar, M, cov)) {
						p->staticFrameNum++;
						if (p->staticFrameNum > FRAME_NUM_FOR_DONTMOVE) {
							double M0[3], cov0[9];
							if (isStaticPoint(numCams, p, pixelVar, M0, cov0,
									NUM_FRAME_CHECK_STATIC)) {
								//go back to static points
								p->setLocalStatic();
								for (int i = 0; i < numCams; i++) {
									if (p->pFeatures[i])
										p->pFeatures[i]->type =
												TYPE_FEATPOINT_STATIC;
								}
								p->updatePosition(M0, cov0);
							} else {
								p->staticFrameNum = 0;
								p->updatePosition(M, cov);
							}
						} else
							p->updatePosition(M, cov);
					} else {
						p->staticFrameNum = 0;
						p->updatePosition(M, cov);
					}
					p->updatePosition(M, cov);
				} else
					p->setFalse();
			}
		}
	}
}



void CoSLAM::mapPointsClassifyNoDynamic(double pixelVar) {
    //std::cout << "enter mapClass:" << std::endl;
    //std::cout << &curMapPts << std::endl;

    //if(allbad)
    //    curMapPts.clearWithoutRelease();

    //std::cout << "num of mappts:" << curMapPts.getNum();
    MapPoint* pCurMapHead = curMapPts.getHead();
    //if (!pCurMapHead)
    //    repErr("no map point is found");


    const int FRAME_NUM_FOR_NEWPOINT = 30;
    const int FRAME_NUM_FOR_DONTMOVE = 50;
    const int NUM_FRAME_CHECK_STATIC = 60;

    int numFalse = 0;
    int k = 0;
    for (MapPoint* p = pCurMapHead; p; p = p->next) {
        k ++ ;
        std::cout << "k:" << k << std::endl;
        if (p->isUncertain() || p->isLocalDynamic()) {
            //check if the map point is a false point or a dynamic point
            if (p->numVisCam == 1) {
                p->setFalse();
                numFalse++;
                continue;
            }
            double M[3], cov[9];
            //check if the map point (with either 'uncertain' or 'dynamic' flag) is a false map point
            //triangulateMultiView(numViews, Rs, Ts, nms, M, 0);
            if (p->isUncertain()) {
                if (p->bNewPt) {
                    //dynamic or static
                    bool isStatic = isStaticPoint(numCams, p, pixelVar, M, cov,
                            NUM_FRAME_CHECK_STATIC);
                    if (isStatic) {
                        if (p->lastFrame - p->firstFrame
                                > FRAME_NUM_FOR_NEWPOINT) {
                            p->setLocalStatic();
                            p->bNewPt = false;
                            p->updatePosition(M, cov);
                        }
                        //continue; - keep checking
                    } else {
                        //simple treat the dynamic points as outliers
                        p->setFalse();
//                        if (isDynamicPoint(numCams, p, pixelVar, M, cov)) {
//                            p->setLocalDynamic();
//                            p->updatePosition(M, cov);
//                            p->bNewPt = false;
//                        } else {
//                            p->setFalse();
//                        }
                    }
                } else {
                    //dynamic or false
                    if (isDynamicPoint(numCams, p, pixelVar, M, cov)) {
//                        p->setLocalDynamic();
//                        p->updatePosition(M, cov);
                        p->setFalse();
                    } else {
                        int outlierViewId = isStaticRemovable(numCams, p,
                                pixelVar, M, cov, NUM_FRAME_CHECK_STATIC);
                        if (outlierViewId >= 0) {
                            FeaturePoint* fp = p->pFeatures[outlierViewId];
                            fp->mpt = 0;
                            p->pFeatures[outlierViewId] = 0;
                            p->numVisCam--;
                            p->setLocalStatic();
                            p->updatePosition(M, cov);
                        } else {
                            p->setFalse();
                        }
                    }
                }
            } else if (p->isLocalDynamic()) {
                //can return to static or be a false point
                if (isDynamicPoint(numCams, p, pixelVar, M, cov)) {
                    if (isLittleMove(numCams, p, pixelVar, M, cov)) {
                        p->staticFrameNum++;
                        if (p->staticFrameNum > FRAME_NUM_FOR_DONTMOVE) {
                            double M0[3], cov0[9];
                            if (isStaticPoint(numCams, p, pixelVar, M0, cov0,
                                    NUM_FRAME_CHECK_STATIC)) {
                                //go back to static points
                                p->setLocalStatic();
                                for (int i = 0; i < numCams; i++) {
                                    if (p->pFeatures[i])
                                        p->pFeatures[i]->type =
                                                TYPE_FEATPOINT_STATIC;
                                }
                                p->updatePosition(M0, cov0);
                            } else {
                                p->staticFrameNum = 0;
                                p->updatePosition(M, cov);
                            }
                        } else
                            p->updatePosition(M, cov);
                    } else {
                        p->staticFrameNum = 0;
                        p->updatePosition(M, cov);
                    }
                    p->updatePosition(M, cov);
                } else
                    p->setFalse();
            }
        }
    }
}

int CoSLAM::searchNearestCamForMapPt(const Mat_d& camDist, const MapPoint* p,
		int iCam) {
	int jCam = -1;
	double dMin = DBL_MAX;
	for (int j = 0; j < numCams; j++) {
		if (p->pFeatures[j]) {
			double d = camDist.data[iCam * numCams + j];
			if (d < dMin) {
				jCam = j;
				dMin = d;
			}
		}
	}
	return jCam;
}
void CoSLAM::getCurrentCamDist(Mat_d& camDist) {
	camDist.resize(numCams, numCams);
	//compute the distance matrix for cameras
#ifdef USE_OPENMP
#pragma omp parallel for num_threads(SLAM_CPU_CORE_NUM)
#endif
	for (int i = 0; i < numCams; i++) {
		camDist.data[i * numCams + i] = -1;
		for (int j = i + 1; j < numCams; j++) {
			double d = getCamDist(slam[i].m_camPos.current(),
					slam[j].m_camPos.current());
			camDist.data[i * numCams + j] = d;
			camDist.data[j * numCams + i] = d;
		}
	}
}
bool CoSLAM::compareFeaturePt(const ImgG& scaledImg1, double imgScale1,
		const ImgG& scaledImg2, double imgScale2, const FeaturePoint* pt1,
		const FeaturePoint* pt2) {

	NCCBlock nccblk1, nccblk2;
	getScaledNCCBlock(scaledImg1, imgScale1, pt1->x, pt1->y, nccblk1);
	getScaledNCCBlock(scaledImg2, imgScale2, pt2->x, pt2->y, nccblk2);

	double ncc = matchNCCBlock(&nccblk1, &nccblk2);
	if (ncc >= 0.75)
		return true;

	return true;
	//return false;
}
bool CoSLAM::checkUnify(MapPoint* mpt1, MapPoint* mpt2, double M[],
		double cov[], double pixelErrVar) {
	double Ks[SLAM_MAX_NUM * 36], Rs[SLAM_MAX_NUM * 36], ts[SLAM_MAX_NUM * 12],
			ms[SLAM_MAX_NUM * 8], nms[SLAM_MAX_NUM * 8];
	int numView = 0;
	for (int i = 0; i < numCams; i++) {
		if (mpt1->pFeatures[i]) {
			MapPoint* p = mpt1;
			double iK[9];
			getInvK(slam[i].K.data, iK);
			FeaturePoint* fp = p->pFeatures[i];
			double* R0 = fp->cam->R;
			double* t0 = fp->cam->t;

			doubleArrCopy(Ks, numView, slam[i].K.data, 9);
			doubleArrCopy(Rs, numView, R0, 9);
			doubleArrCopy(ts, numView, t0, 3);
			doubleArrCopy(ms, numView, fp->m, 2);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;

			double C0[3];
			getCameraCenter(R0, t0, C0);

			FeaturePoint* maxFp = 0;
			double maxAngle = 0;
			fp = fp->preFrame;
			while (fp) {
				double C[3];
				getCameraCenter(fp->cam->R, fp->cam->t, C);
				double angle = getAbsRadiansBetween(p->M, C0, C);
				if (angle > maxAngle) {
					maxAngle = angle;
					maxFp = fp;
				}
				fp = fp->preFrame;
			}

			if (maxFp) {
				doubleArrCopy(Ks, numView, slam[i].K.data, 9);
				doubleArrCopy(Rs, numView, maxFp->cam->R, 9);
				doubleArrCopy(ts, numView, maxFp->cam->t, 3);
				doubleArrCopy(ms, numView, maxFp->m, 2);
				normPoint(iK, maxFp->m, nms + 2 * numView);
				numView++;
			}
		}
		if (mpt2->pFeatures[i]) {
			MapPoint* p = mpt2;
			double iK[9];
			getInvK(slam[i].K.data, iK);
			FeaturePoint* fp = p->pFeatures[i];
			double* R0 = fp->cam->R;
			double* t0 = fp->cam->t;

			doubleArrCopy(Ks, numView, slam[i].K.data, 9);
			doubleArrCopy(Rs, numView, R0, 9);
			doubleArrCopy(ts, numView, t0, 3);
			doubleArrCopy(ms, numView, fp->m, 2);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;

			double C0[3];
			getCameraCenter(R0, t0, C0);

			FeaturePoint* maxFp = 0;
			double maxAngle = 0;
			fp = fp->preFrame;
			while (fp) {
				double C[3];
				getCameraCenter(fp->cam->R, fp->cam->t, C);
				double angle = getAbsRadiansBetween(p->M, C0, C);
				if (angle > maxAngle) {
					maxAngle = angle;
					maxFp = fp;
				}
				fp = fp->preFrame;
			}

			if (maxFp) {
				doubleArrCopy(Ks, numView, slam[i].K.data, 9);
				doubleArrCopy(Rs, numView, maxFp->cam->R, 9);
				doubleArrCopy(ts, numView, maxFp->cam->t, 3);
				doubleArrCopy(ms, numView, maxFp->m, 2);
				normPoint(iK, maxFp->m, nms + 2 * numView);
				numView++;
			}
		}
	}

	triangulateMultiView(numView, Rs, ts, nms, M);
	getTriangulateCovMat(numView, Ks, Rs, ts, M, cov, pixelErrVar);

	//check the reprojection error
	for (int i = 0; i < numView; i++) {
		double rm[2], var[4], ivar[4];
		project(Ks + 9 * i, Rs + 9 * i, ts + 3 * i, M, rm);
		getProjectionCovMat(Ks + 9 * i, Rs + 3 * i, ts + 3 * i, M, cov, var,
				pixelErrVar);
		mat22Inv(var, ivar);
		if (mahaDist2(rm, ms + 2 * i, ivar) > 1.0)
			return false;
	}
	return true;
}
void CoSLAM::refineMapPoint(MapPoint* p) {
	double Ks[SLAM_MAX_NUM * 18], Rs[SLAM_MAX_NUM * 18], ts[SLAM_MAX_NUM * 6],
			ms[SLAM_MAX_NUM * 4], nms[SLAM_MAX_NUM * 4];
	int numView = 0;
	for (int i = 0; i < numCams; i++) {
		if (p->pFeatures[i]) {
			double iK[9];
			getInvK(slam[i].K.data, iK);
			FeaturePoint* fp = p->pFeatures[i];
			double* R0 = fp->cam->R;
			double* t0 = fp->cam->t;

			doubleArrCopy(Ks, numView, slam[i].K.data, 9);
			doubleArrCopy(Rs, numView, R0, 9);
			doubleArrCopy(ts, numView, t0, 3);
			doubleArrCopy(ms, numView, fp->m, 2);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;

			double C0[3];
			getCameraCenter(R0, t0, C0);
			FeaturePoint* maxFp = 0;
			double maxAngle = 0;
			fp = fp->preFrame;
			while (fp) {
				double C[3];
				getCameraCenter(fp->cam->R, fp->cam->t, C);
				double angle = getAbsRadiansBetween(p->M, C0, C);
				if (angle > maxAngle) {
					maxAngle = angle;
					maxFp = fp;
				}
				fp = fp->preFrame;
			}

			if (maxFp) {
				doubleArrCopy(Ks, numView, slam[i].K.data, 9);
				doubleArrCopy(Rs, numView, maxFp->cam->R, 9);
				doubleArrCopy(ts, numView, maxFp->cam->t, 3);
				doubleArrCopy(ms, numView, maxFp->m, 2);
				normPoint(iK, maxFp->m, nms + 2 * numView);
				numView++;
			}
		}
	}
	triangulateMultiView(numView, Rs, ts, nms, p->M);
	getTriangulateCovMat(numView, Ks, Rs, ts, p->M, p->cov,
			Const::PIXEL_ERR_VAR);
}
bool CoSLAM::staticCheckMergability(const MapPoint* mp, const FeaturePoint* fp,
		double pixelVar) {
	bool bMergable = true;
	for (const FeaturePoint* p = fp; p; p = p->preFrame) {
		double rm[2], var[4], ivar[4];
		project(p->K, p->cam->R, p->cam->t, mp->M, rm);
		getProjectionCovMat(p->K, p->cam->R, p->cam->t, mp->M, mp->cov, var,
				pixelVar);
		mat22Inv(var, ivar);
		if (mahaDist2(rm, p->m, ivar) > 1.0) {
			bMergable = false;
			break;
		}
	}
	return bMergable;
}
bool CoSLAM::curStaticPointRegInGroup(const CameraGroup& camGroup,
		Mat_d& camDist, MapPoint* p, double pixelErrVar, bool bMerge) {
	bool bReg = false;
	if (p->isLocalStatic() && p->numVisCam > 0) {
		for (int i = 0; i < camGroup.num; i++) {
			int iCam = camGroup.camIds[i];
			if (p->pFeatures[iCam] && p->pFeatures[iCam]->f == curFrame)
				continue;
			double m[2], var[4];
			if (isAtCameraBack(slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M))
				continue;

			project(slam[iCam].K.data, slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, m);
			if (m[0] < 0 || m[0] >= slam[iCam].videoReader->_w || m[1] < 0
					|| m[1] >= slam[iCam].videoReader->_h)
				continue;

			getProjectionCovMat(slam[iCam].K.data,
					slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, p->cov, var,
					pixelErrVar);

			FeaturePoint* pFeat = searchMahaNearestFeatPt(slam[iCam].m_featPts,
					curFrame, m, var, pixelErrVar * 3);

			if (pFeat > 0 && pFeat->type != TYPE_FEATPOINT_DYNAMIC) {
				if (pFeat->mpt == 0) {
					int jCam = searchNearestCamForMapPt(camDist, p, iCam);
					assert(jCam >= 0 && p->pFeatures[jCam]);
					if (jCam >= 0 && p->pFeatures[jCam]) {
						FeaturePoint* pFeatTmp = p->pFeatures[jCam];
						if (compareFeaturePt(slam[iCam].m_smallImg,
								slam[iCam].m_smallScale, slam[jCam].m_smallImg,
								slam[jCam].m_smallScale, pFeat, pFeatTmp)) {
							if (staticCheckMergability(p, pFeat, pixelErrVar)) {

								FeaturePoint* fp = pFeat->preFrame;
								while (fp) {
									fp->mpt = p;
									fp = fp->preFrame;
								}

								if (p->pFeatures[iCam]) {
									//already have a 2D point at the old frame
									p->pFeatures[iCam]->nextFrame = pFeat;
									pFeat->preFrame = p->pFeatures[iCam];
								}
								p->addFeature(iCam, pFeat);
								//update the NCC block
								p->nccBlks[iCam].computeScaled(
										slam[iCam].m_smallImg,
										slam[iCam].m_smallScale, pFeat->x,
										pFeat->y);
								bReg = true;
							}
						}
					}
				} else {
					if (!bMerge)
						return bReg;
					if (!pFeat->mpt->isLocalStatic())
						continue;
					if (p == pFeat->mpt)
						continue;
					//assert(p != pFeat->mpt);
					//conflict, check if the two map points can be unified
					double M[3], cov[9];
					if (checkUnify(p, pFeat->mpt, M, cov, pixelErrVar)) {
						p->updatePosition(M, cov);
						//merge the two map points
						pFeat->mpt->numVisCam = 0;
						pFeat->mpt->setFalse();

						//unify the two map points
						for (int v = 0; v < numCams; v++) {
							FeaturePoint* pFt = pFeat->mpt->pFeatures[v];
							if (pFt && !p->pFeatures[v]) {
								pFeat->mpt->pFeatures[v] = 0;
								pFt->mpt = 0;
								p->addFeature(v, pFt);
								for (FeaturePoint* fp = pFt->preFrame; fp; fp =
										fp->preFrame)
									fp->mpt = p;
							}
						}

						//update the visibility
						p->numVisCam = 0;
						for (int v = 0; v < numCams; v++) {
							if (p->pFeatures[v])
								p->numVisCam++;
						}
						bReg = true;
						return bReg;
					}
				}
			}
		}
	}
	return bReg;
}
int CoSLAM::currentMapPointsRegister(double pixelErrVar, bool bMerge) {
	//time measurement
	TimeMeasurer tm;
	tm.tic();

	//register static points
	int nReg = 0;
	for (int i = 0; i < m_groupNum; i++) {
		nReg += curStaticPointsRegInGroup(m_groups[i], pixelErrVar, bMerge);
	}

	//register dynamic points
	for (int i = 0; i < m_groupNum; i++) {
		nReg += curDynamicPointsRegInGroup(m_groups[i], pixelErrVar, bMerge);
	}
	//time measurement
	m_tmCurMapRegister = tm.toc();
	return nReg;
}

int CoSLAM::curStaticPointsRegInGroup(const CameraGroup& camGroup,
		double pixelErrVar, bool bMerge) {
	enterBACriticalSection();
	Mat_d camDist;
	getCurrentCamDist(camDist);

	int nTotalRegged = 0;

	for (int i = 0; i < camGroup.num; i++) {
		int iCam = camGroup.camIds[i];
		MapPoint* pCurMapHead = curMapPts.getHead();
		if (!pCurMapHead)
			repErr("no map point is found");

		std::vector<MapPoint*> vecMapPts;
		for (MapPoint* p = pCurMapHead; p; p = p->next) {
			if (p->isCertainStatic() && p->pFeatures[iCam]
					&& p->pFeatures[iCam]->f == curFrame) {
				vecMapPts.push_back(p);
			}
		}
		size_t nPts = vecMapPts.size();
		std::vector<bool> flagReged;
		flagReged.assign(nPts, false);

		std::vector<MapPoint*> vecReggedMapPts;
		vecReggedMapPts.reserve(nPts);
		int nRegged = 0;
		for (size_t i = 0; i < nPts; i++) {
			MapPoint* p = vecMapPts[i];
			if (p->isCertainStatic()) {
				flagReged[i] = curStaticPointRegInGroup(camGroup, camDist, p,
						pixelErrVar, bMerge);
				if (flagReged[i]) {
					vecReggedMapPts.push_back(p);
					nRegged++;
				}
			}
		}
		for (int i = 0; i < nRegged; i++) {
			//update 3D position
			MapPoint* p = vecReggedMapPts[i];
			refineMapPoint(p);
		}
		nTotalRegged += nRegged;
	}
	leaveBACriticalSection();
	return nTotalRegged;
}

int CoSLAM::curDynamicPointsRegInGroup(const CameraGroup& camGroup,
		double pixelErrVar, bool bMerge) {
	enterBACriticalSection();
	//	//test
	//	logInfo("currentMapPointsRegisterInGroup!\n");
	Mat_d camDist;
	getCurrentCamDist(camDist);

	int nTotalRegged = 0;

	for (int i = 0; i < camGroup.num; i++) {
		int iCam = camGroup.camIds[i];
		MapPoint* pCurMapHead = curMapPts.getHead();
		if (!pCurMapHead)
			repErr("no map point is found");

		std::vector<MapPoint*> vecMapPts;
		for (MapPoint* p = pCurMapHead; p; p = p->next) {
			if (p->isCertainDynamic() && p->pFeatures[iCam]
					&& p->pFeatures[iCam]->f == curFrame) {
				vecMapPts.push_back(p);
			}
		}
		size_t nPts = vecMapPts.size();
		std::vector<bool> flagReged;
		flagReged.assign(nPts, false);

		std::vector<MapPoint*> vecReggedMapPts;
		vecReggedMapPts.reserve(nPts);
		int nRegged = 0;
		for (size_t i = 0; i < nPts; i++) {
			MapPoint* p = vecMapPts[i];
			if (p->isCertainDynamic()) {
				flagReged[i] = curDynamicPointRegInGroup(camGroup, camDist, p,
						pixelErrVar, bMerge);
				if (flagReged[i]) {
					vecReggedMapPts.push_back(p);
					nRegged++;
				}
			}
		}
		for (int i = 0; i < nRegged; i++) {
			//update 3D position
			MapPoint* p = vecReggedMapPts[i];
			refineMapPoint(p);
		}
		nTotalRegged += nRegged;
	}
	leaveBACriticalSection();
	return nTotalRegged;
}
bool CoSLAM::curDynamicPointRegInGroup(const CameraGroup& camGroup,
		Mat_d& camDist, MapPoint* p, double pixelErrVar, bool bMerge) {
	bool bReg = false;
	if (p->isCertainDynamic() && p->numVisCam > 0) {
		for (int i = 0; i < camGroup.num; i++) {
			int iCam = camGroup.camIds[i];
			if (p->pFeatures[iCam] && p->pFeatures[iCam]->f == curFrame)
				continue;
			double m[2], var[4];
			if (isAtCameraBack(slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M))
				continue;

			project(slam[iCam].K.data, slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, m);
			if (m[0] < 0 || m[0] >= slam[iCam].videoReader->_w || m[1] < 0
					|| m[1] >= slam[iCam].videoReader->_h)
				continue;

			getProjectionCovMat(slam[iCam].K.data,
					slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, p->cov, var,
					pixelErrVar);
			FeaturePoint* pFeat = searchMahaNearestFeatPt(slam[iCam].m_featPts,
					curFrame, m, var, pixelErrVar * 4);

			if (pFeat > 0 && pFeat->type == TYPE_FEATPOINT_DYNAMIC) {
				if (pFeat->mpt == 0) {
					int jCam = searchNearestCamForMapPt(camDist, p, iCam);
					assert(jCam >= 0 && p->pFeatures[jCam]);
					if (jCam >= 0 && p->pFeatures[jCam]) {
						FeaturePoint* pFeatTmp = p->pFeatures[jCam];
						if (compareFeaturePt(slam[iCam].m_smallImg,
								slam[iCam].m_smallScale, slam[jCam].m_smallImg,
								slam[jCam].m_smallScale, pFeat, pFeatTmp)) {
							if (staticCheckMergability(p, pFeat, pixelErrVar)) {

								FeaturePoint* fp = pFeat->preFrame;
								while (fp) {
									fp->mpt = p;
									fp = fp->preFrame;
								}
								if (p->pFeatures[iCam]) {
									//already have a 2D point at the old frame
									p->pFeatures[iCam]->nextFrame = pFeat;
									pFeat->preFrame = p->pFeatures[iCam];
								}
								p->addFeature(iCam, pFeat);
								//update the NCC block
								p->nccBlks[iCam].computeScaled(
										slam[iCam].m_smallImg,
										slam[iCam].m_smallScale, pFeat->x,
										pFeat->y);
								bReg = true;
							}
						}
					}
				} else {
					return bReg;
				}
			}
		}
	}
	return bReg;
}

int CoSLAM::getNearestFeatPtForReg(const MapPoint* mp,
		const CamPoseItem* cam0) {
	int minI = -1;
	double minDist = DBL_MAX;
	for (int i = 0; i < numCams; i++) {
		const FeaturePoint* fp = mp->pFeatures[i];
		if (fp) {
			double dist = getCamDist(fp->cam, cam0);
			if (dist < minDist) {
				minI = i;
				minDist = dist;
			}
		}
	}
	return minI;
}
int CoSLAM::activeMapPointsRegister(double pixelErrVar) {
	int nReg = 0;
	for (int i = 0; i < m_groupNum; i++) {
		nReg += activeMapPointsRegisterInGroup(m_groups[i], pixelErrVar);
	}
	return nReg;
}
int CoSLAM::activeMapPointsRegisterInGroup(const CameraGroup& camGroup,
		double pixelErrVar) {
	//time measurement
	TimeMeasurer tm;
	tm.tic();

	/*==============(critical section)=================*/
	enterBACriticalSection();
	MapPoint* pActMapHead = actMapPts.getHead();
	leaveBACriticalSection();
	/*------------------------------------------------*/

	if (!pActMapHead) {
		return 0;
	}

	vector<MapPoint*> vecActMapPts;
	vecActMapPts.reserve(actMapPts.getNum() * 2);

	/*==============(critical section)=================*/
	enterBACriticalSection();
	for (MapPoint* p = pActMapHead; p; p = p->next) {
		vecActMapPts.push_back(p);
	}
	actMapPts.clearWithoutRelease();
	size_t nPts = vecActMapPts.size();
	leaveBACriticalSection();
	/*------------------------------------------------*/

	Mat_i flagReged(nPts, 1);

	/*==============(critical section)=================*/
	enterBACriticalSection();
	//	#pragma omp parallel for num_threads(4)
	for (size_t i = 0; i < nPts; i++) {
		flagReged.data[i] = 0;
		MapPoint* p = vecActMapPts[i];
		if (p->isCertainStatic()) {
			flagReged.data[i] = activeMapPointRegisterInGroup(camGroup, p,
					pixelErrVar);
		}
	}
	leaveBACriticalSection();
	/*------------------------------------------------*/

	/*==============(critical section)=================*/
	enterBACriticalSection();
	int nReg = 0;
	for (size_t i = 0; i < nPts; i++) {
		MapPoint* p = vecActMapPts[i];
		if (flagReged.data[i] == 0) {
			actMapPts.add(p);
		} else {
			p->lastFrame = curFrame;
			p->state = STATE_MAPPOINT_CURRENT;
			curMapPts.add(p);
			nReg++;
		}
	}
	leaveBACriticalSection();
	/*------------------------------------------------*/

	//time measurement
	m_tmActMapRegister = tm.toc();
	return nReg;
}
bool CoSLAM::activeMapPointRegisterInGroup(const CameraGroup& camGroup,
		MapPoint* p, double pixelErrVar) {
	bool bReg = false;
	//register the active map point to the feature points at the current frame
	if (p->isCertainStatic() && p->numVisCam > 0 && p->numVisCam < numCams) {
		bool bNotInGroup = true;
		for (int i = 0; i < camGroup.num; i++) {
			int iCam = camGroup.camIds[i];
			if (p->pFeatures[iCam])
				bNotInGroup = false;
		}
		if (bNotInGroup) {
			return false;
		}
		for (int i = 0; i < camGroup.num; i++) {
			int iCam = camGroup.camIds[i];
			double m[2], var[4];
			if (isAtCameraBack(slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M))
				continue;

			//project the active map point to the current view
			project(slam[iCam].K.data, slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, m);
			if (m[0] < 0 || m[0] >= slam[iCam].videoReader->_w || m[1] < 0
					|| m[1] >= slam[iCam].videoReader->_h)
				continue;

			getProjectionCovMat(slam[iCam].K.data,
					slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, p->cov, var,
					pixelErrVar * 2.5);

			FeaturePoint* pFeat = searchMahaNearestFeatPt(slam[iCam].m_featPts,
					curFrame, m, var, pixelErrVar * 3.0);

			if (pFeat > 0) {
				if (pFeat->mpt == 0) {
					int jCam = getNearestFeatPtForReg(p,
							slam[iCam].m_camPos.current());
					assert(jCam >= 0);
					//compute the feature points with NCC blocks
					NCCBlock blk;
					getScaledNCCBlock(slam[iCam].m_smallImg,
							slam[iCam].m_smallScale, pFeat->x, pFeat->y, blk);
					if (matchNCCBlock(&blk, &p->nccBlks[jCam]) > 0.60) {
						if (staticCheckMergability(p, pFeat, pixelErrVar)) {
							if (jCam == iCam) {
								p->pFeatures[jCam]->nextFrame = pFeat;
								pFeat->preFrame = p->pFeatures[jCam];
							}
							p->addFeature(iCam, pFeat);

							//update the NCC block
							p->nccBlks[iCam].copy(blk);

							refineMapPoint(p);
							bReg = true;
						}
					}
				}
			}
		}
	}
	return bReg;
}
void CoSLAM::mapStateUpdate() {
	MapPoint* pCurMapHead = curMapPts.getHead();
	if (!pCurMapHead)
		return;

	//put the map points with no corresponding feature points at the current frame into the `actMapPts' list
	for (MapPoint* p = pCurMapHead; p; p = p->next) {
		p->updateVisCamNum(curFrame);
		if (p->lastFrame < curFrame) {
			MapPoint* pt = p;
			p = curMapPts.remove(pt);
			if (pt->isFalse())
				falseMapPts.add(pt);
			else if (pt->isCertainStatic()) {
				pt->state = STATE_MAPPOINT_ACTIVE;
				actMapPts.add(pt);
			} else {
				pt->state = STATE_MAPPOINT_INACTIVE;
				iactMapPts.add(pt);
			}
		}
	}

	MapPoint* pActMapHead = actMapPts.getHead();
	if (!pActMapHead)
		return;

	//put the map points that have no corresponding image point into the `iactMapPts' list
	for (MapPoint* p = pActMapHead; p; p = p->next) {
		if (p->lastFrame < curFrame - numActFrm) {
			MapPoint* pt = p;
			p = actMapPts.remove(pt);
			if (pt->isFalse())
				falseMapPts.add(pt);
			else {
				pt->state = STATE_MAPPOINT_INACTIVE;
				iactMapPts.add(pt);
			}
		} else if (p->lastFrame == curFrame) {
			//put back to the `curMapPts'
			MapPoint* pt = p;
			p = actMapPts.remove(pt);
			curMapPts.add(pt);
		}
	}assert(curMapPts.getNum() == curMapPts.count());
	assert(actMapPts.getNum() == actMapPts.count());
	assert(iactMapPts.getNum() == iactMapPts.count());
}
void CoSLAM::getCurMapCenterViewFrom(int camId, double center[3]) {
	const FeaturePoint* pHead = slam[camId].m_featPts.getFrameHead(curFrame);
	const FeaturePoint* pTail = slam[camId].m_featPts.getFrameTail(curFrame);

	center[0] = 0;
	center[1] = 0;
	center[2] = 0;

	int nPts = 0;
	for (const FeaturePoint* fp = pHead; fp && fp != pTail; fp = fp->next) {
		if (fp->mpt && !fp->mpt->isFalse()) {
			center[0] += fp->mpt->x;
			center[1] += fp->mpt->y;
			center[2] += fp->mpt->z;
			nPts++;
		}
	}
	//assert(nPts > 0);
	if (nPts == 0)
		pause();

	center[0] /= nPts;
	center[1] /= nPts;
	center[2] /= nPts;
}

bool CoSLAM::IsMappedPtsDecreaseBelow(int camId, double ratio) {
	const FeaturePoint* pHead = slam[camId].m_featPts.getFrameHead(curFrame);
	const FeaturePoint* pTail = slam[camId].m_featPts.getFrameTail(curFrame);

	int num = 0;
	int lastNum = slam[camId].m_keyPose.tail->nMappedPts;
	int lastFrame = slam[camId].m_keyPose.tail->frame;

	for (const FeaturePoint* fp = pHead; fp && fp != pTail; fp = fp->next) {
		if (fp->mpt && fp->mpt->firstFrame <= lastFrame)
			num++;
	}
	if (num < lastNum * ratio) {
		return true;
	}
	if (num < 30)
		return true;
	return false;
}
int CoSLAM::IsReadyForKeyFrame(int camId) {
	double center[3];
	getCurMapCenterViewFrom(camId, center);
	if (IsMappedPtsDecreaseBelow(camId, m_mappedPtsReduceRatio))
		return READY_FOR_KEY_FRAME_DECREASE;
	if (slam[camId].getViewAngleChangeSelf(center) > m_minViewAngleChange)
		return READY_FOR_KEY_FRAME_VIEWANGLE;
	if (slam[camId].getCameraTranslationSelf() > m_minCamTranslation)
		return READY_FOR_KEY_FRAME_TRANSLATION;
	return 0;
}
KeyFrame* CoSLAM::addKeyFrame(int readyForKeyFrame[SLAM_MAX_NUM]) {
	KeyFrame* pKeyFrame = m_keyFrms.add(curFrame);
	m_keyFrameId[curFrame] = m_nKeyFrame++;
	for (int i = 0; i < numCams; i++)
		pKeyFrame->setKeyPose(i, slam[i].addKeyPose(readyForKeyFrame[i] > 0));

	//store some useful information at the current key frame
	pKeyFrame->setCamNum(numCams);
	pKeyFrame->setMapPtsNum(curMapPts.getNum());
	pKeyFrame->setCamGroups(m_groups, m_groupNum);
	
	pKeyFrame->storeDynamicMapPoints(curMapPts);

	return pKeyFrame;
}
int CoSLAM::genNewMapPoints() {
	int num = 0;
	getNumDynamicStaticPoints();

	//check if there is one camera ready for insert a new key frame
	int readyForKeyFrame[SLAM_MAX_NUM];
	int nReady = 0;
	bool decrease = false;
	for (int i = 0; i < numCams; i++) {
		slam[i].getNumMappedStaticPts();
		readyForKeyFrame[i] = IsReadyForKeyFrame(i);
		if (readyForKeyFrame[i] > 0)
			nReady++;
		if (readyForKeyFrame[i] == READY_FOR_KEY_FRAME_DECREASE)
			decrease = true;
	}
	if (nReady > 0) {
		for (int i = 0; i < numCams; i++) {
            if( m_lastFrmInterMapping + 20 < curFrame)
                continue;
            //EDIT FOR TRUE (ALWAYS PERFORMED)
			if ((readyForKeyFrame[i] > 1 && numCams > 1)
					|| (readyForKeyFrame[i] > 0 && numCams == 1)) {
				if (m_groupId[i] == m_mergedgid
						&& curFrame > m_lastFrmGroupMerge
						&& curFrame < m_lastFrmGroupMerge + 130)
					continue;
				enterBACriticalSection();
				vector<MapPoint*> newMapPts;
				int iNum = slam[i].newMapPoints(newMapPts);
				num += iNum;
				m_lastFrmIntraMapping = curFrame;
				for (size_t k = 0; k < newMapPts.size(); k++) {
					curMapPts.add(newMapPts[k]);
				}
				leaveBACriticalSection();

			}
		}

		if (decrease) {
			//for bundle adjsutment and group merging
			//add a new key frame
			KeyFrame* pKeyFrame = addKeyFrame(readyForKeyFrame);

			//check if there are camera groups that can be merged
			bool merged = false;
			if (m_groupNum > 1)
				merged = mergeCamGroups(pKeyFrame);

			if (curFrame > m_lastFrmGroupMerge + 2) {
				requestForBA(5, 2, 2, 30);
			}

			for (int i = 0; i < numCams; i++) {
				if (readyForKeyFrame[i] == READY_FOR_KEY_FRAME_DECREASE
						&& numCams > 1) {
					enterBACriticalSection();
					vector<MapPoint*> newMapPts;
					int iNum = slam[i].newMapPoints(newMapPts);
					num += iNum;
					m_lastFrmIntraMapping = curFrame;
					for (size_t k = 0; k < newMapPts.size(); k++) {
						curMapPts.add(newMapPts[k]);
					}
					leaveBACriticalSection();
				}
			}
		}
	}

	if (numCams == 1)
		return num;

	if (decrease || curFrame - m_lastFrmInterMapping > 3) {
		num += genNewMapPointsInterCam(false);
		m_lastFrmInterMapping = curFrame;
	}
	return num;
}

bool CoSLAM::mergeCamGroups(KeyFrame* newFrame) {
	MergeCameraGroup mcg;
	mcg.setCurrentFrame(newFrame);
	for (int i = 0; i < numCams; i++)
		mcg.setImageSize(i, slam[i].m_img.w, slam[i].m_img.h);

	if( curFrame < m_lastFrmGroupMerge + 130)
		return false;
	
	if (mcg.checkPossibleMergable(10, 0.5, Param::maxDistRatio) > 0) {
#ifdef WIN32
		Sleep(3000);
#else
		sleep(3);
#endif
		//store the images and feature points
		for (int i = 0; i < numCams; i++)
			newFrame->pPose[i]->setImage(slam[i].m_img);

		mcg.storeFeaturePoints();
		mcg.printMergeInfo();

		if (mcg.matchMergableCameras() > 0) {
			//start to compute the new positions of key camera poses
			//test
			logInfo("before correction!\n");
			mcg.printMergeInfo();
			MyApp::bCancelBA = true;

			//search for the first key frame when all the separated cameras are in the same group.
			mcg.searchFirstKeyFrameForMerge();

			mcg.constructGraphForKeyFrms();
			mcg.constructGraphForAllFrms();


			enterBACriticalSection();
			mcg.recomputeKeyCamPoses();
			leaveBACriticalSection();

			enterBACriticalSection();
			mcg.recomputeAllCameraPoses();
			m_mergedgid = mcg.mergeMatchedGroups(m_groups, m_groupNum);
			//update group id
			for (int g = 0; g < m_groupNum; ++g) {
				for (int i = 0; i < m_groups[g].num; ++i) {
					int c = m_groups[g].camIds[i];
					m_groupId[c] = g;
				}
			}
			leaveBACriticalSection();


			enterBACriticalSection();
			vector<MapPoint*> keyMapPoints;
			int f_start =
					mcg.getFirstFrame() < m_lastReleaseFrm ?
							m_lastReleaseFrm : mcg.getFirstFrame();

			int f_end = mcg.getLastFrame();
			getMapPts(f_start, f_end, keyMapPoints);
			mcg.recomputeMapPoints(keyMapPoints, Const::PIXEL_ERR_VAR);
			leaveBACriticalSection();

			currentMapPointsRegister(20.0, true);

			m_lastFrmGroupMerge = curFrame;
			return true;
		}
	}
	return false;
}
void CoSLAM::getNumDynamicStaticPoints() {
	//check whether there are too many dynamic map points
	MapPoint* pM = curMapPts.getHead();
	m_nStatic = 0;
	m_nDynamic = 0;
	for (int j = 0; j < numCams; j++) {
		m_nStaticFeat[j] = 0;
		m_nDynamicFeat[j] = 0;
	}
	for (; pM; pM = pM->next) {
		if (pM->isCertainStatic())
			m_nStatic++;
		else if (pM->isCertainDynamic())
			m_nDynamic++;

		for (int j = 0; j < numCams; j++) {
			if (pM->pFeatures[j]) {
				if (pM->isCertainStatic())
					m_nStaticFeat[j]++;
				else if (pM->isCertainDynamic())
					m_nDynamicFeat[j]++;
			}
		}
	}
}
//b#define USE_GPUSURF
int CoSLAM::genNewMapPointsInterCam(bool bUseSURF) {
	int num = 0;
	if (bUseSURF) {
#ifdef USE_GPUSURF
		TimeMeasurer tm;
		tm.tic();
		NewMapPtsSURF matcher;

		const int thresNum = 400;
		for (int i = 0; i < m_groupNum; i++) {
			if (m_groups[i].num <= 1)
			continue;
			bool runMatching = false;
			enterBACriticalSection();
			for (int k = 0; k < m_groups[i].num; k++) {
				int camId = m_groups[i].camIds[k];
				if (m_nStaticFeat[camId] < thresNum) {
					runMatching = true;
					break;
				}
			}
			if (curFrame < 5)
			runMatching = false;
			if (runMatching) {
				matcher.setInputs(m_groups[i], this, curFrame);
				num += matcher.run();
				matcher.output();
				//test
				//logInfo("frame:%d, group (#%d) : inter-camera mapping :%d \n", curFrame, i, num);
			}
			leaveBACriticalSection();
		}
		m_tmNewMapPoints = tm.toc();
#endif
	} else {
		TimeMeasurer tm;
		tm.tic();
		NewMapPtsNCC matcher;

		//to avoid frequently calling inter-camera mapping if there are enough map points already
		const int thresNum = Param::nMaxMapPts;
		for (int i = 0; i < m_groupNum; i++) {
			if (m_groups[i].num <= 1)
				continue;
			bool runMatching = false;
			enterBACriticalSection();
			for (int k = 0; k < m_groups[i].num; k++) {
				int camId = m_groups[i].camIds[k];
				if (m_nStaticFeat[camId] < thresNum) {
					runMatching = true;
					break;
				}
			}
			//skip first five frames
			if (curFrame < 5)
				runMatching = false;

			if (runMatching) {
				matcher.setInputs(m_groups[i], this, curFrame);
				num += matcher.run();
				matcher.output();
			}

			leaveBACriticalSection();
		}
		m_tmNewMapPoints = tm.toc();
	}
	return num;
}

void CoSLAM::getViewOverlapCosts(double vcosts[SLAM_MAX_NUM * SLAM_MAX_NUM],
		int minOverlapNum, double minOverlapAreaRatio) {
	int nSharePoints[SLAM_MAX_NUM * SLAM_MAX_NUM];
	memset(nSharePoints, 0, numCams * numCams * sizeof(int));

	//store the shared points : sharePoints[iCam][jCam] represents feature points between camera #iCam and #jCam.
	for (int i = 0; i < numCams; i++) {
		for (int j = 0; j < numCams; j++) {
			if (i == j)
				continue;
			sharedPoints[i][j].clear();
			sharedPoints[i][j].reserve(8192);
			sharedConvexHull[i][j].clear();
			sharedConvexHull[i][j].reserve(1024);
		}
	}

	//compute the number of shared feature points between cameras
	for (const MapPoint* pt = curMapPts.getHead(); pt; pt = pt->next) {
		if (pt->numVisCam == 1 || pt->isFalse())
			continue;
		int viewIds[SLAM_MAX_NUM];
		double points[SLAM_MAX_NUM * 2];
		int nIds = 0;
		for (int i = 0; i < numCams; i++) {
			const FeaturePoint* fp = pt->pFeatures[i];
			if (fp && fp->f == curFrame) {
				points[2 * nIds] = fp->x;
				points[2 * nIds + 1] = fp->y;
				viewIds[nIds++] = i;
			}
		}

		for (int i = 0; i < nIds; i++) {
			int iCam = viewIds[i];
			for (int j = i + 1; j < nIds; j++) {
				int jCam = viewIds[j];
				nSharePoints[iCam * numCams + jCam]++;
				nSharePoints[jCam * numCams + iCam]++;

				double ix = points[2 * i];
				double iy = points[2 * i + 1];

				double jx = points[2 * j];
				double jy = points[2 * j + 1];

				sharedPoints[iCam][jCam].push_back(ix);
				sharedPoints[iCam][jCam].push_back(iy);
				sharedPoints[jCam][iCam].push_back(jx);
				sharedPoints[jCam][iCam].push_back(jy);
			}
		}
	}

	for (int i = 0; i < numCams; i++) {
		for (int j = 0; j < numCams; j++) {
			if (j == i)
				continue;
			get2DConvexHull(sharedPoints[i][j], sharedConvexHull[i][j]);
		}
	}

	for (int i = 0; i < numCams; i++) {
		vcosts[i * numCams + i] = -1;
		for (int j = i + 1; j < numCams; j++) {
			if (nSharePoints[i * numCams + j] < minOverlapNum) {
				vcosts[i * numCams + j] = -1;
				vcosts[j * numCams + i] = -1;
			} else {
				//check the overlap area
				double area1 = getPolyArea(sharedConvexHull[i][j]);
				double area2 = getPolyArea(sharedConvexHull[j][i]);

				double iArea = slam[i].videoReader->_w * slam[i].videoReader->_h;
				double jArea = slam[j].videoReader->_w * slam[j].videoReader->_h;

				if (area1 < minOverlapAreaRatio * iArea
						|| area2 < minOverlapAreaRatio * jArea) {
					vcosts[i * numCams + j] = -1;
					vcosts[j * numCams + i] = -1;
				} else {
					vcosts[i * numCams + j] = nSharePoints[i * numCams + j];
					vcosts[j * numCams + i] = vcosts[i * numCams + j];
				}
			}
		}
	}
}

int CoSLAM::cameraGrouping() {
	if (numCams == 1)
		return 1;
	getViewOverlapCosts(viewOverlapCost, 0, 0.0);

	//check distances between cameras
	for (int i = 0; i < numCams; ++i) {
		cout << i << " : ";
		for (int j = i + 1; j < numCams; ++j) {
			double M1[3], M2[3];
			getCamCenter(slam[i].m_camPos.current(), M1);
			getCamCenter(slam[j].m_camPos.current(), M2);

			if (viewOverlapCost[i * numCams + j] > 0
					&& dist3(M1, M2)
							> m_initCamTranslation * Param::maxDistRatio) {

				viewOverlapCost[i * numCams + j] = -1;
				viewOverlapCost[j * numCams + i] = -1;
			}
		}
		cout << endl;
	}
	cout << m_initCamTranslation * Param::maxDistRatio << endl;

	//divide the cameras into groups according to the overlap costs 
	//each connect component acts as a camera group
	int VQ[SLAM_MAX_NUM], CON[SLAM_MAX_NUM], flag[SLAM_MAX_NUM], nVQ = 0, nCON =
			0;
	m_groupNum = 0;
	fill_n(flag, numCams, 0);

	for (int i = 0; i < numCams; i++) {
		if (0 == flag[i]) {
			//a new connected components
			nVQ = nCON = 0;
			CON[nCON++] = i;
			VQ[nVQ++] = i;
			flag[i] = 1;

			while (nVQ > 0) {
				//pop from the stack
				int iCam = VQ[nVQ - 1];
				nVQ--;

				//add the connected vertices into the stack
				for (int j = 0; j < numCams; j++) {
					if (j != iCam && flag[j] == 0
							&& viewOverlapCost[iCam * numCams + j] > 0) {
						CON[nCON++] = j;
						VQ[nVQ++] = j;
						flag[j] = 1;
					}
				}
			}
			//output the connect components
			m_groups[m_groupNum].clear();
			for (int k = 0; k < nCON; k++) {
				m_groups[m_groupNum].addCam(CON[k]);
				m_groupId[CON[k]] = m_groupNum;
			}
			m_groupNum++;
		}
	}
	return m_groupNum;
}

/*
 * process the queued BA requests
 */
void* _bundleAdjustmentThread(void* param) {
	std::deque<RobustBundleRTS*>& queueBAs =
			*((std::deque<RobustBundleRTS*>*) param);
	MyApp::bBusyBAing = true;
	while (!queueBAs.empty()) {
        
		//start of critical section
		enterBACriticalSection();
		RobustBundleRTS* pBA = queueBAs.front();
		queueBAs.pop_front();
		leaveBACriticalSection();
		//end of critical section
		pBA->run();
        
        
		//start of critical section
		enterBACriticalSection();
		if (!MyApp::bCancelBA)
			pBA->output();
		delete pBA;
		MyApp::bCancelBA = false;
		leaveBACriticalSection();
		//end of critical section
	}
	MyApp::bBusyBAing = false;
	//test
	logInfo("bundle adjustment completed2!\n");
	return 0;
}
bool CoSLAM::requestForBA(int keyFrameNum, int maxOldKeyFrmNum, int maxIter,
		int innerMaxIter) {
	//get key frames for bundle adjustment
	KeyFrame* pLastKF = m_keyFrms.current();
	KeyFrame* pKF = pLastKF;

	int nKeyFrm = 0;
	int nPts = 0;

	KeyFrame* pFirstKF = 0;
	for (; nKeyFrm < keyFrameNum && pKF; nKeyFrm++, pKF = pKF->prev) {
		pFirstKF = pKF;
		nPts += pKF->nMapPts;
	}

	if (nKeyFrm < keyFrameNum) {
		logInfo(
				"not enough key frames for bundle adjustment[last %d, now %d]\n",
				m_lastFrmBundle, curFrame);
		return false;
	}
	if (MyApp::bBusyBAing) {
		logInfo(
				"waiting for the end of the last bundle adjustment[last %d, now %d]\n",
				m_lastFrmBundle, curFrame);
		return false;
	}
	//record the last key frame for bundle adjustment
	m_firstFrmBundle = pFirstKF->f;
	m_lastFrmBundle = pLastKF->f;
	RobustBundleRTS* pBA = new RobustBundleRTS();

	//allocate enough memory
	pBA->setCoSLAM(this);
	pBA->setFirstKeyFrame(pFirstKF);
	pBA->setLastKeyFrame(pLastKF);
	pBA->addKeyFrames();
	pBA->addPoints();
	pBA->setParameters(2, numCams * maxOldKeyFrmNum, maxIter, innerMaxIter);

	//start of critical section
	enterBACriticalSection();
	m_requestBAs.push_back(pBA);
	leaveBACriticalSection();
	//end of critical section

	if (!MyApp::bBusyBAing) {
		//start a new thread for bundle adjustment
		pthread_t thread;
		pthread_create(&thread, 0, _bundleAdjustmentThread, &m_requestBAs);
	}

	return true;
}

void CoSLAM::releaseFeatPts(int frame) {
	enterBACriticalSection();
	for (int i = 0; i < numCams; i++) {
		slam[i].removeFeatPts(frame);
	}
	leaveBACriticalSection();
}
void CoSLAM::releaseKeyPoseImgs(int frame) {
	enterBACriticalSection();
	for (int i = 0; i < numCams; ++i) {
		slam[i].removeKeyPoseImgs(frame);
	}
	leaveBACriticalSection();
}
#include "opencv2/opencv.hpp"
void CoSLAM::pause() {
	updateDisplayData();
	MyApp::bStop = true;
	redrawAllViews();
	while (MyApp::bStop) {
	};
}
void CoSLAM::printCamGroup() {
	for (int i = 0; i < m_groupNum; ++i) {
		cout << "group " << i << ": ";
		for (int j = 0; j < m_groups[i].num; ++j) {
			cout << m_groups[i].camIds[j] << " ";
		}
		cout << endl;
	}
}

void CoSLAM::getMapPts(int firstFrame, int lastFrame,
		vector<MapPoint*>& mapPoints) {
	//scan all maps
	MapPointList* mptLst = &iactMapPts;
	//test
	int k = 0;
	for (MapPoint* mpt = mptLst->getHead(); mpt; mpt = mpt->next) {
		k++;
		assert(mpt);
		if (mpt->lastFrame < firstFrame || mpt->firstFrame > lastFrame) {
			continue;
		}
		if (mpt->isCertainStatic())
			mapPoints.push_back(mpt);
	}
	mptLst = &actMapPts;
	for (MapPoint* mpt = mptLst->getHead(); mpt; mpt = mpt->next) {
		assert(mpt);
		if (mpt->lastFrame < firstFrame || mpt->firstFrame > lastFrame) {
			continue;
		}
		if (mpt->isCertainStatic())
			mapPoints.push_back(mpt);
	}
	mptLst = &curMapPts;
	for (MapPoint* mpt = mptLst->getHead(); mpt; mpt = mpt->next) {
		assert(mpt);
		if (mpt->lastFrame < firstFrame || mpt->firstFrame > lastFrame) {
			continue;
		}
		if (mpt->isCertainStatic())
			mapPoints.push_back(mpt);
	}
}
void CoSLAM::getAllStaticMapPoints(vector<MapPoint*>& mptPts) const {
	set<MapPoint*> mptSet;
	int num = 0;
	for (int c = 0; c < numCams; c++) {
		for (int f = 0; f <= curFrame; f++) {
			vector<FeaturePoint*> vecFeatPts;
			slam[c].m_featPts.getFrame(f, vecFeatPts);
			for (size_t i = 0; i < vecFeatPts.size(); i++) {
				if (vecFeatPts[i]->mpt
						&& vecFeatPts[i]->mpt->isCertainStatic()) {
					vecFeatPts[i]->mpt->id = (longInt) vecFeatPts[i]->mpt;
					mptSet.insert(vecFeatPts[i]->mpt);
					num++;
				}
			}
		}
	}
	mptPts.clear();
	mptPts.reserve(2 * num);
	for (set<MapPoint*>::iterator iter = mptSet.begin(); iter != mptSet.end();
			iter++) {
		mptPts.push_back(*iter);
	}
}
void CoSLAM::getAllStaticMapPtsAtKeyFrms(vector<MapPoint*>& mapPoints) const {
	mapPoints.clear();
	for (KeyFrame* kf = m_keyFrms.head.next; kf; kf = kf->next) {
		for (int c = 0; c < kf->nCam; c++) {
			FeaturePoint* pHead = kf->pPose[c]->pHead;
			FeaturePoint* pTail = kf->pPose[c]->pTail;
			for (FeaturePoint* fp = pHead; fp != pTail->next; fp = fp->next) {
				if (fp->mpt && !fp->mpt->isFalse()
						&& fp->mpt->isCertainStatic()) {
					mapPoints.push_back(fp->mpt);
				}
			}
		}
	}
}
void CoSLAM::getAllFeatPtsAtKeyFrms(int c, vector<FeaturePoint*>& featPoints) {
	featPoints.clear();
	for (KeyFrame* kf = m_keyFrms.head.next; kf; kf = kf->next) {
		FeaturePoint* pHead = kf->pPose[c]->pHead;
		FeaturePoint* pTail = kf->pPose[c]->pTail;
		for (FeaturePoint* fp = pHead; fp != pTail->next; fp = fp->next)
			featPoints.push_back(fp);
	}
}
void CoSLAM::storeDynamicPoints() {
	if (numCams == 1)
		return;
	using namespace std;
	std::vector<Point3dId> pts;
	for (MapPoint* mpt = curMapPts.getHead(); mpt; mpt = mpt->next) {
		if (mpt->isCertainDynamic()) {
			pts.push_back(Point3dId(mpt->x, mpt->y, mpt->z, mpt->id));
		}
	}
	m_dynPts.push_back(pts);
}

#include <time.h>
void CoSLAM::exportResultsVer1(const char timeStr[], double scale) const {
	using namespace std;
    char resPath[256];
	char dirPath[256];

    sprintf(resPath, "%s/slam_results", getenv("HOME"));
	sprintf(dirPath, "%s/%s", resPath, timeStr);
    
    mkdir(resPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	char filePath[256];

	//save the information of the input video sequences 
	//and camera parameters
	sprintf(filePath, "%s/input_videos.txt", dirPath);
	ofstream file(filePath);

	sprintf(filePath, "%s/pointsAndPath.xyz", dirPath);
	ofstream pointsAndPath(filePath);

	if (!file)
		repErr("cannot open file %s to write!\n", filePath);
	for (int c = 0; c < numCams; c++) {
		file << slam[c].videoFilePath << endl;
		for (size_t i = 0; i < 9; i++)
			file << slam[c].K[i] << " ";
		file << endl;
		for (size_t i = 0; i < 5; i++)
			file << slam[c].k_c.data[i] << " ";
		file << endl;
		file << slam[c].W << " " << slam[c].H << endl;
	}
	file.close();
	cout << filePath << " has been saved!" << endl;

	//save map points
	sprintf(filePath, "%s/mappts.xyz", dirPath);

	file.open(filePath);
	if (!file)
		repErr("cannot open file to write!\n");

	set<MapPoint*> mptSet;
	vector<MapPoint*> mapPoints;
	getAllStaticMapPoints(mapPoints);

	//file << mapPoints.size() << endl;
	for (size_t i = 0; i < mapPoints.size(); i++) {
		//test
		mptSet.insert(mapPoints[i]);

		//file << mapPoints[i]->id << endl;
		file << mapPoints[i]->x / scale << " " << mapPoints[i]->y / scale << " "
				<< mapPoints[i]->z / scale << endl;

		//save the points in the total output file
		pointsAndPath << mapPoints[i]->x / scale << " " << mapPoints[i]->y / scale << " "
			      << mapPoints[i]->z / scale <<  endl;

		//for (size_t i = 0; i < 9; i++)
		//	file << mapPoints[i]->cov[i] << " ";
		//file << endl;
	}
	file.close();
	cout << filePath << " has been saved!" << endl;

	//save camera poses
	for (int c = 0; c < numCams; c++) {
		sprintf(filePath, "%s/%d_campose.xyz", dirPath, c);
		file.open(filePath);
		if (!file)
			repErr("cannot open file '%s' to write!\n", filePath);

		size_t nCam = slam[c].m_camPos.size();
		//file << nCam << endl;
		for (CamPoseItem* cam = slam[c].m_camPos.first(); cam;
				cam = cam->next) {
		  //file << getFrameInVideo(c, cam->f) << endl;
		  //	for (size_t i = 0; i < 9; i++)
		  //		file << cam->R[i] << " ";
			file << cam->t[0] / scale << " " << cam->t[1] / scale << " " << -cam->t[2] / scale << endl;
			//negative X
			pointsAndPath << -cam->t[0] / scale << " " << cam->t[1] / scale << " "
				      << cam->t[2] / scale << endl;
		}
		file.close();
		cout << filePath << " has been saved!" << endl;
	}

	//save feature points
	for (int c = 0; c < numCams; c++) {
		sprintf(filePath, "%s/%d_featpts.txt", dirPath, c);
		file.open(filePath);
		if (!file)
			repErr("cannot open file '%s' to write!\n", filePath);

		int nf = 0;
		for (int f = 0; f <= curFrame; f++)
			if (f >= slam[c].m_camPos.first()->f)
				nf++;
		//file << nf << endl;

		for (int f = 0; f <= curFrame; f++) {
			if (f < slam[c].m_camPos.first()->f)
				continue;
			vector<FeaturePoint*> vecFeatPts;
			slam[c].m_featPts.getFrame(f, vecFeatPts);
			vector<FeaturePoint*> staticPts;
			for (size_t i = 0; i < vecFeatPts.size(); i++) {
				if (vecFeatPts[i]->mpt
						&& vecFeatPts[i]->mpt->isCertainStatic()) {
					staticPts.push_back(vecFeatPts[i]);
					assert(mptSet.count(vecFeatPts[i]->mpt) > 0);
				}
			}
			file << getFrameInVideo(c, f) << " " << staticPts.size() << endl;
			for (size_t i = 0; i < staticPts.size(); i++) {
				file << " " << staticPts[i]->x / scale << " "
				     << staticPts[i]->y / scale;//<< " " << staticPts[i]->z;
			}
			file << endl;
		}
		file.close();
		pointsAndPath.close();
		cout << filePath << " has been saved!" << endl;
	}
}

//Partnumber is in order to break up the parts based on the 
void CoSLAM::exportResults(const char timeStr[], double scale,  int partNumber) const {
  if(partNumber < 0) //regular output for the end of the system run
    exportResultsVer1(timeStr, scale);
  else
    savePointCloud(timeStr, scale, partNumber);//save the part of the map currently obtained
}

void CoSLAM::savePointCloud(const char timeStr[], double scale, int partNumber) const
{
  set<MapPoint*> mptSet;
	vector<MapPoint*> mapPoints;
	getAllStaticMapPoints(mapPoints);
  if (mapPoints.size() < 5) //If not enough points to warrant a new file, return early
    return;

	using namespace std;
    char resPath[256];
	char dirPath[256];

    sprintf(resPath, "%s/slam_results", getenv("HOME"));
	sprintf(dirPath, "%s/%s", resPath, timeStr);
    
    mkdir(resPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	char filePathA[256];
	char filePathB[256];

	//save map points
	std::ostringstream a;
	a << std::string(dirPath) << "/part" << (partNumber) << ".xyz";
	std::string tempAString = a.str();
	//save map and path points
	std::ostringstream b;
	b << std::string(dirPath) << "/pathAndPart" << (partNumber) << ".xyz";
	std::string tempBString = b.str();
	//a.append(partNumber);
	//a.append(".xyz");

	sprintf(filePathA, tempAString.c_str());
	sprintf(filePathB, tempBString.c_str());

	ofstream file(filePathA);
	ofstream fileB(filePathB);
	if (!file || !fileB)
		repErr("cannot open file to write!\n");

  //save feature points in scene
	for (size_t i = 0; i < mapPoints.size(); i++) {
		//test
		mptSet.insert(mapPoints[i]);

		//file << mapPoints[i]->id << endl;
		file << mapPoints[i]->x / scale << " " <<
		  mapPoints[i]->y / scale << " " <<
		  mapPoints[i]->z / scale << endl;

		fileB << mapPoints[i]->x / scale << " " <<
		  mapPoints[i]->y / scale << " " <<
		  mapPoints[i]->z / scale << endl;
		//for (size_t i = 0; i < 9; i++)
		//	file << mapPoints[i]->cov[i] << " ";
		//file << endl;
	}
	
	for(int c = 0; c < numCams; c++)
	{
	  for (CamPoseItem* cam = slam[c].m_camPos.first(); cam; cam = cam->next) 
	    {	      
	      //-X component
	      fileB << -cam->t[0] / scale << " " << cam->t[1] / scale << " "
		    << cam->t[2] / scale << endl;
	    }
	}

	file.close();
	fileB.close();
	//a = "Part ";
	//a.append(partNumber);
	//a.append(" has been saved");
	logInfo(tempAString.c_str());
}

// can output as .OFF to get colour in format X Y Z R G B (where rgb is from 0.0 to 1.0)
// ofstream CoSLAM::PLYOutput(const char timeStr[], int numPoints, double scale) const 
// {
//     using namespace std;
//     char resPath[256];
//     char dirPath[256];

//     sprintf(resPath, "%s/slam_results", getenv("HOME"));
// 	sprintf(dirPath, "%s/%s", resPath, timeStr);
    
//     mkdir(resPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
// 	mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

// 	char filePath[256];

// 	//save the information of the input video sequences 
// 	//and camera parameters
// 	sprintf(filePath, "%s/combinedOutput.ply", dirPath);
// 	ofstream file(filePath);
// 	if (!file)
// 		repErr("cannot open file %s to write!\n", filePath);

// 	for (int c = 0; c < numCams; c++) 
// 	{
// 		file << slam[c].videoFilePath << endl;
// 		for (size_t i = 0; i < 9; i++)
// 			file << slam[c].K[i] << " ";
// 		file << endl;
// 		for (size_t i = 0; i < 5; i++)
// 			file << slam[c].k_c.data[i] << " ";
// 		file << endl;
// 		file << slam[c].W << " " << slam[c].H << endl;
// 	}
// 	return file;
// }


/*USED TO DETERMINE THE SCALING FACTOR FOR THE SCANNED POINTS based on the read odometry files*/
//0.005 is a good threshold to use
double CoSLAM::determineScale(int camera, double thresholdValue)
{
  //STEP 1: detemine the length at which the first movement occurs on the input odometry file
  //set a threshold which counts as the zero movement readings (generally less than 10e-5)
  //scan through the entire file to find the stopping points - ignore the timing

  ifstream file(slam[camera].odoFilePath.c_str(), ios::in);
  std::string readLine;

  //the bounding commas on the value in the file
  int lastComma;
  int secondLast;
  int thirdLast;

  //the value of the timestep
  double odoValueX;
  double odoValueY;
  double odoValueZ;

  //the vector of displacement - will need to be set in case of turning
  double displacement[3] = {0};
  
  //only interested in the actual movement, not the rotation as of now
  //double[9] rotationMatrix = {0};

  //logic to see where it starts and stops
  bool moving = false;
  double stepDistance = 0.0;

  bool read = std::getline(file, readLine);

  while(read && ((moving && (stepDistance > thresholdValue)) || !moving))
  {
    stepDistance = 0.0;
    //now contains timesatmp, 9-elements of rotation matrix, and 3 elements of the translation vector 
    //translations are all in world space, as differentials. All from .CSV file, so comma separated
    lastComma = readLine.find_last_of(",");
    secondLast = readLine.find_last_of(",", lastComma - 1);
    thirdLast = readLine.find_last_of(",", secondLast - 1);
    //part of the substringing process, want the stuff before the last comma (-1) and then don't want to include the previous comma in the substring (+1)

    std::string tempX = readLine.substr(thirdLast + 1, secondLast - thirdLast - 1);
    std::string tempY = readLine.substr(secondLast + 1, lastComma - secondLast - 1);
    std::string tempZ = readLine.substr(lastComma + 1, string::npos);
    //logInfo(temp.c_str());
    //cut out the odoValue
    odoValueX = strtod(tempX.c_str(), NULL);
    odoValueY = strtod(tempY.c_str(), NULL);
    odoValueZ = strtod(tempZ.c_str(), NULL);

    //READ ROTATION MATRIX HERE - not needed now because the dx dy is in world space. Needs to be added if relative
    //rotationMatrix = reading more of the line and substring-ing a lot
    //odoValue * rotationMatrix //rotate the odoValue to fit into the total displacement

    //get the distance travelled by this step
    stepDistance = sqrt(odoValueX*odoValueX + odoValueY*odoValueY + odoValueZ*odoValueZ);
    
    //this counts as movement, and needs to be included
    //stops once moving is true (started movement) and odoValue is less than threshold
    if(stepDistance > thresholdValue)
    {
      moving = true;
      //add the values into the accumulated odoValue - for now only treat it as if it was in one direction
      displacement[0] += odoValueX;
      displacement[1] += odoValueY;
      displacement[2] += odoValueZ;
    }    
    //now we have the total odometry reading of displacement of the robot slam.[camera]
    read = std::getline(file, readLine);
  }

  //if no movement detected by the end of the file, exit
  if(displacement[0] < 0.005 && displacement[1] < 0.005 && displacement[2] < 0.005)
    throw SL_Exception();



  //now need to determine the offset from the SLAM information
  //get the SLAM movement information

  CamPoseItem* cameraPose =  slam[camera].m_camPos.first();
  std::vector<std::array<double, 3>> positions;
  std::array<double, 3> tempVector;
  double offset[3];

  offset[0] = cameraPose->t[0];
  offset[1] = cameraPose->t[1];
  offset[2] = cameraPose->t[2];

  //camPos.over_head.t contains the translation information from CoSLAM 
  //but as absolute from origin values, so subtract the previous position
  //to get the relative positioning
  for(int i = 0; cameraPose != NULL; cameraPose = cameraPose->next)
  {
    //parse through the list and add translation into vector for easier access
    tempVector[0] = cameraPose->t[0] - offset[0];
    tempVector[1] = cameraPose->t[1] - offset[1];
    tempVector[2] = cameraPose->t[2] - offset[2];
    
    positions.push_back(tempVector);

    offset[0] = cameraPose->t[0];
    offset[1] = cameraPose->t[1];
    offset[2] = cameraPose->t[2];
  }

  //filter the results - LPF with a 25 element kernel
  std::vector<std::array<double, 3>> filteredPositions;
  std::array<double, 3> average = {0};
  double averageMovement = 0;

  //
  double kernelSize = 25.0;
  double kernel[25][3] = {0};

  //loop through and replace the oldest value with the new data
  //I think the stuff in m_camPos is relative position data
  for(int i = 0; i < positions.size(); i++)
  {
    //replace the oldest element in the array 
    //stored in the i%kernelSize element of the array, int value for %
    kernel[i % ((int)kernelSize)][0] = positions[i][0];
    kernel[i % ((int)kernelSize)][1] = positions[i][1];
    kernel[i % ((int)kernelSize)][2] = positions[i][2];

    //now get average over the course of the kernel
    for(int j = 0; j < kernelSize; j++)
    {
      average[0] += kernel[j][0];
      average[1] += kernel[j][1];
      average[2] += kernel[j][2];
    }
    //add average into the filetered array
    average[0] = average[0] / kernelSize;
    average[1] = average[1] / kernelSize;
    average[2] = average[2] / kernelSize;
    filteredPositions.push_back(average);
    //get the average displacement for each step
    averageMovement += sqrt(average[0] * average[0] +
			    average[1] * average[1] +
			    average[2] * average[2]);
    average[0] = 0.0;
    average[1] = 0.0;
    average[2] = 0.0;
  }
  //getting the average movement size 
  averageMovement /= (positions.size() / 2);

  //now the filteredPositions contains fully filtered differential positioning
  //do similar movement detection as with CSV file, but use the average/2 as the 
  //threshold
  short movingSLAM = 0;
  double SLAMDisplacement[3] = {0};
  double stepSize = 0.0;

  //need to have at least 10 moving frames for it to count
  for(int i =0; i < filteredPositions.size() && (movingSLAM < 10 || stepSize > averageMovement) ; i++)
  {
    //cycle through the elements and identify the pieces which are moving
    stepSize = sqrt(filteredPositions[i][0] * filteredPositions[i][0] + 
		    filteredPositions[i][1] * filteredPositions[i][1] + 
		    filteredPositions[i][2] * filteredPositions[i][2]);

    //find the pieces which are moving
    if(stepSize > averageMovement)
    {
      movingSLAM++;
      SLAMDisplacement[0] += filteredPositions[i][0];
      SLAMDisplacement[1] += filteredPositions[i][1];
      SLAMDisplacement[2] += filteredPositions[i][2];
    }
    else if(movingSLAM < 10)  //if not moving, reset the counter
      movingSLAM = 0;
    else  //decrease the counter. need 10 still frames to count as stop
      movingSLAM--;
    
  }

  //if no movement detected by the end of the SLAM, exit
  if(abs(SLAMDisplacement[0]) < averageMovement || abs(SLAMDisplacement[1]) < averageMovement || abs(SLAMDisplacement[2]) < averageMovement)
    throw SL_Exception();
  
  //total sizes of the first movement are stored in SLAMDisplacement and displacement
  //need abs size difference between readings -> need magnitudes
  double odoReading = sqrt(displacement[0] * displacement[0] +
			   displacement[1] * displacement[1] +
			   displacement[2] * displacement[2]);

  double SLAMReading = sqrt(SLAMDisplacement[0] * SLAMDisplacement[0] +
			    SLAMDisplacement[1] * SLAMDisplacement[1] +
			    SLAMDisplacement[2] * SLAMDisplacement[2]);

  //these readings now have the absolute sizes of the first movements from both
  //scaling will determine the coefficient which the points need to be multiplied by
  
  //deal with edge case where the scaling calculation failed -> no scaling
  if(odoReading < 0.005 || SLAMReading < 0.005)
    return 1.0;

  logInfo("\nSCALING FACTOR: %d \n", SLAMReading / odoReading);

  //scaling factor: used by taking CoSLAM.measurements / value
  return SLAMReading / odoReading;
}

void CoSLAM::saveCurrentImages(const char* dirPath) const {
#ifdef WIN32
	mkdir(dirPath);
#else
	mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
	for (int i = 0; i < numCams; i++) {
		char filePath[1024];
		sprintf(filePath, "%s/cam_%d.pgm", dirPath, i);
		savePGM(slam[i].m_img, filePath);
	}
	logInfo("save images OK!\n");
}
