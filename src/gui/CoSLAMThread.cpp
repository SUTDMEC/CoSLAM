/*
 * CoSLAMThread.cpp
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#include "CoSLAMThread.h"
#include "app/SL_GlobParam.h"
#include "app/SL_CoSLAM.h"
#include "tools/SL_Tictoc.h"
#include "tools/SL_Timing.h"

#include "gui/MyApp.h"

#ifdef WIN32
#include <io.h>  
#include <process.h>  
#else
#include <unistd.h>
#endif

DEFINE_EVENT_TYPE(EventUpdateViews);

int CoSLAMThread::partNum = 0;

CoSLAMThread::CoSLAMThread() {

}
CoSLAMThread::~CoSLAMThread() {
}
void updateDisplayData() {
	CoSLAM& coSLAM = MyApp::coSLAM;
	for (int i = 0; i < coSLAM.numCams; i++) {
		if (!MyApp::bBusyDrawingVideo[i])
			MyApp::videoWnd[i]->copyDataForDisplay();
	}
	//	MyApp::s_mutexUpdateDispData.Unlock();
	if (!MyApp::bBusyDrawingModel) {
		MyApp::modelWnd1->copyDispData();
		MyApp::modelWnd2->copyDispData();
	}
}
void redrawAllViews() {
	MyApp::redrawViews();
}
CoSLAMThread::ExitCode CoSLAMThread::Entry() {
	CoSLAM& coSLAM = MyApp::coSLAM;
	/////////////////////////1.GPU initilization/////////////////////////
	//initialization for CG;
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow(" ");
	glutHideWindow();

	glewInit();

	V3D_GPU::Cg_ProgramBase::initializeCg();

	//set the default scaling factor for the points
	//if no scaling can be determined by the determineScale function, then
	//don't scale the points
	double scale = 1.0;

	//////////////////////////2.read video information//////////////////
	try {
		for(int c = 0; c < coSLAM.numCams; c++){
			coSLAM.slam[c].videoReader = &MyApp::aviReader[c];
		}
		
		coSLAM.init();
		MyApp::bInitSucc = true;
		logInfo("Loading video sequences.. OK!\n");
	} catch (SL_Exception& e) {
		logInfo(e.what());
#ifdef WIN32
		wxMessageBox(e.what());
#endif
		return 0;
	}

	//notify the GUI thread to create GUIs
	MyApp::broadcastCreateGUI();

	//wait for the accomplishment of creating GUIs
	MyApp::waitCreateGUI();

	for (int i = 0; i < coSLAM.numCams; i++)
		MyApp::videoWnd[i]->setSLAMData(i, &coSLAM);

	MyApp::modelWnd1->setSLAMData(&coSLAM);
	MyApp::modelWnd2->setSLAMData(&coSLAM);


	/* start the SLAM process*/
	try {
		coSLAM.readFrame();
		//copy the data to buffers for display
		updateDisplayData();
		//initialise the map points
		tic();
		coSLAM.initMap();
		toc();
	}catch(...)
	  {
	    logInfo("\nError starting SLAM\n");
	  }
	int i = 1;

	try{


	  for(int k = 0; i < Param::nTotalFrame; i++){
			while (MyApp::bStop) {/*stop*/
			}
	  
			TimeMeasurer tmPerStep;
			tmPerStep.tic();

			coSLAM.grabReadFrame();
			coSLAM.featureTracking();
			coSLAM.poseUpdate();
			coSLAM.cameraGrouping();
            
			//existing 3D to 2D points robust
			coSLAM.activeMapPointsRegister(Const::PIXEL_ERR_VAR);

			TimeMeasurer tmNewMapPoints;
			tmNewMapPoints.tic();

			coSLAM.genNewMapPoints();
			coSLAM.m_tmNewMapPoints = tmNewMapPoints.toc();

			//point registration
			coSLAM.currentMapPointsRegister(Const::PIXEL_ERR_VAR,
					i % 50 == 0 ? true : false);

			coSLAM.storeDynamicPoints();

			updateDisplayData();
			redrawAllViews();

			coSLAM.m_tmPerStep = tmPerStep.toc();
			Sleep(50);

            cout << "f:" << coSLAM.curFrame << "/" << Param::nTotalFrame << endl;
		}



		cout << " the result is saved at " << MyApp::timeStr << endl;

		//SCALE THE OUTPUT FILES SO THEY ARE WITH SAME SCALE AS THE TRAVELLING OF THE 
		//ODOMETRY FILES - SENSE THE BLOCKS OF MOVEMENT (ADD UP THE ODO FILES TO THE END OF THE
		//MOVEMENT WITH SMALL DISPLACEMENTS), AND COMPARE AGAINST THE LPF OF THE CAMPOSE 
		//only need to compare against one of the cameras, as both are related. Use the 1st camera (0)
		scale = coSLAM.determineScale(0, 0.001);

		coSLAM.exportResults(MyApp::timeStr, scale, -1); //Do default exporting of scan with explicit scale
		logInfo("slam finished\n");




	} catch (SL_Exception& e) {

	  //IF THERE ARE REGISTERED POINTS, THEN SAVE POINTS BEFORE RESARTING
	  coSLAM.exportResults(MyApp::timeStr, scale, CoSLAMThread::partNum++);

	  logInfo("\nSL_Exception:\n");
	  logInfo(e.what());
	} catch (std::exception& e) {
#ifdef WIN32
		wxMessageBox(e.what());
#endif
		logInfo("%s\n", e.what());
		logInfo("slam failed!\n");
#ifdef WIN32
		wxMessageBox(e.what());
#endif
	}
	logInfo("\nslam stopped!\n");
	return 0;
}
