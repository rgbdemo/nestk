/****************************************************************************
*                                                                           *
*   Nite 1.2 - Multi-Process Server Sample                                  *
*                                                                           *
*   Author:     Oz Magal                                                    *
*                                                                           *
****************************************************************************/

/****************************************************************************
*                                                                           *
*   Nite 1.2	                                                            *
*   Copyright (C) 2006 PrimeSense Ltd. All Rights Reserved.                 *
*                                                                           *
*   This file has been provided pursuant to a License Agreement containing  *
*   restrictions on its use. This data contains valuable trade secrets      *
*   and proprietary information of PrimeSense Ltd. and is protected by law. *
*                                                                           *
****************************************************************************/

#include <stdio.h>
#include <XnOpenNI.h>
#include <XnVSessionManager.h>
#include <XnVPointControl.h>
#include <XnVMultiProcessFlowServer.h>
#include <XnVHandPointContext.h>
#include "kbhit.h"
#include "signal_catch.h"

void XN_CALLBACK_TYPE SessionStart(const XnPoint3D& ptPosition, void* UserCxt)
{
	printf("session start: (%f,%f,%f)\n", ptPosition.X, ptPosition.Y, ptPosition.Z);
}

void XN_CALLBACK_TYPE SessionEnd(void* UserCxt)
{
	printf("session end!\n");
}

void XN_CALLBACK_TYPE FocusStart(const XnChar* strFocus, const XnPoint3D& ptPosition, XnFloat fProgress, void* UserCxt)
{
	printf("Focus gesture start detected - %5.2f!\n", fProgress);
}

xn::Context g_Context;
XnVSessionManager* g_pSessionManager = NULL;
XnBool g_bQuit = false;

void CleanupExit()
{
	if (NULL != g_pSessionManager) {
		delete g_pSessionManager;
		g_pSessionManager = NULL;
	}

	g_Context.Shutdown();

	exit (1);
}

#define SAMPLE_XML_FILE "../../Data/Sample-Tracking.xml"

int main(int argc, char *argv[])
{
	if (argc < 2) {
		printf("usage: %s section_name\n", argv[0]);
		return 0;
	}

	printf("\n");

	char * strSectionName = argv[1];

	// Initialize device
	XnStatus rc = g_Context.InitFromXmlFile(SAMPLE_XML_FILE);
	if (rc != XN_STATUS_OK)
	{
		printf("Couldn't initialize: %s\n", xnGetStatusString(rc));
		return 1;
	}

	// Create and initialize point tracker
	g_pSessionManager = new XnVSessionManager();
	rc = g_pSessionManager->Initialize(&g_Context, "Wave", "RaiseHand");
	if (rc != XN_STATUS_OK)
	{
		printf("Session Manager couldn't initialize: %s\n", xnGetStatusString(rc));
		CleanupExit();
	}

	g_Context.StartGeneratingAll();

	g_pSessionManager->RegisterSession(NULL, &SessionStart, &SessionEnd, &FocusStart);

	CatchSignals(&g_bQuit);

	// static variable destructors will be called on exit(), while
	// automatic lifetime variables (regular variables on the stack) will not
	// have their destructors called (since stack unwinding isn't performed)
	static XnVMultiProcessFlowServer writer(strSectionName);
	rc = writer.Initialize();
	if (XN_STATUS_OK != rc) {
		printf("Failed to initialize server: %s\n", xnGetStatusString(rc));
		CleanupExit();
	}

	g_pSessionManager->RegisterSession(&writer);
	g_pSessionManager->AddListener(&writer);
	//g_pSessionManager->AddListener(&print);

	printf("Server running in section <%s>. Hit any key to exit.\n", strSectionName);
	while((!g_bQuit) && (!_kbhit()))
	{
		g_Context.WaitAndUpdateAll();

		// Update point tracker
		g_pSessionManager->Update(&g_Context);

		// Publish current state
//		rc = writer.WriteState();

		if (XN_STATUS_OK != rc) {
			// Error
			break;
		}
	}

	g_pSessionManager->UnregisterSession(&writer);
	CleanupExit();
}