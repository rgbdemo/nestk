/****************************************************************************
*                                                                           *
*   Nite 1.3 - Boxes Sample                                                 *
*                                                                           *
*   Author:     Oz Magal                                                    *
*                                                                           *
****************************************************************************/

/****************************************************************************
*                                                                           *
*   Nite 1.3	                                                            *
*   Copyright (C) 2006 PrimeSense Ltd. All Rights Reserved.                 *
*                                                                           *
*   This file has been provided pursuant to a License Agreement containing  *
*   restrictions on its use. This data contains valuable trade secrets      *
*   and proprietary information of PrimeSense Ltd. and is protected by law. *
*                                                                           *
****************************************************************************/

#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnVHandPointContext.h>
#include <XnVSessionManager.h>
#include <XnVFlowRouter.h>
#include <XnVSwipeDetector.h>
#include <XnVSelectableSlider1D.h>
#include <XnVSteadyDetector.h>
#include <XnVBroadcaster.h>
#include <XnVPushDetector.h>

#ifdef USE_GLUT
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
        #include <GLUT/glut.h>
#else
        #include <GL/glut.h>
#endif
#else
#include "opengles.h"
#include "kbhit.h"
#endif
#include "signal_catch.h"

#ifndef USE_GLUT
static EGLDisplay display = EGL_NO_DISPLAY;
static EGLSurface surface = EGL_NO_SURFACE;
static EGLContext context = EGL_NO_CONTEXT;
#endif

// Drawing functions
void DrawLine(XnFloat fMinX, XnFloat fMinY, XnFloat fMinZ,
			  XnFloat fMaxX, XnFloat fMaxY, XnFloat fMaxZ,
			  int width, double r = 1, double g = 1, double b = 1)
{
	#ifdef USE_GLUT

	glLineWidth(width);
	glBegin(GL_LINES);
	glColor3f(r, g, b);
	glVertex3f(fMinX, fMinY, fMinZ);
	glVertex3f(fMaxX, fMaxY, fMaxZ);
	glEnd();

	#else

	const GLubyte ind[2] = {0, 1};
	GLfloat verts[6] = { fMinX, fMinY, fMinZ, fMaxX, fMaxY, fMaxZ };
	glColor4f(r,g,b,1.0f);
	glVertexPointer(3, GL_FLOAT, 0, verts);
	glLineWidth(width);
	glDrawArrays(GL_LINES, 0, 2);
	glFlush();

	#endif
}

void DrawFrame(const XnPoint3D& ptMins, const XnPoint3D& ptMaxs, int width, double r, double g, double b)
{
	XnPoint3D ptTopLeft = ptMins;
	XnPoint3D ptBottomRight = ptMaxs;

	// Top line
	DrawLine(ptTopLeft.X, ptTopLeft.Y, 0,
		ptBottomRight.X, ptTopLeft.Y, 0,
		width, r, g, b);
	// Right Line
	DrawLine(ptBottomRight.X, ptTopLeft.Y, 0,
		ptBottomRight.X, ptBottomRight.Y,0,
		width, r, g, b);
	// Bottom Line
	DrawLine(ptBottomRight.X, ptBottomRight.Y,0,
		ptTopLeft.X, ptBottomRight.Y,0,
		width, r, g, b);
	// Left Line
	DrawLine(ptTopLeft.X, ptBottomRight.Y,0,
		ptTopLeft.X, ptTopLeft.Y,0,
		width, r, g, b);

}

// Define the MyBox object
#include "MyBox.h"


XnVSessionManager* g_pSessionManager = NULL;
XnVFlowRouter* g_pMainFlowRouter;

XnVSelectableSlider1D* g_pMainSlider;

MyBox* g_pBox[3];

XnBool g_bActive = false;
XnBool g_bIsInput = false;
XnBool g_bInSession = false;
XnFloat g_fValue = 0.5f;

// Main slider
void XN_CALLBACK_TYPE MainSlider_OnHover(XnInt32 nItem, void* cxt)
{
	for (int i = 0; i < 3; i++)
		g_pBox[i]->SetFrameMode(i==nItem?MyBox::FrameOver:MyBox::FrameNone);
}

void XN_CALLBACK_TYPE MainSlider_OnSelect(XnInt32 nItem, XnVDirection dir, void* cxt)
{
	if (nItem == -1)
	{
	}
	else if (dir == DIRECTION_UP || dir == DIRECTION_FORWARD)
	{
		g_pMainFlowRouter->SetActive(g_pBox[nItem]);
	}
	else
	{
		printf("Slider: Bad direction for selection: %s\n", XnVDirectionAsString(dir));
	}
}

void XN_CALLBACK_TYPE MainSlider_OnValueChange(XnFloat fValue, void* cxt)
{
	g_bActive = true;
	g_bIsInput = true;
	g_fValue = fValue;
}
void XN_CALLBACK_TYPE MainSlider_OnActivate(void* cxt)
{
	g_bActive = true;
	g_bIsInput = false;
}
void XN_CALLBACK_TYPE MainSlider_OnDeactivate(void* cxt)
{
	g_bActive = false;
	g_bIsInput = false;
}
void XN_CALLBACK_TYPE MainSlider_OnPrimaryCreate(const XnVHandPointContext* hand, const XnPoint3D& ptFocus, void* cxt)
{
	g_bIsInput = true;
}
void XN_CALLBACK_TYPE MainSlider_OnPrimaryDestroy(XnUInt32 nID, void* cxt)
{
	g_bIsInput = false;
}

#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480

// More drawing
void DrawSlider()
{
	if (!g_bInSession)
		return;

	XnPoint3D ptMin = xnCreatePoint3D(50, GL_WIN_SIZE_Y-380, 0);
	XnPoint3D ptMax = xnCreatePoint3D(650, GL_WIN_SIZE_Y-460, 0);

	XnDouble r, g, b;
	XnBool bDrawCursor = false;

	if (!g_bActive)
	{
		r = g = b = 1;
	}
	else if (!g_bIsInput)
	{
		r = g = b = 0.5;
	}
	else
	{
		r = b = 0;
		g = 1;
		bDrawCursor = true;
	}

	DrawFrame(ptMin, ptMax, 20, r, g, b);

	if (bDrawCursor)
	{
		DrawLine(600*g_fValue+50, GL_WIN_SIZE_Y-380, 0,
				600*g_fValue+50, GL_WIN_SIZE_Y-460, 0,
				30, 1, 1, 0);
	}

}

xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;


XnBool g_bQuit = false;
void CleanupExit()
{
	if (NULL != g_pSessionManager) {
		delete g_pSessionManager;
		g_pSessionManager = NULL;
	}

	delete g_pMainFlowRouter;
	delete g_pMainSlider;
	delete g_pBox[0];
	delete g_pBox[1];
	delete g_pBox[2];

	g_Context.Shutdown();

	exit (1);
}

// this function is called each frame
void glutDisplay (void)
{
	// Read next available data
	g_Context.WaitAndUpdateAll();

	// Process the data
	g_pSessionManager->Update(&g_Context);

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	#ifdef USE_GLUT
	glOrtho(0, GL_WIN_SIZE_X, 0, GL_WIN_SIZE_Y, -1.0, 1.0);
	#else
	glOrthof(0, GL_WIN_SIZE_X, 0, GL_WIN_SIZE_Y, -1.0, 1.0);
	#endif

	glDisable(GL_TEXTURE_2D);

	// Draw the MyBoxes
	g_pBox[0]->Draw();
	g_pBox[1]->Draw();
	g_pBox[2]->Draw();

	// Draw the slider
	DrawSlider();

	#ifdef USE_GLUT
	glutSwapBuffers();
	#endif
}

#ifdef USE_GLUT
void glutIdle (void)
{
	if (g_bQuit) {
		CleanupExit();
	}

	// Display the frame
	glutPostRedisplay();
}
void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		CleanupExit();
	}
}
void glInit (int * pargc, char ** argv)
{
	glutInit(pargc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("PrimeSense Nite Boxes Sample");
	//glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
}
#endif

// Callback for the MyBox
void XN_CALLBACK_TYPE MyBox_Leave(void* UserContext)
{
	g_pMainFlowRouter->SetActive(g_pMainSlider);
}

void XN_CALLBACK_TYPE SessionStart(const XnPoint3D& pFocus, void* UserCxt)
{
	g_bInSession = true;
	g_pMainFlowRouter->SetActive(g_pMainSlider);
}
void XN_CALLBACK_TYPE SessionEnd(void* UserCxt)
{
	g_bInSession = false;
	g_pMainFlowRouter->SetActive(NULL);
}

#define SAMPLE_XML_PATH "config/Sample-Tracking.xml"

#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
{																\
	printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
	return rc;													\
}

#define CHECK_ERRORS(rc, errors, what)		\
	if (rc == XN_STATUS_NO_NODE_PRESENT)	\
{										\
	XnChar strError[1024];				\
	errors.ToString(strError, 1024);	\
	printf("%s\n", strError);			\
	return (rc);						\
}

int main(int argc, char **argv)
{
	XnStatus rc = XN_STATUS_OK;
	xn::EnumerationErrors errors;

	rc = g_Context.InitFromXmlFile(SAMPLE_XML_PATH);
	CHECK_ERRORS(rc, errors, "InitFromXmlFile");
	CHECK_RC(rc, "InitFromXml");

	rc = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(rc, "Find depth generator");

	// Create and initialize point tracker
	g_pSessionManager = new XnVSessionManager;
	rc = g_pSessionManager->Initialize(&g_Context, "Wave", "RaiseHand");
	if (rc != XN_STATUS_OK)
	{
		printf("Couldn't initialize the Session Manager: %s\n", xnGetStatusString(rc));
		delete g_pSessionManager;
		return rc;
	}

	g_pSessionManager->RegisterSession(NULL, &SessionStart, &SessionEnd);

	// Start catching signals for quit indications
	CatchSignals(&g_bQuit);

	// Create and initialize the main slider
	g_pMainSlider = new XnVSelectableSlider1D(3);
	g_pMainSlider->RegisterItemHover(NULL, &MainSlider_OnHover);
	g_pMainSlider->RegisterItemSelect(NULL, &MainSlider_OnSelect);
	g_pMainSlider->RegisterActivate(NULL, &MainSlider_OnActivate);
	g_pMainSlider->RegisterDeactivate(NULL, &MainSlider_OnDeactivate);
	g_pMainSlider->RegisterPrimaryPointCreate(NULL, &MainSlider_OnPrimaryCreate);
	g_pMainSlider->RegisterPrimaryPointDestroy(NULL, &MainSlider_OnPrimaryDestroy);
	g_pMainSlider->RegisterValueChange(NULL, &MainSlider_OnValueChange);
	g_pMainSlider->SetValueChangeOnOffAxis(true);

	// Creat the flow manager
	g_pMainFlowRouter = new XnVFlowRouter;

	// Connect flow manager to the point tracker
	g_pSessionManager->AddListener(g_pMainFlowRouter);

	// Create the MyBox objects
	XnPoint3D ptMax, ptMin;
	ptMax.Z = ptMin.Z = 0;
	ptMax.Y = GL_WIN_SIZE_Y-50;
	ptMin.Y = GL_WIN_SIZE_Y-300;

	ptMax.X = 50; ptMin.X = 240;
	g_pBox[0] = new MyBox(ptMax, ptMin);
	ptMax.X = 260; ptMin.X = 450;
	g_pBox[1] = new MyBox(ptMax, ptMin);
	ptMax.X = 470; ptMin.X = 650;
	g_pBox[2] = new MyBox(ptMax, ptMin);

	// Register callback to the MyBox objects for their Leave event.
	g_pBox[0]->RegisterLeave(NULL, &MyBox_Leave);
	g_pBox[1]->RegisterLeave(NULL, &MyBox_Leave);
	g_pBox[2]->RegisterLeave(NULL, &MyBox_Leave);

	g_Context.StartGeneratingAll();

	#ifdef USE_GLUT

	glInit(&argc, argv);
	glutMainLoop();

	#else

	if (!opengles_init(GL_WIN_SIZE_X, GL_WIN_SIZE_Y, &display, &surface, &context))
	{
		printf("Error initing opengles\n");
		CleanupExit();
	}

	glDisable(GL_DEPTH_TEST);
//	glEnable(GL_TEXTURE_2D);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	while ((!_kbhit()) && (!g_bQuit))
	{
		glutDisplay();
		eglSwapBuffers(display, surface);
	}

	opengles_shutdown(display, surface, context);

	CleanupExit();

	#endif
}
