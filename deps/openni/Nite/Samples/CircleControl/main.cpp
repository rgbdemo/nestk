/****************************************************************************
*                                                                           *
*   Nite 1.3 - Circle Sample                                                *
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

//-----------------------------------------------------------------------------
// Headers
//-----------------------------------------------------------------------------

#include <XnOpenNI.h>
#include <XnVHandPointContext.h>
#include <XnVSessionManager.h>
#include <XnVCircleDetector.h>
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

#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------

xn::Context g_Context;
XnVSessionManager * g_pSessionManager = NULL;
XnVCircleDetector*  g_pCircle = NULL;

#ifndef USE_GLUT
static EGLDisplay display = EGL_NO_DISPLAY;
static EGLSurface surface = EGL_NO_SURFACE;
static EGLContext context = EGL_NO_CONTEXT;
#endif

// visual feedback related
bool g_bDrawFrame = false;
float g_fFrameR = 0;
float g_fFrameG = 0;
float g_fFrameB = 0;

bool g_bDrawCircle = false;
float g_fCircleAngle = 0; // radians
float g_fCircleR = 0;
float g_fCircleG = 0;
float g_fCircleB = 0;
float g_fCircleLineR = 0;
float g_fCircleLineG = 0;
float g_fCircleLineB = 0;

XnBool g_bQuit = false;

//-----------------------------------------------------------------------------
// Drawing functions
//-----------------------------------------------------------------------------

void DrawLine(const XnPoint3D& ptMins, const XnPoint3D& ptMaxs, int width, double r = 1, double g = 1, double b = 1)
{
	
	#ifdef USE_GLUT

	glLineWidth(width);
	glBegin(GL_LINES);
	glColor3f(r, g, b);
	glVertex3f(ptMins.X, ptMins.Y, ptMins.Z);
	glVertex3f(ptMaxs.X, ptMaxs.Y, ptMaxs.Z);
	glEnd();

	#else

	const GLubyte ind[2] = {0, 1};
	GLfloat verts[6] = { ptMins.X, ptMins.Y, ptMins.Z, ptMaxs.X, ptMaxs.Y, ptMaxs.Z };
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
	DrawLine(xnCreatePoint3D(ptTopLeft.X, ptTopLeft.Y, 0),
		xnCreatePoint3D(ptBottomRight.X, ptTopLeft.Y, 0),
		width, r, g, b);
	// Right Line
	DrawLine(xnCreatePoint3D(ptBottomRight.X, ptTopLeft.Y, 0),
		xnCreatePoint3D(ptBottomRight.X, ptBottomRight.Y,0),
		width, r, g, b);
	// Bottom Line
	DrawLine(xnCreatePoint3D(ptBottomRight.X, ptBottomRight.Y,0),
		xnCreatePoint3D(ptTopLeft.X, ptBottomRight.Y,0),
		width, r, g, b);
	// Left Line
	DrawLine(xnCreatePoint3D(ptTopLeft.X, ptBottomRight.Y,0),
		xnCreatePoint3D(ptTopLeft.X, ptTopLeft.Y,0),
		width, r, g, b);
}

void DrawCircle(const XnPoint3D& ptCenter, double radius, int width, double r, double g, double b)
{
	const int n=50;
	glColor4f(r,g,b,1.0f);
	glLineWidth(width);
	double theta=0, dtheta = 2*XnVMathCommon::PI/n;
	XnV3DVector vecCurr;

	#ifdef USE_GLUT

        glBegin(GL_LINE_LOOP);
        for(int i=0; i<n; i++, theta+=dtheta) {
		vecCurr = XnV3DVector(ptCenter) + radius * XnV3DVector(sin(theta), cos(theta), 0);
		glVertex3f(vecCurr.X, vecCurr.Y, vecCurr.Z);
        }
        glEnd();

	#else

	GLfloat verts[n * 3];
	int curr = 0;

	for(int i=0; i<n; i++, theta+=dtheta, curr += 3) 
	{
		vecCurr = XnV3DVector(ptCenter) + radius * XnV3DVector(sin(theta), cos(theta), 0);
		verts[curr] = vecCurr.X;
		verts[curr+1] = vecCurr.Y;
		verts[curr+2] = vecCurr.Z;
	}

	glVertexPointer(3, GL_FLOAT, 0, verts);
	glDrawArrays(GL_LINE_LOOP, 0, n);
	glFlush();

	#endif
}

//-----------------------------------------------------------------------------
// Internal functions
//-----------------------------------------------------------------------------

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
	glLoadIdentity();

	#ifdef USE_GLUT
	//glOrtho(0, GL_WIN_SIZE_X, 0, GL_WIN_SIZE_Y, -1.0, 1.0);
	glOrtho(0, 1.0, 1.0, 0, -1.0, 1.0);
	#else
	//glOrthof(0, GL_WIN_SIZE_X, 0, GL_WIN_SIZE_Y, -1.0, 1.0);
	glOrthof(0, 1.0, 1.0, 0, -1.0, 1.0);
	#endif

	// draw
	if (g_bDrawFrame)
	{
		DrawFrame(xnCreatePoint3D(0.0, 0.0, 0), xnCreatePoint3D(1.0, 1.0, 0), 8, g_fFrameR, g_fFrameG, g_fFrameB);
	}

	if (g_bDrawCircle)
	{
		XnV3DVector vec(0.5, 0.5, 0);
		vec += XnV3DVector(sin(g_fCircleAngle), -cos(g_fCircleAngle), 0) * 0.3;
		DrawCircle(xnCreatePoint3D(0.5, 0.5, 0), 0.3, 3, g_fCircleR, g_fCircleG, g_fCircleB);
		DrawLine(xnCreatePoint3D(0.5, 0.5, 0), vec, 4, g_fCircleLineR, g_fCircleLineG, g_fCircleLineB);
	}

	// flip surfaces
	#ifdef USE_GLUT
	glutSwapBuffers();
	#else
	eglSwapBuffers(display, surface);
	#endif
}

void CleanupExit()
{
	if (NULL != g_pSessionManager) {
		delete g_pSessionManager;
		g_pSessionManager = NULL;
	}
	if (NULL != g_pCircle)
	{
		delete g_pCircle;
		g_pCircle = NULL;
	}

	g_Context.Shutdown();

	exit (1);
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
	glutCreateWindow ("PrimeSense Circle Control Sample");
	//glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	//glEnable(GL_TEXTURE_2D);
}
#endif

void SetVisualFeedbackFrame(bool bDisplay, float r, float g, float b)
{
	g_bDrawFrame = bDisplay;
	g_fFrameR = r;
	g_fFrameG = g;
	g_fFrameB = b;
}

void SetCircle(bool bDisplay, float fAngle)
{
	g_bDrawCircle = bDisplay;
	g_fCircleAngle = fAngle;
}

void SetCircleColor(float r, float g, float b)
{
	g_fCircleR = r;
	g_fCircleG = g;
	g_fCircleB = b;
}	

void SetCircleLineColor(float r, float g, float b)
{
	g_fCircleLineR = r;
	g_fCircleLineG = g;
	g_fCircleLineB = b;
}	


void XN_CALLBACK_TYPE SessionStart(const XnPoint3D& ptFocus, void *pUserCxt)
{
	SetVisualFeedbackFrame(true, 0.1, 1.0, 0.1);
}

void XN_CALLBACK_TYPE SessionEnd(void *pUserCxt)
{
	SetVisualFeedbackFrame(false, 0, 0, 0);
}

void XN_CALLBACK_TYPE CircleCB(XnFloat fTimes, XnBool bConfident, const XnVCircle* pCircle, void* pUserCxt)
{
	SetCircleLineColor(1, 0.1, 0.1);
	SetCircleColor(0.2, 0.2, 1.0);
	SetCircle(true, fmod((double)fTimes, 1.0) * 2 * XnVMathCommon::PI);
}

void XN_CALLBACK_TYPE NoCircleCB(XnFloat fLastValue, XnVCircleDetector::XnVNoCircleReason reason, void * pUserCxt)
{
	SetCircleLineColor(0.7,0.7,0.7);
	SetCircleColor(1, 1, 1);
}

void XN_CALLBACK_TYPE Circle_PrimaryCreate(const XnVHandPointContext *cxt, const XnPoint3D& ptFocus, void * pUserCxt)
{
	SetVisualFeedbackFrame(true, 0.1, 1, 0.1);
}

void XN_CALLBACK_TYPE Circle_PrimaryDestroy(XnUInt32 nID, void * pUserCxt)
{
	SetVisualFeedbackFrame(true, 0.2, 0.7, 0.2);
}

#define SAMPLE_XML_FILE "config/NestkConfig.xml"

void XN_CALLBACK_TYPE BodyEventDetectorPush_Pushed(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  printf("PUSH DETECTED\n");
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(int argc, char ** argv)
{
	// Configure
	XnStatus rc = g_Context.InitFromXmlFile(SAMPLE_XML_FILE);
	if (rc != XN_STATUS_OK)
	{
		printf("Couldn't initialize from file: %s\n", xnGetStatusString(rc));
		return 1;
	}

	// Create and initialize point tracker
	g_pSessionManager = new XnVSessionManager();
	rc = g_pSessionManager->Initialize(&g_Context, "Wave", "RaiseHand");
	if (rc != XN_STATUS_OK)
	{
		printf("Couldn't initialize the Session Manager: %s\n", xnGetStatusString(rc));
		CleanupExit();
	}

	g_pSessionManager->RegisterSession(NULL, &SessionStart, &SessionEnd);

	// Start catching signals for quit indications
	CatchSignals(&g_bQuit);

	// init and register circle control
#if 0
	g_pCircle = new XnVCircleDetector;
	g_pCircle->RegisterCircle(NULL, &CircleCB);
	g_pCircle->RegisterNoCircle(NULL, &NoCircleCB);
	g_pCircle->RegisterPrimaryPointCreate(NULL, &Circle_PrimaryCreate);
	g_pCircle->RegisterPrimaryPointDestroy(NULL, &Circle_PrimaryDestroy);
	g_pSessionManager->AddListener(g_pCircle);
#endif

  XnVPushDetector* m_push_detector = new XnVPushDetector;
  m_push_detector->RegisterPush(0, &BodyEventDetectorPush_Pushed);
  g_pSessionManager->AddListener(m_push_detector);

	SetCircle(true, 0);
	SetCircleColor(1,1,1);
	SetCircleLineColor(0.7,0.7,0.7);

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
	//glEnable(GL_TEXTURE_2D);
	
	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	while ((!_kbhit()) && (!g_bQuit))
	{
		glutDisplay();
	}

	opengles_shutdown(display, surface, context);
	
	CleanupExit();

	#endif
}
