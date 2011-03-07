/****************************************************************************
*                                                                           *
*   Nite 1.3 - Track Pad Sample                                             *
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
#include <XnVSelectableSlider2D.h>

XnBool g_bActive = FALSE;
XnBool g_bIsInput = FALSE;
XnBool g_bInSession = FALSE;

XnBool g_bIsPushed = FALSE;
const XnUInt32 XN_PUSH_DISPLAY_FRAMES = 30;
static XnUInt32 g_nCurrentFrame = 0;

XnFloat g_fXValue = 0.5f;
XnFloat g_fYValue = 0.5f;

XnUInt32 g_nXIndex = 0;
XnUInt32 g_nYIndex = 0;

XnUInt32 g_TP_XDim = 4;
XnUInt32 g_TP_YDim = 9;

const XnUInt32 XN_MIN_X_DIM = 2;
const XnUInt32 XN_MAX_X_DIM = 12;
const XnUInt32 XN_MIN_Y_DIM = 2;
const XnUInt32 XN_MAX_Y_DIM = 12;

xn::Context g_Context;
XnVSelectableSlider2D* g_pTrackPad = NULL;
XnVSessionManager* g_pSessionManager = NULL;

XnCallbackHandle g_nItemHoverHandle = NULL;
XnCallbackHandle g_nItemSelectHandle = NULL;
XnCallbackHandle g_nValueChangeHandle = NULL;

XnCallbackHandle g_nPrimaryCreateHandle = NULL;
XnCallbackHandle g_nPrimaryDestroyHandle = NULL;

XnUInt32 g_TrackPadHandle = 0;

XnBool g_isPrintItemHover = TRUE;
XnBool g_isPrintValueChange = FALSE;
XnBool g_isInputStarted = FALSE;

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
	GLfloat verts[6] = { ptMins.X(), ptMins.Y(), ptMins.Z(), ptMaxs.X(), ptMaxs.Y(), ptMaxs.Z() };
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


struct 
{
	XnInt32 X, Y;
} CurrentItem;

void XN_CALLBACK_TYPE TrackPad_ValueChange(XnFloat fXValue, XnFloat fYValue, void* cxt)
{
  if(TRUE == g_isPrintValueChange)
	  printf("Value changed: %f, %f\n", fXValue, fYValue);
  g_fXValue = fXValue;
  g_fYValue = fYValue;
}
void XN_CALLBACK_TYPE TrackPad_ItemHover(XnInt32 nXItem, XnInt32 nYItem, void* cxt)
{
  if(TRUE == g_isPrintItemHover)
	  printf("Hover: %d,%d\n", nXItem, nYItem);
  if((TRUE == g_bIsPushed) && (CurrentItem.X != nXItem || CurrentItem.Y != nYItem))
  {
    g_bIsPushed = FALSE;
    g_nCurrentFrame = 0;
  }
	CurrentItem.X = nXItem;
	CurrentItem.Y = nYItem;
}

void XN_CALLBACK_TYPE TrackPad_ItemSelect(XnInt32 nXItem, XnInt32 nYItem, XnVDirection eDir, void* cxt)
{
	printf("Select: %d,%d (%s)\n", nXItem, nYItem, XnVDirectionAsString(eDir));
	g_bIsPushed = TRUE;
}

void XN_CALLBACK_TYPE TrackPad_PrimaryCreate(const XnVHandPointContext* cxt, const XnPoint3D& ptFocus, void* UserCxt)
{
  printf("TrackPad input has started!!!, point ID: [%d] ", cxt->nID);
  printf("Starting point position: [%f],[%f],[%f]\n", cxt->ptPosition.X, cxt->ptPosition.Y, cxt->ptPosition.Z);
  g_isInputStarted = TRUE;
}

void XN_CALLBACK_TYPE TrackPad_PrimaryDestroy(XnUInt32 nID, void* UserCxt)
{
  printf("TrackPad input has stopped!!!\n");
  g_isInputStarted = FALSE;
}

const XnFloat GL_WIN_SIZE_X = 720.0;
const XnFloat GL_WIN_SIZE_Y = 480.0;

// More drawing
void DrowTrackPad()
{
	if (!g_bInSession)
		return;

	XnDouble r, g, b;

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
	}


  XnFloat width = 20;
  XnFloat x_step = (XnFloat)GL_WIN_SIZE_X/g_TP_XDim;
  for(XnUInt32 i=0 ; i<=g_TP_XDim ; ++i)
  {
    DrawLine(xnCreatePoint3D(GL_WIN_SIZE_X-(i*x_step), GL_WIN_SIZE_Y, 0.0),
      xnCreatePoint3D(GL_WIN_SIZE_X-(i*x_step), 0.0, 0.0),
      width, r, g, b);
  }

  XnFloat y_step = (XnFloat)GL_WIN_SIZE_Y/g_TP_YDim;
  for(XnUInt32 j=1 ; j<=g_TP_YDim ; ++j)
  {
    DrawLine(xnCreatePoint3D(GL_WIN_SIZE_X, GL_WIN_SIZE_Y-(j*y_step), 0.0),
     xnCreatePoint3D(0.0, GL_WIN_SIZE_Y-(j*y_step), 0.0),
     width, r, g, b); 
  }


  if(TRUE == g_isPrintItemHover)
  {
    XnPoint3D ptTopLeft = xnCreatePoint3D((CurrentItem.X*x_step), /*GL_WIN_SIZE_Y-*/(CurrentItem.Y*y_step), 0.0);
    XnPoint3D ptBottomRight = xnCreatePoint3D(((CurrentItem.X+1)*x_step), /*GL_WIN_SIZE_Y-*/((CurrentItem.Y+1)*y_step), 0.0);

    if(TRUE == g_isInputStarted)
      DrawFrame(ptTopLeft, ptBottomRight, width, 1, 1, 0);
    else
      DrawFrame(ptTopLeft, ptBottomRight, width, 0.2, 0.2, 0);

  }

  width /= 2;
  if(TRUE == g_isPrintValueChange)
  {
    XnFloat TopPointX = (g_fXValue)*GL_WIN_SIZE_X;
    XnFloat TopPointY = (g_fYValue)*GL_WIN_SIZE_Y;
    XnPoint3D ptTopLeft = xnCreatePoint3D(TopPointX, TopPointY, 0.0);
    XnPoint3D ptBottomRight = xnCreatePoint3D(TopPointX+width, TopPointY+width, 0.0);

    if(TRUE == g_isInputStarted)
      DrawFrame(ptTopLeft, ptBottomRight, width, 1, 0, 0);
    else
      DrawFrame(ptTopLeft, ptBottomRight, width, 0.2, 0, 0);

  }

  if(TRUE == g_bIsPushed)
  {
    XnPoint3D ptTopLeft = xnCreatePoint3D((CurrentItem.X*x_step), (CurrentItem.Y*y_step), 0.0);
    XnPoint3D ptBottomRight = xnCreatePoint3D(((CurrentItem.X+1)*x_step), ((CurrentItem.Y+1)*y_step), 0.0);
    DrawFrame(ptTopLeft, ptBottomRight, width, 1, 0.5, 0);

    ++g_nCurrentFrame;
    if(XN_PUSH_DISPLAY_FRAMES <= g_nCurrentFrame)
    {
      g_bIsPushed = FALSE;
      g_nCurrentFrame = 0;
    }
  }
}

#define SAMPLE_XML_FILE "../../Data/Sample-Tracking.xml"

XnBool g_bQuit = false;
void CleanupExit()
{

  if(NULL != g_pTrackPad)
  {
    // Unregister for the Hover event of the TrackPad
    if(NULL != g_nItemHoverHandle)
      g_pTrackPad->UnregisterItemHover(g_nItemHoverHandle);
    // Unregister for the Value Change event of the TrackPad
    if(NULL != g_nValueChangeHandle)
      g_pTrackPad->UnregisterValueChange(g_nValueChangeHandle);

	// Unregister for the Select event of the TrackPad
	if (NULL != g_nItemSelectHandle)
		g_pTrackPad->UnregisterItemSelect(g_nItemSelectHandle);

    // Unregister for Input Stop event of the TrackPad
    if(NULL != g_nPrimaryDestroyHandle)
      g_pTrackPad->UnregisterPrimaryPointDestroy(g_nPrimaryDestroyHandle);
    // Unregister for Input Start event of the TrackPad
    if(NULL != g_nPrimaryCreateHandle)
      g_pTrackPad->UnregisterPrimaryPointCreate(g_nPrimaryCreateHandle);
  }

  if (NULL != g_pSessionManager) 
  {
    if(0 != g_TrackPadHandle)
      g_pSessionManager->RemoveListener(g_TrackPadHandle);
    delete g_pSessionManager;
    g_pSessionManager = NULL;
  }

  if(NULL != g_pTrackPad)
  {
    delete g_pTrackPad;
    g_pTrackPad = NULL;
  }

  g_Context.Shutdown();

  exit (1);
}

void InitiateTrackPad()
{
  if(NULL != g_pTrackPad)
  {
	  g_pTrackPad->SetItemCount(g_TP_XDim, g_TP_YDim);
  }
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

	// Draw the TrackPad
	DrowTrackPad();

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
    break;
  case 'v': //Enable/Disable Value Change presentation on screen
    g_isPrintValueChange = !g_isPrintValueChange;
    break;
  case 'h': //Enable/Disable Item Hover presentation on screen
    g_isPrintItemHover = !g_isPrintItemHover;
    break;
  case 't': //Toggle between Value Change and Item Hover presentation on screen
    if(FALSE == g_isPrintItemHover)
    {
      g_isPrintValueChange = FALSE;
      g_isPrintItemHover = TRUE;
    }
    else
    {
      g_isPrintValueChange = TRUE;
      g_isPrintItemHover = FALSE;
    }
    break;
  case 'r': // Remove a row from the TrackPad
    if(XN_MIN_X_DIM >= g_TP_YDim) //Check if we have reached minimum size
      return;
    --g_TP_YDim;
    InitiateTrackPad();
    break;
  case 'R': // Add a row to the TrackPad
    if(XN_MAX_X_DIM <= g_TP_YDim) //Check if we have reached maximum size
      return;
    ++g_TP_YDim;
    InitiateTrackPad();
    break;
  case 'c': // Remove a column from the TrackPad
    if(XN_MIN_Y_DIM >= g_TP_XDim) //Check if we have reached minimum size
      return;
    --g_TP_XDim;
    InitiateTrackPad();
    break;
  case 'C': // Add a column to the TrackPad
    if(XN_MAX_Y_DIM <= g_TP_XDim) //Check if we have reached maximum size
      return;
    ++g_TP_XDim;
    InitiateTrackPad();
    break;
	}
}


void glInit (int * pargc, char ** argv)
{
	glutInit(pargc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("PrimeSense NITE TrackPad Sample");
	//glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
}
#endif


void XN_CALLBACK_TYPE SessionStart(const XnPoint3D& ptFocus, void* UserCxt)
{
	g_bInSession = true;
}
void XN_CALLBACK_TYPE SessionEnd(void* UserCxt)
{
 	g_bInSession = false;
}

void AllocateAllGlobals()
{
  // Configure
  XnStatus rc = g_Context.InitFromXmlFile(SAMPLE_XML_FILE);
  if (rc != XN_STATUS_OK)
  {
	  printf("Error initializing: %s\n", xnGetStatusString(rc));
	  CleanupExit();
  }

  if(NULL == g_pSessionManager)
  {
	  g_pSessionManager = new XnVSessionManager();
  }
  if(NULL == g_pSessionManager)
  {
    printf("Couldn't create PointTracker!! (out of memory)\n");
    CleanupExit();
  }

  if(NULL == g_pTrackPad)
  {
	  g_pTrackPad = new XnVSelectableSlider2D(g_TP_XDim, g_TP_YDim);
  }
  if(NULL == g_pTrackPad)
  {
    printf("Couldn't create TrackPad!! (out of memory)\n");
    CleanupExit();
  }

  g_Context.StartGeneratingAll();
}

int main(int argc, char ** argv)
{
  AllocateAllGlobals();

	// Initialize the point tracker
	XnStatus rc = g_pSessionManager->Initialize(&g_Context, "Wave", "RaiseHand");
	if (rc != XN_STATUS_OK)
	{
		printf("Couldn't initialize the Session Manager: %s\n", xnGetStatusString(rc));
		CleanupExit();
	}
  g_pSessionManager->RegisterSession(NULL, &SessionStart, &SessionEnd);

	// Add TrackPad to the point tracker
	g_TrackPadHandle = g_pSessionManager->AddListener(g_pTrackPad);

	// Register for the Hover event of the TrackPad
	g_nItemHoverHandle = g_pTrackPad->RegisterItemHover(NULL, &TrackPad_ItemHover);
	// Register for the Value Change event of the TrackPad
	g_nValueChangeHandle = g_pTrackPad->RegisterValueChange(NULL, &TrackPad_ValueChange);
	// Register for the Select event of the TrackPad
	g_nItemSelectHandle = g_pTrackPad->RegisterItemSelect(NULL, &TrackPad_ItemSelect);

  // Register for Input Start event of the TrackPad
  g_nPrimaryCreateHandle = g_pTrackPad->RegisterPrimaryPointCreate(NULL, &TrackPad_PrimaryCreate);
  // Register for Input Stop event of the TrackPad
  g_nPrimaryDestroyHandle = g_pTrackPad->RegisterPrimaryPointDestroy(NULL, &TrackPad_PrimaryDestroy);

  // Start catching signals for quit indications
  CatchSignals(&g_bQuit);



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
