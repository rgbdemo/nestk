using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using xn;
using xnv;

namespace Boxes.net
{
    class MyBox : PointControl
    {
        public MyBox(System.Windows.Forms.Panel box, string name) :
            base()
        {
            this.name = name;
            this.box = box;

            pushDetector = new PushDetector();
            swipeDetector = new SwipeDetector();
            steadyDetector = new SteadyDetector();
            flowRouter = new FlowRouter();
            broadcaster = new Broadcaster();

            broadcaster.AddListener(pushDetector);
            broadcaster.AddListener(flowRouter);

            pushDetector.Push += new PushDetector.PushHandler(pushDetector_Push);
            steadyDetector.Steady += new SteadyDetector.SteadyHandler(steadyDetector_Steady);
            swipeDetector.GeneralSwipe += new SwipeDetector.GeneralSwipeHandler(swipeDetector_GeneralSwipe);

            PrimaryPointCreate += new PrimaryPointCreateHandler(MyBox_PrimaryPointCreate);
            PrimaryPointDestroy += new PrimaryPointDestroyHandler(MyBox_PrimaryPointDestroy);
            OnUpdate += new UpdateHandler(MyBox_OnUpdate);
        }

        void MyBox_PrimaryPointDestroy(uint id)
        {
            this.box.BackColor = Color.LightBlue;
        }

        void MyBox_OnUpdate(IntPtr message)
        {
            broadcaster.UpdateMessage(message);
        }

        void MyBox_PrimaryPointCreate(ref HandPointContext context, ref Point3D ptFocus)
        {
            flowRouter.SetActive(steadyDetector);
        }

        void swipeDetector_GeneralSwipe(Direction dir, float velocity, float angle)
        {
            Console.WriteLine("{0}: Swipe {1}", this.name, dir);
            flowRouter.SetActive(steadyDetector);
            this.box.BackColor = Color.Red;
        }

        void steadyDetector_Steady(float velocity)
        {
            Console.WriteLine("{0}: Steady", this.name);
            flowRouter.SetActive(swipeDetector);
            this.box.BackColor = Color.White;
        }

        void pushDetector_Push(float velocity, float angle)
        {
            Leave();
        }

        #region Leave Event
        public delegate void LeaveHandler();
        public event LeaveHandler Leave;
        #endregion

        System.Windows.Forms.Panel box;

        private PushDetector pushDetector;
        private SwipeDetector swipeDetector;
        private SteadyDetector steadyDetector;
        private FlowRouter flowRouter;
        private Broadcaster broadcaster;
        private string name;
    }
/*
    class Program
    {

        static FlowRouter mainRouter;
        static SelectableSlider1D ss;
        static MyBox[] boxes = new MyBox[3];
        static void Main(string[] args)
        {
            string SAMPLE_XML_FILE = @"c:/proj/SDK/Modules/NITE_OpenNI/Data/Sample-Tracking.xml";

            Context context = new Context(SAMPLE_XML_FILE);

            SessionManager sm = new SessionManager(context, "Wave", "RaiseHand");

            boxes[0] = new MyBox("Box1");
            boxes[1] = new MyBox("Box2");
            boxes[2] = new MyBox("Box3");
            ss = new SelectableSlider1D(3, Axis.X);
            mainRouter = new FlowRouter();

            sm.AddListener(mainRouter);

            sm.SessionStart += new SessionManager.SessionStartHandler(sm_SessionStart);
            sm.SessionFocusProgress += new SessionManager.SessionFocusProgressHandler(sm_SessionFocusProgress);
            sm.SessionEnd += new SessionManager.SessionEndHandler(sm_SessionEnd);
            sm.SetQuickRefocusTimeout(0);

            ss.ItemSelect += new SelectableSlider1D.ItemSelectHandler(ss_ItemSelect);
            ss.ItemHover += new SelectableSlider1D.ItemHoverHandler(ss_ItemHover);
            boxes[0].Leave += new MyBox.LeaveHandler(box_Leave);
            boxes[1].Leave += new MyBox.LeaveHandler(box_Leave);
            boxes[2].Leave += new MyBox.LeaveHandler(box_Leave);

            while (!shouldStop)
            {
                context.WaitAndUpdateAll();
                sm.Update(context);
            }
        }

        static void ss_ItemHover(int index)
        {
            Console.WriteLine("Slider: hover {0}", index);
        }

        static void box_Leave()
        {
            mainRouter.SetActive(ss);
        }

        static void ss_ItemSelect(int index, Direction dir)
        {
            if (dir == Direction.Up)
            {
                Console.WriteLine("Selected box {0}", index);
                mainRouter.SetActive(boxes[index]);
            }
            else if (dir == Direction.Down)
            {
                shouldStop = true;
            }
        }

        static private bool shouldStop = false;

        static void sm_SessionEnd()
        {
            Console.WriteLine("Session end");
        }

        static void sm_SessionFocusProgress(string strFocus, ref Point3D ptPosition, float fProgress)
        {
            Console.WriteLine("focus gesture \"{0}\" is in progress at ({1},{2},{3}) - {4}",
                strFocus, ptPosition.X, ptPosition.Y, ptPosition.Z, fProgress);
        }

        static void sm_SessionStart(ref Point3D position)
        {
            Console.WriteLine("Session start at ({0},{1},{2})", position.X, position.Y, position.Z);
            mainRouter.SetActive(ss);
        }
    }
 */
}
