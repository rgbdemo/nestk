using System;
using System.Collections.Generic;
using System.ComponentModel;
//using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Windows.Forms;

namespace Boxes.net
{
    public partial class Boxes : Form
    {
        public Boxes()
        {
            InitializeComponent();

            this.context = new xn.Context(SAMPLE_XML_FILE);
            this.sessionManager = new xnv.SessionManager(this.context, "Wave", "RaiseHand");

            this.flowRouter = new xnv.FlowRouter();
            this.selectableSlider = new xnv.SelectableSlider1D(3, xnv.Axis.X);
            this.boxes = new MyBox[3];
            this.boxes[0] = new MyBox(this.Box1, "Box1");
            this.boxes[1] = new MyBox(this.Box2, "Box2");
            this.boxes[2] = new MyBox(this.Box3, "Box3");

            this.boxes[0].Leave += new MyBox.LeaveHandler(Boxes_Leave);
            this.boxes[1].Leave += new MyBox.LeaveHandler(Boxes_Leave);
            this.boxes[2].Leave += new MyBox.LeaveHandler(Boxes_Leave);

            this.selectableSlider.ItemHover += new xnv.SelectableSlider1D.ItemHoverHandler(selectableSlider_ItemHover);
            this.selectableSlider.ItemSelect += new xnv.SelectableSlider1D.ItemSelectHandler(selectableSlider_ItemSelect);
            this.selectableSlider.PrimaryPointCreate += new xnv.PointControl.PrimaryPointCreateHandler(selectableSlider_PrimaryPointCreate);
            this.selectableSlider.PrimaryPointDestroy += new xnv.PointControl.PrimaryPointDestroyHandler(selectableSlider_PrimaryPointDestroy);

            this.sessionManager.SessionStart += new xnv.SessionManager.SessionStartHandler(sessionManager_SessionStart);

            this.sessionManager.AddListener(this.flowRouter);

            this.shouldRun = true;
            this.readerThread = new Thread(ReaderThread);
            this.readerThread.Start();
        }


        void selectableSlider_ItemSelect(int index, xnv.Direction dir)
        {
            if (dir == xnv.Direction.Up)
            {
                this.flowRouter.SetActive(this.boxes[index]);
            }
        }

        void sessionManager_SessionStart(ref xn.Point3D position)
        {
            this.flowRouter.SetActive(this.selectableSlider);
        }

        void selectableSlider_PrimaryPointDestroy(uint id)
        {
            this.SliderArea.BackColor = Color.White;
        }

        void selectableSlider_PrimaryPointCreate(ref xnv.HandPointContext context, ref xn.Point3D ptFocus)
        {
            this.SliderArea.BackColor = Color.Red;
        }

        void selectableSlider_ItemHover(int index)
        {
            switch (index)
            {
                case 0:
                    this.Box1.BackColor = Color.Yellow;
                    this.Box2.BackColor = Color.LightBlue;
                    this.Box3.BackColor = Color.LightBlue;
                    break;
                case 1:
                    this.Box1.BackColor = Color.LightBlue;
                    this.Box2.BackColor = Color.Yellow;
                    this.Box3.BackColor = Color.LightBlue;
                    break;
                case 2:
                    this.Box1.BackColor = Color.LightBlue;
                    this.Box2.BackColor = Color.LightBlue;
                    this.Box3.BackColor = Color.Yellow;
                    break;
            }
        }

        void Boxes_Leave()
        {
            this.flowRouter.SetActive(this.selectableSlider);
            throw new NotImplementedException();
        }

        private void Boxes_Load(object sender, EventArgs e)
        {
            // First time
            this.Box1.BackColor = Color.LightBlue;
            this.Box2.BackColor = Color.LightBlue;
            this.Box3.BackColor = Color.LightBlue;
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            this.shouldRun = false;
            this.readerThread.Join();
            base.OnClosing(e);
        }
        protected override void OnKeyPress(KeyPressEventArgs e)
        {
            switch(e.KeyChar)
            {
                case (char)27:
                    Close();
                    break;
            }
            base.OnKeyPress(e);
        }

        private void ReaderThread()
        {
            while (this.shouldRun)
            {
                try
                {
                    this.context.WaitAndUpdateAll();
                    this.sessionManager.Update(this.context);
                }
                catch (System.Exception)
                {
                	
                }
            }
        }

        private readonly string SAMPLE_XML_FILE = @"../../Data//Sample-Tracking.xml";

        private xn.Context context;
        private Thread readerThread;
        private bool shouldRun;
        private xnv.SessionManager sessionManager;
        private xnv.FlowRouter flowRouter;
        private xnv.SelectableSlider1D selectableSlider;
        private MyBox[] boxes;

    }
}
