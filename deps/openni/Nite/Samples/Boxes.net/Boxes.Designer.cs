namespace Boxes.net
{
    partial class Boxes
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.Box1 = new System.Windows.Forms.Panel();
            this.Box2 = new System.Windows.Forms.Panel();
            this.Box3 = new System.Windows.Forms.Panel();
            this.SliderArea = new System.Windows.Forms.Panel();
            this.Scroller = new System.Windows.Forms.Panel();
            this.SliderArea.SuspendLayout();
            this.SuspendLayout();
            // 
            // Box1
            // 
            this.Box1.BackColor = System.Drawing.Color.White;
            this.Box1.Location = new System.Drawing.Point(20, 30);
            this.Box1.Name = "Box1";
            this.Box1.Size = new System.Drawing.Size(200, 180);
            this.Box1.TabIndex = 0;
            // 
            // Box2
            // 
            this.Box2.BackColor = System.Drawing.Color.White;
            this.Box2.Location = new System.Drawing.Point(250, 30);
            this.Box2.Name = "Box2";
            this.Box2.Size = new System.Drawing.Size(200, 180);
            this.Box2.TabIndex = 1;
            // 
            // Box3
            // 
            this.Box3.BackColor = System.Drawing.Color.White;
            this.Box3.Location = new System.Drawing.Point(480, 30);
            this.Box3.Name = "Box3";
            this.Box3.Size = new System.Drawing.Size(200, 180);
            this.Box3.TabIndex = 1;
            // 
            // SliderArea
            // 
            this.SliderArea.BackColor = System.Drawing.Color.White;
            this.SliderArea.Controls.Add(this.Scroller);
            this.SliderArea.Location = new System.Drawing.Point(20, 280);
            this.SliderArea.Name = "SliderArea";
            this.SliderArea.Size = new System.Drawing.Size(660, 60);
            this.SliderArea.TabIndex = 2;
            // 
            // Scroller
            // 
            this.Scroller.Location = new System.Drawing.Point(0, 0);
            this.Scroller.Name = "Scroller";
            this.Scroller.Size = new System.Drawing.Size(10, 60);
            this.Scroller.TabIndex = 3;
            // 
            // Boxes
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(712, 373);
            this.Controls.Add(this.SliderArea);
            this.Controls.Add(this.Box3);
            this.Controls.Add(this.Box2);
            this.Controls.Add(this.Box1);
            this.Name = "Boxes";
            this.Text = "Boxes";
            this.Load += new System.EventHandler(this.Boxes_Load);
            this.SliderArea.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Panel Box1;
        private System.Windows.Forms.Panel Box2;
        private System.Windows.Forms.Panel SliderArea;
        private System.Windows.Forms.Panel Box3;
        private System.Windows.Forms.Panel Scroller;
    }
}

