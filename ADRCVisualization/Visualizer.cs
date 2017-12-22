using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;
using System.Windows.Forms;
using ADRCVisualization.Class_Files;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.IO;

namespace ADRCVisualization
{
    public partial class Visualizer : Form
    {
        private BackgroundWorker backgroundWorker;
        private DateTime dateTime;
        private Bitmap feedbackBitmap;

        //Inverted Pendulum
        private InvertedPendulum invertedPendulum;
        private double SetPoint = 0;
        private double NoiseFactor = 0;

        private bool useADRC = true;

        //FeedbackControllers
        private double maxOutput = 1000;
        private bool initializeFeedbackControllers = false;
        private double WaitTimeForCalculation = 5;
        private double RunTime = 20;
        
        //Proportional Integral Derivative Parameters
        private PID pid;
        private double kp = 0.36;
        private double ki = 0;
        private double kd = 0.2485;//0.18

        //Active Disturbance Rejection Control Parameters
        private ADRC_PD adrc;
        private double r = 2000;//80   acceleration
        private double c = 500;//500 overshoot velocityf
        private double b = 2.875;//0.5   smoothing and acceleration
        private double hModifier = 0.00085;//0.005   overshoot


        private List<float> PreviousOutputs = new List<float>();
        private List<float> PreviousAngles = new List<float>();
        private StreamWriter fileWriter;
        private KalmanFilter feedbackMaxKalman;
        private double currentAngle;
        private double currentOutput;
        private int counter = 0;


        //Timers for alternate threads and asynchronous calculations
        private System.Timers.Timer t1;
        private System.Timers.Timer t2;
        private System.Timers.Timer t3;

        //Fourier Transforms
        private FourierBitmap FourierBitmap;
        private float FourierTolerance = 1f;
        
        public Visualizer()
        {
            InitializeComponent();
            
            dateTime = DateTime.Now;

            InitializeFileWriters();

            InitializeCharts();

            feedbackBitmap = new Bitmap(Path.GetFullPath(@"..\..\PID.png"));

            FourierBitmap = new FourierBitmap(540, 350, (float)maxOutput);

            feedbackMaxKalman = new KalmanFilter(0.001, 20);

            backgroundWorker = new BackgroundWorker();
            backgroundWorker.DoWork += new DoWorkEventHandler(CalculateFourierTransforms);
            backgroundWorker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(ChangeFourierTransforms);

            StartTimers();
            StopTimers();
        }

        /// <summary>
        /// Creates the file writers and adds headers to the files
        /// </summary>
        private void InitializeFileWriters()
        {
            fileWriter = new StreamWriter(Environment.GetFolderPath(Environment.SpecialFolder.Desktop) + @"\Data.csv");
        }

        private void InitializeCharts()
        {
            chart1.ChartAreas[0].AxisY.Maximum = 360;
            chart1.ChartAreas[0].AxisY.Minimum = -60;

            chart3.Series[0].Points.Add(0);

            chart3.ChartAreas[0].AxisY.Maximum = maxOutput;
            chart3.ChartAreas[0].AxisY.Minimum = -maxOutput;
        }

        /// <summary>
        /// Starts alternate threads for calculation of the inverted pendulum and updating the display of the user interface for the FFTWs, pendulum, and graphs.
        /// </summary>
        private async void StartTimers()
        {
            await Task.Delay(75);

            this.BeginInvoke((Action)(() =>
            {
                Thread t = new Thread(new ThreadStart(ShowWaitingForCOM));
                t.Start();

                invertedPendulum = new InvertedPendulum();//Initiates connection

                SendKeys.Send("{ESC}");//Cancels Messagebox after connection

                t.Abort();
                
                t1 = new System.Timers.Timer
                {
                    Interval = 50, //In milliseconds here
                    AutoReset = true //Stops it from repeating
                };
                t1.Elapsed += new ElapsedEventHandler(SetInvertedPendulumAngle);
                t1.Start();

                t2 = new System.Timers.Timer
                {
                    Interval = 5, //In milliseconds here
                    AutoReset = true //Stops it from repeating
                };
                t2.Elapsed += new ElapsedEventHandler(ChangeAngle);
                t2.Start();
                
                t3 = new System.Timers.Timer
                {
                    Interval = 50, //In milliseconds here
                    AutoReset = true //Stops it from repeating
                };
                t3.Elapsed += new ElapsedEventHandler(UpdateFourierTransforms);
                t3.Start();
            }));
        }

        /// <summary>
        /// Used in secondary thread only
        /// </summary>
        private void ShowWaitingForCOM()
        {
            MessageBox.Show("Awaiting connection to COM Port.");
        }

        /// <summary>
        /// Stops the secondary threads to end the calculation.
        /// </summary>
        private async void StopTimers()
        {
            await Task.Delay((int)RunTime * 1000);

            this.BeginInvoke((Action)(() =>
            {
                t1.Stop();
                t2.Stop();
                t3.Stop();

                label1.Text = "Calculation Stopped";
            }));
        }
        
        /// <summary>
        /// Why FFT?
        /// -Displays change of frequency of pendulum, slowing/speeding up
        /// -Displays noise effectively from output
        /// -Displays switching frequency of feedback controller
        /// 
        /// Updates the Fourier Transform charts and 2d memory displays.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void UpdateFourierTransforms(object sender, ElapsedEventArgs e)
        {
            if (!(DateTime.Now.Subtract(dateTime).TotalSeconds > RunTime))
            {
                this.BeginInvoke((Action)(() =>
                {
                    if (PreviousOutputs.ToArray().Length > 1)//FourierTransform.FourierMemory / 1.25)
                    {
                        chart3.Series[0].Points.Clear();

                        chart3.Series[1].Points.Clear();

                        while (!backgroundWorker.IsBusy)
                        {
                            try
                            {
                                backgroundWorker.RunWorkerAsync();
                                break;
                            }
                            catch(Exception ex)
                            {
                                Console.WriteLine(ex.ToString());
                            }
                        }
                    }
                }));
            }
        }
        
        /// <summary>
        /// Changes the angle of the pendulums and calculates the corrections for the feedback controllers.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void ChangeAngle(object sender, ElapsedEventArgs e)
        {
            if (DateTime.Now.Subtract(dateTime).TotalSeconds > WaitTimeForCalculation)
            {
                if (!initializeFeedbackControllers)
                {
                    pid = new PID(kp, ki, kd, maxOutput);
                    adrc = new ADRC_PD(r, c, b, hModifier, kp, kd, maxOutput);
                    
                    this.BeginInvoke((Action)(() =>
                    {
                        label1.Text = "Correction State: True";
                    }));
                    
                    initializeFeedbackControllers = true;
                }

                if (useADRC)
                {
                    currentOutput = adrc.Calculate(SetPoint, currentAngle);
                }
                else
                {
                    currentOutput = pid.Calculate(SetPoint, currentAngle);
                }


                fileWriter.WriteLine(counter + "," + currentOutput + "," + currentAngle);

                counter++;

                PreviousAngles.Add((float)currentAngle);
                PreviousOutputs.Add((float)currentOutput);

                if (counter > FourierTransform.FourierMemory)
                {
                    PreviousAngles.RemoveAt(0);
                    PreviousOutputs.RemoveAt(0);
                }
            }

            Random rand = new Random();

            double noise = rand.NextDouble() * NoiseFactor * (rand.Next(0, 1) * 2 - 1);
            double invertedPendulumAngle = 0;

            invertedPendulumAngle = invertedPendulum.Calculate(-currentOutput);

            currentAngle = (invertedPendulumAngle + noise);
        }
        
        /// <summary>
        /// Updates the diplay of the inverted pendulums and the charts.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void SetInvertedPendulumAngle(object sender, ElapsedEventArgs e)
        {
            if (!(DateTime.Now.Subtract(dateTime).TotalSeconds > RunTime))
            {
                Bitmap rotatedFeedbackBMP = BitmapModifier.RotateImage(feedbackBitmap, (float)currentAngle + 180f);

                this.BeginInvoke((Action)(() =>
                {
                    chart1.Series[0].Points.Add(currentAngle);
                    chart1.Series[1].Points.Add(SetPoint);

                    chart2.Series[0].Points.Add(currentOutput);

                    chart1.Series[0].Color = Color.DarkGreen;
                    chart1.Series[1].Color = Color.Red;

                    chart2.Series[0].Color = Color.Red;
                    
                    pictureBox1.Image = rotatedFeedbackBMP;
                }));
            }
        }

        /// <summary>
        /// Calculates the fourier transforms and updates the axis scales.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CalculateFourierTransforms(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker worker = (BackgroundWorker)sender;
            
            float[] FFTW = FourierTransform.CalculateFFTW(PreviousOutputs.ToArray());

            float[] AngleFFTW = FourierTransform.CalculateFFTW(PreviousAngles.ToArray());

            this.BeginInvoke((Action)(() =>
            {
                double fourierMaxOutput = feedbackMaxKalman.Filter((FFTW.Max() + Math.Abs(FFTW.Min())) / 2);

                pidScale.Text = "Scale: " + fourierMaxOutput;

                chart3.ChartAreas[0].AxisY.Maximum = fourierMaxOutput;
                chart3.ChartAreas[0].AxisY.Minimum = -fourierMaxOutput;
            }));

            e.Result = new float[2][] { FFTW, AngleFFTW};
        }

        /// <summary>
        /// Updates the fourier transform charts and the 2d memory displays.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ChangeFourierTransforms(object sender, RunWorkerCompletedEventArgs e)
        {
            foreach (float freq in ((float[][])(e.Result))[0])
            {
                chart3.Series[0].Points.Add(freq);
            }

            foreach (float freq in ((float[][])(e.Result))[1])
            {
                chart3.Series[1].Points.Add(freq);
            }

            double FFTWStdDev = MathFunctions.CalculateStdDev(Array.ConvertAll(((float[][])(e.Result))[0], x => (double)x).AsEnumerable());
            
            if (FFTWStdDev > FourierTolerance)
            {
                FourierBitmap.SetMaxOutput((float)feedbackMaxKalman.GetFilteredValue());
                pidPictureBox.Image = FourierBitmap.Calculate2DFourierTransform(((float[][])(e.Result))[0]);
            }
        }

        private void Visualizer_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (t1 != null)
            {
                t1.Stop();
                t2.Stop();
                t3.Stop();
            }
        }
    }
}
