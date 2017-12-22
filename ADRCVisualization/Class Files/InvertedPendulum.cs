using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RJCP.IO.Ports;

namespace ADRCVisualization.Class_Files
{
    using System.Configuration;
    using System;
    using System.Windows.Forms;
    using System.Threading;

    class InvertedPendulum
    {
        private SerialPortStream serialPort;
        private double theta;
        private double modifier = 1;

        /// <summary>
        /// Initializes the COM port
        /// </summary>
        public InvertedPendulum()
        {
            string sourcePort = "";

            foreach (string c in SerialPortStream.GetPortNames())
            {
                Console.WriteLine("GetPortNames: " + c);
                sourcePort = c;
            }

            serialPort = new SerialPortStream(sourcePort, 115200);

            try
            {
                serialPort.Open();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        /// <summary>
        /// Reads and writes data to an arduino controller through a serialport to 
        /// </summary>
        /// <param name="force">Force applied to point of rotation</param>
        /// <returns>Returns current angle of pendulum</returns>
        public double Calculate(double force)
        {
            // Read data from MPU6050
            // Write data to DC Motor

            // DC motor must not be based on velocity, the output should be based on torque change
            // IE higher motor output doesn't just increase speed, it also increases torque

            if (serialPort.IsOpen && serialPort.CanRead && serialPort.CanWrite)
            {
                //Write motor force to arduino
                serialPort.WriteLine(force.ToString());

                //Read current angle from arduino
                Double.TryParse(serialPort.ReadLine(), out double arduinoMPUData);

                theta = arduinoMPUData;
            }
            else
            {
                try
                {
                    serialPort.Open();
                }
                catch (Exception ex){ }

                //Use to test functionality of this application
                //modifier -= force * 0.001;
                //theta += modifier;
            }

            return theta;
        }
    }
}
