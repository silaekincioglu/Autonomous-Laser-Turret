using System;
using System.Drawing;
using System.IO.Ports;
using System.Windows.Forms;
using Emgu.CV;
using Emgu.CV.Structure;

namespace AutonomousLaserTurret
{
    /// <summary>
    /// Autonomous Vision-Based Targeting & 2-DOF Pan-Tilt Laser Turret Controller.
    /// Combines real-time image processing, centroid extraction, analytical 
    /// inverse kinematics, and UART hardware communication.
    /// </summary>
    public class TurretController
    {
        // Serial Communication Protocol: [Th1 (Pan), Th2 (Tilt), LaserState]
        private SerialPort _serialPort;
        public byte[] SerialBuffer = new byte[3];

        // Mechanical Turret Link Constants (cm)
        private const double L1 = 3.0; // Base to tilt axis offset
        private const double D1 = 4.0; // Height offset
        private const double DistanceScale = 100.0 / 365.0; // Pixel-to-cm conversion scale

        // Kinematic Position and Angle Parameters
        public double Xcm { get; private set; }
        public double Ycm { get; private set; }
        public double Theta1 { get; private set; } // Pan Servo Angle (0 - 180 deg)
        public double Theta2 { get; private set; } // Tilt Servo Angle (0 - 180 deg)

        public TurretController(string portName = "COM5", int baudRate = 9600)
        {
            _serialPort = new SerialPort(portName, baudRate);
            try
            {
                if (!_serialPort.IsOpen)
                {
                    _serialPort.Open();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Serial Port Initialization Error: {ex.Message}");
            }
        }

        /// <summary>
        /// Pipeline: Color space conversion, inverse binary thresholding, and centroid detection.
        /// </summary>
        public void ProcessFrame(Image<Bgr, byte> currentFrame, int thresholdValue)
        {
            if (currentFrame == null) return;

            // 1. Grayscale Conversion
            Image<Gray, byte> grayFrame = currentFrame.Convert<Gray, byte>();

            // 2. Inverse Binary Thresholding for dark target isolation
            Image<Gray, byte> binaryMask = grayFrame.ThresholdBinaryInv(new Gray(thresholdValue), new Gray(255));

            // 3. Compute Spatial Moments for target centroid
            MCvMoments moments = binaryMask.GetMoments(true);
            if (moments.m00 != 0)
            {
                int centroidX = (int)Math.Round(moments.m10 / moments.m00);
                int centroidY = (int)Math.Round(moments.m01 / moments.m00);

                // Highlight detected centroid
                binaryMask.Draw(new CircleF(new PointF(centroidX, centroidY), 5), new Gray(128), 2);
            }

            // 4. Calculate Center of Gravity and Metric Offset
            Point cogMetric = CalculateCOG(binaryMask);
            Xcm = cogMetric.X;
            Ycm = cogMetric.Y;
        }

        /// <summary>
        /// Custom 3x3 Neighborhood Erosion filter to suppress background noise.
        /// </summary>
        public Image<Gray, byte> ApplyCustomCorrosion(Image<Gray, byte> sourceImage, int iterations)
        {
            Image<Gray, byte> processedImage = sourceImage.Copy();

            for (int r = 0; r < iterations; r++)
            {
                sourceImage.CopyTo(processedImage);
                for (int i = 1; i < sourceImage.Width - 1; i++)
                {
                    for (int j = 1; j < sourceImage.Height - 1; j++)
                    {
                        if (sourceImage[j, i].Intensity == 0)
                        {
                            int count = 0;
                            for (int x = i - 1; x <= i + 1 && count == 0; x++)
                            {
                                for (int y = j - 1; y <= j + 1 && count == 0; y++)
                                {
                                    if (sourceImage[y, x].Intensity == 255) count++;
                                }
                                processedImage.Data[j, i, 0] = 0;
                            }
                        }
                    }
                }
                processedImage.CopyTo(sourceImage);
            }
            return processedImage;
        }

        /// <summary>
        /// Translates pixel space intensity distribution into real-world coordinate offsets (cm).
        /// </summary>
        private Point CalculateCOG(Image<Gray, byte> binaryMask)
        {
            double sumX = 0.0;
            double sumY = 0.0;
            double totalIntensity = 0.0;

            for (int y = binaryMask.Height / 5; y < (binaryMask.Height * 4) / 5; y++)
            {
                for (int x = binaryMask.Width / 5; x < (binaryMask.Width * 4) / 5; x++)
                {
                    double intensity = binaryMask[y, x].Intensity;
                    if (intensity > 0)
                    {
                        sumX += x * intensity;
                        sumY += y * intensity;
                        totalIntensity += intensity;
                    }
                }
            }

            if (totalIntensity == 0) return new Point(0, 0);

            int pixelCogX = (int)(sumX / totalIntensity);
            int pixelCogY = (int)(sumY / totalIntensity);

            // Coordinate shifting relative to camera optical center
            double relX = (binaryMask.Width / 2.0) - pixelCogX;
            double relY = pixelCogY - (binaryMask.Height / 2.0);

            // Metrical mapping with systemic calibration offsets (+8cm, +2cm)
            double xMetric = -(relX * DistanceScale) + 8.0;
            double yMetric = -(relY * DistanceScale) + 2.0;

            return new Point((int)xMetric, (int)yMetric);
        }

        /// <summary>
        /// Solves 2-DOF analytical inverse kinematics equations and transmits command packet via UART.
        /// </summary>
        public void ExecuteKinematicTargeting(double targetX, double targetY)
        {
            // Workspace target vectors
            double Px = 163.4;
            double Py = targetX + 15.0;
            double Pz = targetY + 21.0;

            // 1. Pan Axis Angle (Theta 1)
            double radTh1 = Math.Atan2(Py, Px);

            // Planar projections
            double PxPrime = Px - (L1 * Math.Cos(radTh1));
            double PzPrime = Pz + D1;

            // 2. Tilt Axis Angle (Theta 2)
            double radTh2 = Math.PI - Math.Atan2(PxPrime, PzPrime);

            // Convert to degrees and map hardware bounds
            double degTh1 = radTh1 * (180.0 / Math.PI);
            double degTh2 = radTh2 * (180.0 / Math.PI);

            // Clamp servo motion to physical safety domain [0, 180]
            Theta1 = Math.Max(0, Math.Min(180, degTh1));
            Theta2 = Math.Max(0, Math.Min(180, degTh2));

            // Construct and dispatch 3-byte payload over UART
            SerialBuffer[0] = (byte)Theta1; // Pan Servo
            SerialBuffer[1] = (byte)Theta2; // Tilt Servo
            SerialBuffer[2] = 1;            // Fire Laser Pulse

            if (_serialPort != null && _serialPort.IsOpen)
            {
                _serialPort.Write(SerialBuffer, 0, 3);
            }
        }

        /// <summary>
        /// Safe shutdown signal for laser diode preservation.
        /// </summary>
        public void DisableLaserEmitter()
        {
            if (_serialPort != null && _serialPort.IsOpen)
            {
                SerialBuffer[2] = 0; // Laser Diode OFF
                _serialPort.Write(SerialBuffer, 0, 3);
            }
        }
    }
}