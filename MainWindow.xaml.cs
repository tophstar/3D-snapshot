
namespace DataCaptureMultipleKinect
{
    using Microsoft.Kinect;
    using System;
    using System.ComponentModel;
    using System.Globalization;
    using System.IO;
    using System.Threading.Tasks;
    using System.Windows;
    using System.Windows.Data;
    using System.Windows.Controls;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Threading;
    using Microsoft.Kinect.Toolkit;
    using Microsoft.Kinect.Toolkit.Fusion;

    /// <summary>
    /// A struct containing depth image pixels and frame timestamp
    /// </summary>
    internal struct DepthData
    {
        public DepthImagePixel[] DepthImagePixels;
        public long FrameTimestamp;
    }

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged, IDisposable
    {

        /// <summary>
        /// The Kinect Fusion volume
        /// </summary>
        private Reconstruction volume;
        /// <summary>
        /// Timer to count FPS
        /// </summary>
        private DispatcherTimer fpsTimer;
        /// <summary>
        /// The counter for frames that have been processed
        /// </summary>
        private int processedFrameCount = 0;
        /// <summary>
        /// Timer stamp of last computation of FPS
        /// </summary>
        private DateTime lastFPSTimestamp;
        /// <summary>
        /// Saving mesh flag
        /// </summary>
        private bool savingMesh;
        /// <summary>
        /// Timestamp of last depth frame in milliseconds
        /// </summary>
        private long lastFrameTimestamp = 0;
        /// <summary>
        /// The transformation between the world and camera view coordinate system
        /// </summary>
        private Matrix4 worldToCameraTransform;
        /// <summary>
        /// The reconstruction volume voxel density in voxels per meter (vpm)
        /// 1000mm / 256vpm = ~3.9mm/voxel
        /// </summary>
        private float voxelsPerMeter = 256.0f;
        /// <summary>
        /// The reconstruction volume processor type. This parameter sets whether AMP or CPU processing
        /// is used. Note that CPU processing will likely be too slow for real-time processing.
        /// </summary>
        private const ReconstructionProcessor ProcessorType = ReconstructionProcessor.Amp;
        /// <summary>
        /// The default transformation between the world and volume coordinate system
        /// </summary>
        private Matrix4 defaultWorldToVolumeTransform;
        /// <summary>
        /// Parameter to translate the reconstruction based on the minimum depth setting. When set to
        /// false, the reconstruction volume +Z axis starts at the camera lens and extends into the scene.
        /// Setting this true in the constructor will move the volume forward along +Z away from the
        /// camera by the minimum depth threshold to enable capture of very small reconstruction volumes
        /// by setting a non-identity world-volume transformation in the ResetReconstruction call.
        /// Small volumes should be shifted, as the Kinect hardware has a minimum sensing limit of ~0.35m,
        /// inside which no valid depth is returned, hence it is difficult to initialize and track robustly  
        /// when the majority of a small volume is inside this distance.
        /// </summary>
        private bool translateResetPoseByMinDepthThreshold = true;
        /// <summary>
        /// The counter for image process failures
        /// </summary>
        private int trackingErrorCount = 0;
        /// <summary>
        /// Minimum depth distance threshold in meters. Depth pixels below this value will be
        /// returned as invalid (0). Min depth must be positive or 0.
        /// </summary>
        private float minDepthClip = FusionDepthProcessor.DefaultMinimumDepth;
        /// <summary>
        /// Maximum depth distance threshold in meters. Depth pixels above this value will be
        /// returned as invalid (0). Max depth must be greater than 0.
        /// </summary>
        private float maxDepthClip = FusionDepthProcessor.DefaultMaximumDepth;
        /// <summary>
        /// Pause or resume image integration
        /// </summary>
        private bool pauseIntegration;
        /// <summary>
        /// Binding property to check box "Pause Integration"
        /// </summary>
        /// <summary>
        /// Property change event
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;
        public bool PauseIntegration
        {
            get
            {
                return this.pauseIntegration;
            }

            set
            {
                this.pauseIntegration = value;
                if (null != this.PropertyChanged)
                {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("PauseIntegration"));
                }
            }
        }
        /// <summary>
        /// Track whether Dispose has been called
        /// </summary>
        private bool disposed;
        /// <summary>
        /// Intermediate storage for the depth float data converted from depth image frame
        /// </summary>
        private FusionFloatImageFrame depthFloatFrame;

        /// <summary>
        /// Per-pixel alignment values
        /// </summary>
        private FusionFloatImageFrame deltaFromReferenceFrame;

        /// <summary>
        /// minT alignment energy for frame
        /// </summary>
        private float alignmentEnergy;

        /// <summary>
        /// Shaded surface frame from shading point cloud frame
        /// </summary>
        private FusionColorImageFrame shadedSurfaceFrame;

        /// <summary>
        /// Shaded surface normals frame from shading point cloud frame
        /// </summary>
        private FusionColorImageFrame shadedSurfaceNormalsFrame;

        /// <summary>
        /// Calculated point cloud frame from image integration
        /// </summary>
        private FusionPointCloudImageFrame pointCloudFrame;




        /// <summary>
        /// The reconstruction volume voxel resolution in the X axis
        /// At a setting of 256vpm the volume is 512 / 256 = 2m wide
        /// </summary>
        private int voxelsX = 512;

        /// <summary>
        /// The reconstruction volume voxel resolution in the Y axis
        /// At a setting of 256vpm the volume is 384 / 256 = 1.5m high
        /// </summary>
        private int voxelsY = 384;

        /// <summary>
        /// The reconstruction volume voxel resolution in the Z axis
        /// At a setting of 256vpm the volume is 512 / 256 = 2m deep
        /// </summary>
        private int voxelsZ = 512;


        /// <summary>
        /// The sensor1
        /// </summary>
        private KinectSensor sensor1;

        /// <summary>
        /// The sensor2
        /// </summary>
        private KinectSensor sensor2;

        /// <summary>
        /// Initializes a new instance of the <see cref="MainWindow" /> class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();

            this.sensor1 = KinectSensor.KinectSensors[0];
            this.sensor1.ColorStream.Enable();
            this.sensor1.ColorFrameReady += sensor1_ColorFrameReady;
            this.sensor1.Start();

            this.RecreateReconstruction();

            //Loaded += MainWindow_Loaded;
        }

        /// <summary>
        /// Handles the Loaded event of the MainWindow control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="RoutedEventArgs" /> instance containing the event data.</param>
        void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {

            

            this.sensor1 = KinectSensor.KinectSensors[0];
            this.sensor1.ColorStream.Enable();
            this.sensor1.ColorFrameReady += sensor1_ColorFrameReady;
            this.sensor1.Start();

            this.RecreateReconstruction();

            this.sensor2 = KinectSensor.KinectSensors[1];
            this.sensor2.ColorStream.Enable();
            this.sensor2.ColorFrameReady += sensor2_ColorFrameReady;
            this.sensor2.Start();

            
        }

        /// <summary>
        /// Handles the DepthFrameReady event of the sensor2 control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="DepthImageFrameReadyEventArgs" /> instance containing the event data.</param>
        void sensor2_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthimageFrame = e.OpenDepthImageFrame())
            {
                if (depthimageFrame == null)
                {
                    return;
                }

                short[] pixelData = new short[depthimageFrame.PixelDataLength];
                int stride = depthimageFrame.Width * 2;
                depthimageFrame.CopyPixelDataTo(pixelData);

                SensorBImageViewer.Source = BitmapSource.Create(
                   depthimageFrame.Width, depthimageFrame.Height, 96, 96, PixelFormats.Gray16, null, pixelData, stride);
            }
        }

        /// <summary>
        /// Handles the DepthFrameReady event of the sensor1 control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="DepthImageFrameReadyEventArgs" /> instance containing the event data.</param>
        void sensor1_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthimageFrame = e.OpenDepthImageFrame())
            {
                if (depthimageFrame == null)
                {
                    return;
                }

                short[] pixelData = new short[depthimageFrame.PixelDataLength];
                int stride = depthimageFrame.Width * 2;
                depthimageFrame.CopyPixelDataTo(pixelData);

                SensorAImageViewer.Source = BitmapSource.Create(
                   depthimageFrame.Width, depthimageFrame.Height, 96, 96, PixelFormats.Gray16, null, pixelData, stride);
            }
        }

        /// <summary>
        /// Handles the ColorFrameReady event of the sensor2 control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="ColorImageFrameReadyEventArgs" /> instance containing the event data.</param>
        void sensor2_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame imageframe = e.OpenColorImageFrame())
            {
                if (imageframe == null)
                {
                    return;
                }

                byte[] pixelData = new byte[imageframe.PixelDataLength];

                imageframe.CopyPixelDataTo(pixelData);

                SensorBImageViewer.Source = BitmapSource.Create(imageframe.Width, imageframe.Height, 96, 96, PixelFormats.Bgr32, null, pixelData, imageframe.Width * 4);
            }
        }

        /// <summary>
        /// Handles the ColorFrameReady event of the sensor1 control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="ColorImageFrameReadyEventArgs" /> instance containing the event data.</param>
        void sensor1_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame imageframe = e.OpenColorImageFrame())
            {
                if (imageframe == null)
                {
                    return;
                }

                byte[] pixelData = new byte[imageframe.PixelDataLength];

                imageframe.CopyPixelDataTo(pixelData);

                SensorAImageViewer.Source = BitmapSource.Create(imageframe.Width, imageframe.Height, 96, 96, PixelFormats.Bgr32, null, pixelData, imageframe.Width * 4);
            }
        }

        /// <summary>
        /// Handles the event event of the CheckBoxDevice1DepthCheck control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="RoutedEventArgs" /> instance containing the event data.</param>
        private void CheckBoxDevice1DepthCheck_event(object sender, RoutedEventArgs e)
        {
            CheckBox chkbox = (CheckBox)sender;
            if (chkbox.IsChecked == true)
            {
                EnableDepthStreamforDevice1();
            }
            else if (chkbox.IsChecked == false)
            {
                EnableColorStreamforDevice1();
            }
        }

        /// <summary>
        /// Enables the color streamfor device1.
        /// </summary>
        private void EnableColorStreamforDevice1()
        {
            if (this.sensor1.DepthStream.IsEnabled)
            {
                this.sensor1.DepthStream.Disable();
                this.sensor1.ColorStream.Enable();
                this.sensor1.ColorFrameReady += sensor1_ColorFrameReady;
            }
        }

        /// <summary>
        /// Enables the depth streamfor device1.
        /// </summary>
        private void EnableDepthStreamforDevice1()
        {
            if (this.sensor1.ColorStream.IsEnabled)
            {
                this.sensor1.ColorStream.Disable();
                this.sensor1.DepthStream.Enable();
                this.sensor1.DepthFrameReady += sensor1_DepthFrameReady;
            }
        }

        /// <summary>
        /// Handles the event event of the CheckBoxDevice2DepthCheck control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="RoutedEventArgs" /> instance containing the event data.</param>
        private void CheckBoxDevice2DepthCheck_event(object sender, RoutedEventArgs e)
        {
            CheckBox chkbox = (CheckBox)sender;
            if (chkbox.IsChecked == true)
            {
                EnableDepthStreamforDevice2();
            }
            else if (chkbox.IsChecked == false)
            {
                EnableColorStreamforDevice2();
            }
        }

        /// <summary>
        /// Enables the color streamfor device2.
        /// </summary>
        private void EnableColorStreamforDevice2()
        {
            if (this.sensor2.DepthStream.IsEnabled)
            {
                this.sensor2.DepthStream.Disable();
                this.sensor2.ColorStream.Enable();
                this.sensor2.ColorFrameReady += sensor2_ColorFrameReady;
            }
        }

        /// <summary>
        /// Enables the depth streamfor device2.
        /// </summary>
        private void EnableDepthStreamforDevice2()
        {
            if (this.sensor2.ColorStream.IsEnabled)
            {
                this.sensor2.ColorStream.Disable();
                this.sensor2.DepthStream.Enable();
                this.sensor2.DepthFrameReady += sensor2_DepthFrameReady;
            }
        }



        /// <summary>
        /// Handler for click event from "Create Mesh" button
        /// </summary>
        /// <param name="sender">Event sender</param>
        /// <param name="e">Event arguments</param>
        private void CreateMeshButtonClick(object sender, RoutedEventArgs e)
        {
            if (null == this.volume)
            {
                this.ShowStatusMessage(Properties.Resources.MeshNullVolume);
                return;
            }

            this.savingMesh = true;

            // Mark the start time of saving mesh
            DateTime begining = DateTime.Now;

            try
            {
                this.ShowStatusMessage(Properties.Resources.SavingMesh);

                Mesh mesh = this.volume.CalculateMesh(1);

                Microsoft.Win32.SaveFileDialog dialog = new Microsoft.Win32.SaveFileDialog();

                if (true == this.stlFormat.IsChecked)
                {
                    dialog.FileName = "MeshedReconstruction.stl";
                    dialog.Filter = "STL Mesh Files|*.stl|All Files|*.*";
                }
                else
                {
                    dialog.FileName = "MeshedReconstruction.obj";
                    dialog.Filter = "OBJ Mesh Files|*.obj|All Files|*.*";
                }

                if (true == dialog.ShowDialog())
                {
                    if (true == this.stlFormat.IsChecked)
                    {
                        using (BinaryWriter writer = new BinaryWriter(dialog.OpenFile()))
                        {
                            SaveBinarySTLMesh(mesh, writer);
                        }
                    }
                    else
                    {
                        using (StreamWriter writer = new StreamWriter(dialog.FileName))
                        {
                            SaveAsciiObjMesh(mesh, writer);
                        }
                    }

                    this.ShowStatusMessage(Properties.Resources.MeshSaved);
                }
                else
                {
                    this.ShowStatusMessage(Properties.Resources.MeshSaveCanceled);
                }
            }
            catch (ArgumentException)
            {
                this.ShowStatusMessage(Properties.Resources.ErrorSaveMesh);
            }
            catch (InvalidOperationException)
            {
                this.ShowStatusMessage(Properties.Resources.ErrorSaveMesh);
            }
            catch (IOException)
            {
                this.ShowStatusMessage(Properties.Resources.ErrorSaveMesh);
            }

            // Update timestamp of last frame to avoid auto reset reconstruction
            this.lastFrameTimestamp += (long)(DateTime.Now - begining).TotalMilliseconds;

            this.savingMesh = false;
        }



        /// <summary>
        /// Show exception info on status bar
        /// </summary>
        /// <param name="message">Message to show on status bar</param>
        private void ShowStatusMessage(string message)
        {
            this.Dispatcher.BeginInvoke((Action)(() =>
            {
                this.ResetFps();
                this.statusBarText.Text = message;
            }));
        }



        /// <summary>
        /// Reset FPS timer and counter
        /// </summary>
        private void ResetFps()
        {
            // Restart fps timer
            if (null != this.fpsTimer)
            {
                this.fpsTimer.Stop();
                this.fpsTimer.Start();
            }

            // Reset frame counter
            this.processedFrameCount = 0;
            this.lastFPSTimestamp = DateTime.Now;
        }




        /// <summary>
        /// Save mesh in binary .STL file
        /// </summary>
        /// <param name="mesh">Calculated mesh object</param>
        /// <param name="writer">Binary file writer</param>
        /// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save</param>
        private static void SaveBinarySTLMesh(Mesh mesh, BinaryWriter writer, bool flipYZ = true)
        {
            var vertices = mesh.GetVertices();
            var normals = mesh.GetNormals();
            var indices = mesh.GetTriangleIndexes();

            // Check mesh arguments
            if (0 == vertices.Count || 0 != vertices.Count % 3 || vertices.Count != indices.Count)
            {
                throw new ArgumentException(Properties.Resources.InvalidMeshArgument);
            }

            char[] header = new char[80];
            writer.Write(header);

            // Write number of triangles
            int triangles = vertices.Count / 3;
            writer.Write(triangles);

            // Sequentially write the normal, 3 vertices of the triangle and attribute, for each triangle
            for (int i = 0; i < triangles; i++)
            {
                // Write normal
                var normal = normals[i * 3];
                writer.Write(normal.X);
                writer.Write(flipYZ ? -normal.Y : normal.Y);
                writer.Write(flipYZ ? -normal.Z : normal.Z);

                // Write vertices
                for (int j = 0; j < 3; j++)
                {
                    var vertex = vertices[(i * 3) + j];
                    writer.Write(vertex.X);
                    writer.Write(flipYZ ? -vertex.Y : vertex.Y);
                    writer.Write(flipYZ ? -vertex.Z : vertex.Z);
                }

                ushort attribute = 0;
                writer.Write(attribute);
            }
        }

        /// <summary>
        /// Save mesh in ASCII Wavefront .OBJ file
        /// </summary>
        /// <param name="mesh">Calculated mesh object</param>
        /// <param name="writer">Stream writer</param>
        /// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save</param>
        private static void SaveAsciiObjMesh(Mesh mesh, StreamWriter writer, bool flipYZ = true)
        {
            var vertices = mesh.GetVertices();
            var normals = mesh.GetNormals();
            var indices = mesh.GetTriangleIndexes();

            // Check mesh arguments
            if (0 == vertices.Count || 0 != vertices.Count % 3 || vertices.Count != indices.Count)
            {
                throw new ArgumentException(Properties.Resources.InvalidMeshArgument);
            }

            // Write the header lines
            writer.WriteLine("#");
            writer.WriteLine("# OBJ file created by Microsoft Kinect Fusion");
            writer.WriteLine("#");

            // Sequentially write the 3 vertices of the triangle, for each triangle
            for (int i = 0; i < vertices.Count; i++)
            {
                var vertex = vertices[i];

                string vertexString = "v " + vertex.X.ToString(CultureInfo.CurrentCulture) + " ";

                if (flipYZ)
                {
                    vertexString += (-vertex.Y).ToString(CultureInfo.CurrentCulture) + " " + (-vertex.Z).ToString(CultureInfo.CurrentCulture);
                }
                else
                {
                    vertexString += vertex.Y.ToString(CultureInfo.CurrentCulture) + " " + vertex.Z.ToString(CultureInfo.CurrentCulture);
                }

                writer.WriteLine(vertexString);
            }

            // Sequentially write the 3 normals of the triangle, for each triangle
            for (int i = 0; i < normals.Count; i++)
            {
                var normal = normals[i];

                string normalString = "vn " + normal.X.ToString(CultureInfo.CurrentCulture) + " ";

                if (flipYZ)
                {
                    normalString += (-normal.Y).ToString(CultureInfo.CurrentCulture) + " " + (-normal.Z).ToString(CultureInfo.CurrentCulture);
                }
                else
                {
                    normalString += normal.Y.ToString(CultureInfo.CurrentCulture) + " " + normal.Z.ToString(CultureInfo.CurrentCulture);
                }

                writer.WriteLine(normalString);
            }

            // Sequentially write the 3 vertex indices of the triangle face, for each triangle
            // Note this is typically 1-indexed in an OBJ file when using absolute referencing!
            for (int i = 0; i < vertices.Count / 3; i++)
            {
                string baseIndex0 = ((i * 3) + 1).ToString(CultureInfo.CurrentCulture);
                string baseIndex1 = ((i * 3) + 2).ToString(CultureInfo.CurrentCulture);
                string baseIndex2 = ((i * 3) + 3).ToString(CultureInfo.CurrentCulture);

                string faceString = "f " + baseIndex0 + "//" + baseIndex0 + " " + baseIndex1 + "//" + baseIndex1 + " " + baseIndex2 + "//" + baseIndex2;
                writer.WriteLine(faceString);
            }
        }
        /// <summary>
        /// Re-create the reconstruction object
        /// </summary>
        /// <returns>Indicate success or failure</returns>
        private bool RecreateReconstruction()
        {
            // Check if sensor has been initialized
            if (null == this.sensor1)
            {
                return false;
            }

            if (null != this.volume)
            {
                this.volume.Dispose();
            }

            try
            {
                // The zero-based GPU index to choose for reconstruction processing if the 
                // ReconstructionProcessor AMP options are selected.
                // Here we automatically choose a device to use for processing by passing -1, 
                int deviceIndex = -1;

                ReconstructionParameters volParam = new ReconstructionParameters(this.voxelsPerMeter, this.voxelsX, this.voxelsY, this.voxelsZ);

                // Set the world-view transform to identity, so the world origin is the initial camera location.
                this.worldToCameraTransform = Matrix4.Identity;

                this.volume = Reconstruction.FusionCreateReconstruction(volParam, ProcessorType, deviceIndex, this.worldToCameraTransform);

                this.defaultWorldToVolumeTransform = this.volume.GetCurrentWorldToVolumeTransform();

                if (this.translateResetPoseByMinDepthThreshold)
                {
                    this.ResetReconstruction();
                }

                // Reset "Pause Integration"
                if (this.PauseIntegration)
                {
                    this.PauseIntegration = false;
                }

                return true;
            }
            catch (ArgumentException)
            {
                this.volume = null;
                this.ShowStatusMessage(Properties.Resources.VolumeResolution);
            }
            catch (InvalidOperationException ex)
            {
                this.volume = null;
                this.ShowStatusMessage(ex.Message);
            }
            catch (DllNotFoundException)
            {
                this.volume = null;
                this.ShowStatusMessage(Properties.Resources.MissingPrerequisite);
            }
            catch (OutOfMemoryException)
            {
                this.volume = null;
                this.ShowStatusMessage(Properties.Resources.OutOfMemory);
            }

            return false;
        }

        /// <summary>
        /// Handler for click event from "Reset Reconstruction" button
        /// </summary>
        /// <param name="sender">Event sender</param>
        /// <param name="e">Event arguments</param>
        private void ResetReconstructionButtonClick(object sender, RoutedEventArgs e)
        {
            if (null == this.sensor1)
            {
                return;
            }

            // Reset volume
            this.ResetReconstruction();

            // Update manual reset information to status bar
            this.ShowStatusMessage(Properties.Resources.ResetVolume);
        }

        /// <summary>
        /// Reset reconstruction object to initial state
        /// </summary>
        private void ResetReconstruction()
        {
            if (null == this.sensor1)
            {
                return;
            }

            // Reset tracking error counter
            this.trackingErrorCount = 0;

            // Set the world-view transform to identity, so the world origin is the initial camera location.
            this.worldToCameraTransform = Matrix4.Identity;

            // Reset volume
            if (null != this.volume)
            {
                try
                {
                    // Translate the reconstruction volume location away from the world origin by an amount equal
                    // to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
                    // If set false, the default world origin is set to the center of the front face of the 
                    // volume, which has the effect of locating the volume directly in front of the initial camera
                    // position with the +Z axis into the volume along the initial camera direction of view.
                    if (this.translateResetPoseByMinDepthThreshold)
                    {
                        Matrix4 worldToVolumeTransform = this.defaultWorldToVolumeTransform;

                        // Translate the volume in the Z axis by the minDepthThreshold distance
                        float minDist = (this.minDepthClip < this.maxDepthClip) ? this.minDepthClip : this.maxDepthClip;
                        worldToVolumeTransform.M43 -= minDist * this.voxelsPerMeter;

                        this.volume.ResetReconstruction(this.worldToCameraTransform, worldToVolumeTransform);
                    }
                    else
                    {
                        this.volume.ResetReconstruction(this.worldToCameraTransform);
                    }

                    if (this.PauseIntegration)
                    {
                        this.PauseIntegration = false;
                    }
                }
                catch (InvalidOperationException)
                {
                    this.ShowStatusMessage(Properties.Resources.ResetFailed);
                }
            }

            // Reset fps counter
            this.ResetFps();
        }

        /// <summary>
        /// Dispose resources
        /// </summary>
        public void Dispose()
        {
            this.Dispose(true);

            // This object will be cleaned up by the Dispose method.
            GC.SuppressFinalize(this);
        }
        /// <summary>
        /// Frees all memory associated with the FusionImageFrame.
        /// </summary>
        /// <param name="disposing">Whether the function was called from Dispose.</param>
        protected virtual void Dispose(bool disposing)
        {
            if (!this.disposed)
            {
                if (disposing)
                {
                    if (null != this.depthFloatFrame)
                    {
                        this.depthFloatFrame.Dispose();
                    }

                    if (null != this.deltaFromReferenceFrame)
                    {
                        this.deltaFromReferenceFrame.Dispose();
                    }

                    if (null != this.shadedSurfaceFrame)
                    {
                        this.shadedSurfaceFrame.Dispose();
                    }

                    if (null != this.shadedSurfaceNormalsFrame)
                    {
                        this.shadedSurfaceNormalsFrame.Dispose();
                    }

                    if (null != this.pointCloudFrame)
                    {
                        this.pointCloudFrame.Dispose();
                    }

                    if (null != this.volume)
                    {
                        this.volume.Dispose();
                    }
                }
            }

            this.disposed = true;
        }
    }
}
