package org.firstinspires.ftc.teamcode.Subsystems.Cameras;

import android.graphics.Canvas;
import android.graphics.Color;
import android.util.Size;
import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.VisionProcessorMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Ramp Camera Subsystem managing webcam video streaming, image orientation flipping,
 * and OpenCV color blob detection for purple and green game artifacts.
 *
 * @author kw126
 * @author Glenn
 * @author knutt5
 */
public class RampCamera extends SubsystemBase {

    // Camera vision portal
    private volatile VisionPortal visionPortal;

    // Available vision processors to be used with camera
    private volatile ColorBlobLocatorProcessor purpleBlobProcessor;
    private volatile ColorBlobLocatorProcessor greenBlobProcessor;

    // Processor to flip image right-side-up
    private volatile ImageFlipProcessor flipProcessor;

    // Current selected vision mode
    private VisionProcessorMode currentMode = VisionProcessorMode.NONE;

    /**
     * Initializes the ramp camera subsystem, configures the color blob locator processors
     * for purple and green artifacts, builds the VisionPortal pipeline, and enables artifact detection.
     *
     * @author kw126
     *
     * @param cameraName hardware map identifier for the USB webcam
     */
    public RampCamera(String cameraName) {
        // Create processor to flip image right-side up
        flipProcessor = new ImageFlipProcessor();

        // Create purple blob processor
        purpleBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // Add contour filtering for purple artifacts
        purpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 2000, 76800));
        purpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1));

        // Sort purple blobs by contour area descending
        purpleBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));

        // Create green blob processor
        greenBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // Add contour filtering for green artifacts
        greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 2000, 76800));
        greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1));

        // Sort green blobs by contour area descending
        greenBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));

        // Build the camera vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(320, 240))
                .addProcessors(flipProcessor)
                .addProcessors(purpleBlobProcessor, greenBlobProcessor)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // By default enable artifact detection processors
        setVisionProcessingMode(VisionProcessorMode.ARTIFACT);

        // Show camera stream on panels dashboard
        enableDashBoardView(true);
    }

    /**
     * Method called periodically by the command scheduler.
     *
     * @author kw126
     */
    @Override
    public void periodic() {
        // Periodic camera logic if required
    }

    /**
     * Configures the active vision processing mode for the camera subsystem.
     * Enables or disables color blob processors accordingly.
     *
     * @author kw126
     *
     * @param newMode desired vision processing mode from VisionProcessorMode enum
     */
    public void setVisionProcessingMode(@NonNull VisionProcessorMode newMode) {
        enableDashBoardView(false);
        RobotContainer.ActiveOpMode.sleep(100);

        // Enable or disable processors depending on the selected mode
        switch (newMode) {
            case NONE:
                currentMode = VisionProcessorMode.NONE;
                visionPortal.setProcessorEnabled(purpleBlobProcessor, false);
                visionPortal.setProcessorEnabled(greenBlobProcessor, false);
                break;
            case ARTIFACT:
                currentMode = VisionProcessorMode.ARTIFACT;
                visionPortal.setProcessorEnabled(purpleBlobProcessor, true);
                visionPortal.setProcessorEnabled(greenBlobProcessor, true);
                break;
            default:
                break;
        }

        RobotContainer.ActiveOpMode.sleep(100);
        enableDashBoardView(true);
    }

    /**
     * Retrieves the current vision processing mode of the camera.
     *
     * @author kw126
     *
     * @return active VisionProcessorMode
     */
    public VisionProcessorMode getVisionProcessingMode() {
        return currentMode;
    }

    /**
     * Retrieves all detected color blobs (purple and green) from the camera, sorted by area.
     *
     * @author kw126
     *
     * @return list containing detected color blobs
     */
    public List<ColorBlobLocatorProcessor.Blob> GetAllBlobDetections() {
        List<ColorBlobLocatorProcessor.Blob> blobs;

        switch (currentMode) {
            case ARTIFACT:
                blobs = purpleBlobProcessor.getBlobs();
                blobs.addAll(greenBlobProcessor.getBlobs());
                break;
            default:
                blobs = new ArrayList<>();
        }

        // Sort blobs by contour area descending
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);

        // Return only the primary detected blob if present
        if (blobs != null && !blobs.isEmpty()) {
            blobs = Collections.singletonList(blobs.get(0));
        }
        return blobs;
    }

    /**
     * Retrieves all detected purple artifact blobs from the camera.
     *
     * @author kw126
     *
     * @return list containing detected purple blobs
     */
    public List<ColorBlobLocatorProcessor.Blob> GetPurpleBlobDetections() {
        List<ColorBlobLocatorProcessor.Blob> blobs;

        switch (currentMode) {
            case ARTIFACT:
                blobs = purpleBlobProcessor.getBlobs();
                break;
            default:
                blobs = new ArrayList<>();
        }

        // Sort by area descending
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);

        // Return top blob
        if (blobs != null && !blobs.isEmpty()) {
            blobs = Collections.singletonList(blobs.get(0));
        }

        return blobs;
    }

    /**
     * Retrieves all detected green artifact blobs from the camera.
     *
     * @author kw126
     *
     * @return list containing detected green blobs
     */
    public List<ColorBlobLocatorProcessor.Blob> GetGreenBlobDetections() {
        List<ColorBlobLocatorProcessor.Blob> blobs;

        switch (currentMode) {
            case ARTIFACT:
                blobs = greenBlobProcessor.getBlobs();
                break;
            default:
                blobs = new ArrayList<>();
        }

        // Sort by area descending
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);

        // Return top blob
        if (blobs != null && !blobs.isEmpty()) {
            blobs = Collections.singletonList(blobs.get(0));
        }

        return blobs;
    }

    /**
     * Calculates the center (X, Y) coordinate of a bounding box.
     *
     * @author kw126
     *
     * @param boundingBox the OpenCV Rect bounding box
     * @return OpenCV Point representing the center coordinates
     */
    private Point calculateCenter(Rect boundingBox) {
        // Calculate center coordinate
        double centerX = boundingBox.x + boundingBox.width / 2.0;
        double centerY = boundingBox.y + boundingBox.height / 2.0;
        return new Point(centerX, centerY);
    }

    /**
     * Enables or disables dashboard streaming for this camera.
     *
     * @author kw126
     *
     * @param enable true to enable dashboard stream, false to disable
     */
    public void enableDashBoardView(boolean enable) {
        // Placeholder for dashboard camera streaming
    }

    /**
     * Displays the camera stream on the Driver Station via CameraStreamServer.
     *
     * @author kw126
     */
    public void enableCameraStream() {
        // Attach VisionPortal to the Driver Station stream server
        CameraStreamServer.getInstance().setSource(visionPortal);
    }

    /**
     * Enables or disables camera live preview.
     *
     * @author kw126
     *
     * @param enable true to resume live view, false to stop
     */
    public void enableLiveView(boolean enable) {
        if (enable)
            visionPortal.resumeLiveView();
        else
            visionPortal.stopLiveView();
    }

    /**
     * Retrieves the current camera processing frame rate in frames per second.
     *
     * @author kw126
     *
     * @return current frame rate in FPS
     */
    public double GetCameraFPS() {
        return visionPortal.getFps();
    }

    /**
     * Checks if the camera is currently in active streaming mode.
     *
     * @author kw126
     *
     * @return true if camera state is STREAMING, false otherwise
     */
    public boolean cameraReady() {
        return (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
    }

    /**
     * Sets manual exposure duration and gain on the camera sensor.
     *
     * @author kw126
     *
     * @param exposureMS exposure time in milliseconds
     * @param gain sensor amplification gain
     */
    private void setCameraExposure(int exposureMS, int gain) {
        // Wait until camera enters streaming mode
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }

        // Configure manual exposure control
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            RobotContainer.ActiveOpMode.sleep(50);
        }

        // Apply exposure and gain
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        RobotContainer.ActiveOpMode.sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        RobotContainer.ActiveOpMode.sleep(20);
    }

    /**
     * Sets camera exposure control to automatic.
     *
     * @author kw126
     */
    private void SetAutoCameraExposure() {
        // Wait until streaming
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }

        // Configure auto exposure
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Auto);
    }

    /**
     * Custom OpenCV VisionProcessor that flips the camera frame 180 degrees
     * (both horizontally and vertically) to correct for camera mounting orientation.
     *
     * @author kw126
     */
    public static class ImageFlipProcessor implements VisionProcessor {
        /**
         * Initializes the flip processor.
         *
         * @author kw126
         *
         * @param width image width in pixels
         * @param height image height in pixels
         * @param cameraControlInternal camera calibration data
         */
        @Override
        public void init(int width, int height, CameraCalibration cameraControlInternal) {
            // Initialization steps if necessary
        }

        /**
         * Flips the incoming frame 180 degrees using OpenCV Core.flip.
         *
         * @author kw126
         *
         * @param frame input image matrix from webcam
         * @param captureTimeNanos timestamp in nanoseconds
         * @return null user context
         */
        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // Flip both horizontally and vertically (-1)
            Core.flip(frame, frame, -1);
            return null;
        }

        /**
         * Post-frame canvas overlay drawing.
         *
         * @author kw126
         *
         * @param canvas drawing canvas
         * @param onscreenWidth screen width in pixels
         * @param onscreenHeight screen height in pixels
         * @param scaleBmpPxToCanvasPx pixel scaling factor
         * @param scaleCanvasDensity density scaling factor
         * @param userContext custom user context object
         */
        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // No custom canvas drawing needed
        }
    }
}