package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final String limelightName = "limelight-4";

    // public VisionSubsystem(String limelightName) {
    // this.limelightName = limelightName;
    // }

    /**
     * Get all current neural detector results
     */
    public LimelightHelpers.RawDetection[] getDetections() {
        return LimelightHelpers.getRawDetections(limelightName);
    }

    /**
     * Check if Limelight has any valid targets
     */
    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Get the class name of the primary detected object
     */
    public String getDetectedClassName() {
        return LimelightHelpers.getDetectorClass(limelightName);
    }

    /**
     * Get the class ID of the primary detected object
     */
    public int getDetectedClassId() {
        return LimelightHelpers.getDetectorClassIndex(limelightName);
    }

    /**
     * Returns the primary detected object based on largest area (ta).
     * Returns null if no detections exist.
     */
    public LimelightHelpers.RawDetection getPrimaryObject() {
        LimelightHelpers.RawDetection[] detections = getDetections();
        if (detections == null || detections.length == 0) {
            return null;
        }

        LimelightHelpers.RawDetection primary = detections[0];
        double maxArea = primary.ta;

        for (int i = 1; i < detections.length; i++) {
            LimelightHelpers.RawDetection det = detections[i];
            if (det.ta > maxArea) {
                primary = det;
                maxArea = det.ta;
            }
        }

        return primary;
    }

    /**
     * Get the IMU data from the Limelight
     */
    public LimelightHelpers.IMUData getIMUData() {
        return LimelightHelpers.getIMUData(limelightName);
    }

    public double getIMUPitch() {
        LimelightHelpers.IMUData imu = getIMUData();
        return imu != null ? imu.Pitch : 0.0;
    }

    public static Translation2d getObjectCameraRelativeTranslation2d(
            LimelightHelpers.RawDetection detection,
            double cameraHeight,
            double cameraPitchDegrees) {
        if (detection == null)
            return null;

        // Convert angles to radians
        double txRad = Math.toRadians(detection.txnc);
        double tyRad = Math.toRadians(detection.tync);
        double pitchRad = Math.toRadians(cameraPitchDegrees);

        // Distance from camera to object along ground
        double distance = cameraHeight / Math.tan(pitchRad + tyRad);

        // Robot-relative coordinates
        double xRel = distance * Math.cos(txRad); // forward
        double yRel = distance * Math.sin(txRad); // rightward (+)

        return new Translation2d(xRel, yRel);
    }

    /**
     * Display detection information to dashboard
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LL4 Has Target", hasValidTarget());
        SmartDashboard.putString("LL4 Class", getDetectedClassName());
        SmartDashboard.putNumber("LL4 Class ID", getDetectedClassId());

        // Example: show how many neural detections are found
        LimelightHelpers.RawDetection[] detections = getDetections();
        SmartDashboard.putNumber("LL4 Neural Detections", detections != null ? detections.length : 0);
    }
}