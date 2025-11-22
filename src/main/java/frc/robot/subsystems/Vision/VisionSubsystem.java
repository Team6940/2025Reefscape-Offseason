package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final String limelightName = "limelight-neural";
    private final Pose3d cameraPose3d = VisionConstants.cameraPose3d;
    public static VisionSubsystem vision;

    public static VisionSubsystem getInstance() {
        return vision == null ? vision = new VisionSubsystem() : vision;
    }

    public VisionSubsystem() {
        LimelightHelpers.setPipelineIndex(limelightName, 2);
        for (var point : VisionConstants.tyToDistancePoints) {
            VisionConstants.tyToDistance.put(point.getX(), point.getY());
        }
    }

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

    /*
     * Get IMU pitch data
     */
    public double getIMUPitch() {
        LimelightHelpers.IMUData imu = getIMUData();
        return imu != null ? imu.Pitch : 0.0;
    }

    /*
     * use this func carefully! 'cause different perspectives varies greatly
     * estimate distance from ta
     */
    public static double estimateDistanceFromTA(double ta) {
        if (ta <= 0.0)
            return Double.POSITIVE_INFINITY; // No valid target

        // Calibration constant (tune this!)
        // Example: if ta = 4% corresponds to 1 meter, then k ≈ 1 * sqrt(4) = 2.0
        double k = 2.0; // You must adjust this constant experimentally

        return k / Math.sqrt(ta);
    }

    /*
     * calculate the best intake pose
     */
    public Pose2d getObjectFieldRelativePose2d(LimelightHelpers.RawDetection detection, Pose2d robotPose) {
        Rotation2d cameraYawOffset = new Rotation2d(cameraPose3d.getRotation().getZ());
        var fieldRelTranslation2d = getObjectFieldRelativeTranslation2d(detection, robotPose);
        Rotation2d targetRotation = robotPose.getTranslation().minus(fieldRelTranslation2d).getAngle()
                .rotateBy(cameraYawOffset);
        return new Pose2d(fieldRelTranslation2d, targetRotation);
    }

    /*
     * Transform translationRobotRelative to translationFieldRelative
     */
    public Translation2d getObjectFieldRelativeTranslation2d(LimelightHelpers.RawDetection detection,
            Pose2d robotPose) {
        if (detection == null || robotPose == null)
            return null;

        // 1. Robot-relative
        Translation2d robotRelTranslation2d = getObjectRobotRelativeTranslation2d(detection);
        if (robotRelTranslation2d == null)
            return null;

        // 2. Rotate robot-relative detection into field frame
        Translation2d fieldRelTranslation2d = robotRelTranslation2d
                .rotateBy(robotPose.getRotation())
                .plus(new Translation2d(robotPose.getX(), robotPose.getY()));

        return fieldRelTranslation2d;
    }

    /*
     * Transform translationCameraRelative to translationRobotRelative
     */
    public Translation2d getObjectRobotRelativeTranslation2d(LimelightHelpers.RawDetection detection) {
        if (detection == null)
            return null;

        // 1. Camera-relative
        Translation2d camRelTranslation2d = getObjectCameraRelativeTranslation2d(detection);
        if (camRelTranslation2d == null)
            return null;

        Translation2d cameraOffset = new Translation2d(cameraPose3d.getX(), cameraPose3d.getY());
        Rotation2d cameraYawOffset = new Rotation2d(cameraPose3d.getRotation().getZ());// PI/0 theoretically

        // 2. Rotate camera-relative detection into robot frame
        Translation2d robotRelTranslation2d = camRelTranslation2d.rotateBy(cameraYawOffset).plus(cameraOffset);

        return robotRelTranslation2d;
    }

    /*
     * Estimate object translation (camera relative) by ty & tx
     */
    public Translation2d getObjectCameraRelativeTranslation2d(LimelightHelpers.RawDetection detection) {
        if (detection == null)
            return null;

        double txRad = Math.toRadians(detection.txnc);
        double tyRad = Math.toRadians(detection.tync);

        double distance = VisionConstants.tyToDistance.get(tyRad);
        double xRel = distance;
        double yRel = Math.sqrt(distance * distance + cameraPose3d.getZ() * cameraPose3d.getZ()) * Math.tan(txRad);
        // calculate yRel from simple math

        return new Translation2d(xRel, yRel);// x: seeing forward from the cam ; y: left/right shifting
    }

    /**
     * Display detection information to dashboard
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LL4 Has Target", hasValidTarget());
        SmartDashboard.putString("LL4 Class", getDetectedClassName());
        SmartDashboard.putNumber("LL4 Class ID", getDetectedClassId());

        // showing how many neural detections are found
        LimelightHelpers.RawDetection[] detections = getDetections();
        SmartDashboard.putNumber("LL4 Neural Detections", detections != null ? detections.length : 0);

        // printing primary object translations
        LimelightHelpers.RawDetection primaryObject = getPrimaryObject();
        if (primaryObject != null) {
            Translation2d cameraObjectTranslation = getObjectCameraRelativeTranslation2d(primaryObject);
            SmartDashboard.putString("Primary Object Translation CAM", cameraObjectTranslation.toString());
        } else {
            SmartDashboard.putString("Primary Object Translation CAM", "None");
        }
        if (primaryObject != null) {
            Translation2d robotObjectTranslation = getObjectRobotRelativeTranslation2d(primaryObject);
            SmartDashboard.putString("Primary Object Translation RBT", robotObjectTranslation.toString());
        } else {
            SmartDashboard.putString("Primary Object Translation RBT", "None");
        }
        if (primaryObject != null) {
            Translation2d fieldObjectTranslation = getObjectFieldRelativeTranslation2d(primaryObject,
                    RobotContainer.chassis.getPose());
            SmartDashboard.putString("Primary Object Translation FLD", fieldObjectTranslation.toString());
        } else {
            SmartDashboard.putString("Primary Object Translation FLD", "None");
        }
    }
}