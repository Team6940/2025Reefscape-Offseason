package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final String limelightName = "limelight-neural";
    private final Pose3d cameraPose3d = new Pose3d(new Translation3d(0.13, -0.17, 0.85),
            new Rotation3d(0, 0.48363073, 0));
    public static VisionSubsystem vision;

    public static VisionSubsystem getInstance() {
        return vision == null ? vision = new VisionSubsystem() : vision;
    }

    public VisionSubsystem() {
        LimelightHelpers.setPipelineIndex(limelightName, 2);
        for (var a : VisionConstants.tyToDistancePoints) {
            VisionConstants.tyToDistance.put(a.getX(), a.getY());
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

    public double getIMUPitch() {
        LimelightHelpers.IMUData imu = getIMUData();
        return imu != null ? imu.Pitch : 0.0;
    }

    // use this func carefully! 'cause different perspectives varies greatly
    public static double estimateDistanceFromTA(double ta) {
        if (ta <= 0.0)
            return Double.POSITIVE_INFINITY; // No valid target

        // Calibration constant (tune this!)
        // Example: if ta = 4% corresponds to 1 meter, then k ≈ 1 * sqrt(4) = 2.0
        double k = 2.0; // You must adjust this constant experimentally

        return k / Math.sqrt(ta);
    }

    public Translation2d getObjectRobotRelativeTranslation2d(LimelightHelpers.RawDetection detection) {
        if (detection == null)
            return null;

        // 1. Camera-relative
        Translation2d camRel = getObjectCameraRelativeTranslation2d(detection);
        if (camRel == null)
            return null;

        // 2. Get camera pose in robot space
        Pose3d cameraPose3d = this.cameraPose3d;
        if (cameraPose3d == null)
            return null;

        Translation2d cameraOffset = new Translation2d(cameraPose3d.getX(), cameraPose3d.getY());
        Rotation2d cameraYawOffset = new Rotation2d(cameraPose3d.getRotation().getZ());

        // 3. Rotate camera-relative detection into robot frame
        Translation2d robotRel = camRel.rotateBy(cameraYawOffset).plus(cameraOffset);

        return robotRel;
    }

    public Translation2d getObjectCameraRelativeTranslation2d(LimelightHelpers.RawDetection detection) {
        if (detection == null)
            return null;

        // Pull camera pose from LL config
        Pose3d cameraPose3d = this.cameraPose3d;
        if (cameraPose3d == null)
            return null;

        double cameraHeight = cameraPose3d.getZ();
        double pitchRad = cameraPose3d.getRotation().getY(); // camera pitch from LL mount config

        double txRad = Math.toRadians(detection.txnc);
        double tyRad = Math.toRadians(detection.tync);

        // Ground-plane distance using camera pitch + vertical offset
        // double distance = cameraHeight * Math.tan(Math.PI / 2 - pitchRad + tyRad);
        double distance = VisionConstants.tyToDistance.get(tyRad);
        double xRel = distance;
        double yRel = Math.sqrt(distance * distance + cameraPose3d.getZ() * cameraPose3d.getZ()) * Math.tan(txRad);

        return new Translation2d(xRel, yRel);// seeing forward from the cam + left/right shifting
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

        // primary object info
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

        // Get camera pose in robot space
        Pose3d cameraPose3d = this.cameraPose3d;
        // Print the camera pose to the SmartDashboard
        if (cameraPose3d != null) {
            SmartDashboard.putString("Camera Pose", cameraPose3d.toString());
        } else {
            SmartDashboard.putString("Camera Pose", "None");
        }
    }
}