package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;

public class CameraPoseHelper {
    public static Pose3d getBotPose3d(String limelightName) {
        return LimelightHelpers.getBotPose3d(limelightName);
    }

    public CameraPoseHelper() {}

    public double getCameraX(String limelightName) {
        Pose3d botPose = getBotPose3d(limelightName);
        return botPose == null ? Double.NaN : botPose.getX();
    }

    public double getCameraY(String limelightName) {
        Pose3d botPose = getBotPose3d(limelightName);
        return botPose == null ? Double.NaN : botPose.getY();
    }

    public double getCameraZ(String limelightName) {
        Pose3d botPose = getBotPose3d(limelightName);
        return botPose == null ? Double.NaN : botPose.getZ();

    }

    public double getCameraYaw(String limelightName) {
        Pose3d botPose = getBotPose3d(limelightName);
        return botPose == null ? Double.NaN : Math.toDegrees(botPose.getRotation().getZ());

    }
    
    public double getCameraPitch(String limelightName) {
        Pose3d botPose = getBotPose3d(limelightName);
        return botPose == null ? Double.NaN : Math.toDegrees(botPose.getRotation().getY());

    }

    // public double getTargetXOffset(String limelightName, double targetX) {
    //     return getCameraX(limelightName) - targetX;
    // }

    // public double getTargetYOffset(String limelightName, double targetY) {
    //     return getCameraY(limelightName) - targetY;
    // }

    /**
     * Get the yaw angle from the camera to the target point (targetX, targetY)
     * @param limelightName
     * @param targetX
     * @param targetY
     * @return the angle of the target relative to the camera's forward direction (in degrees)
     */
    public double getTargetYawOffset(String limelightName, double targetX, double targetY) {
        double cameraX = getCameraX(limelightName);
        double cameraY = getCameraY(limelightName);

        double angle = Math.toDegrees(Math.atan2(targetY - cameraY, targetX - cameraX));
        return angle;
    }
}
