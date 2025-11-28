package frc.robot.library.ideasFrom6940.turretAiming.V2;

// import edu.wpi.first.math.geometry.Pose2d;
// import frc.robot.library.ideasFrom6940.turretAiming.constants.NewFieldConstants;
// import frc.robot.library.ideasFrom6940.turretAiming.constants.TurretConstants;

public class AimCalculatorV2 {

    private static TurretAimingSystemV2 aimingSystem = new TurretAimingSystemV2();
    public static double launchVelocity = 0.;
    public static double elevationAngle = 0.;
    public static double azimuthAngle = 0.;
    public static String status = "IDLE";

    /**
     * Calculate the ballistic solution from turret to target once (with XY chassis velocity compensation)
     */
    public static void calculateOnce(double turretX, double turretY, double turretZ,
                                     double turretVx, double turretVy,
                                     double targetX, double targetY, double targetZ) {

        aimingSystem.setTarget(targetX, targetY, targetZ);
        BallisticSolution solution = aimingSystem.calculateOptimalTrajectory(
                turretX, turretY, turretZ,
                turretVx, turretVy
        );

        if (solution != null 
            && !Double.isNaN(solution.launchVelocity) 
            && !Double.isNaN(solution.elevationAngle)) {

            launchVelocity = solution.launchVelocity;
            elevationAngle = solution.elevationAngle;
            azimuthAngle = solution.azimuthAngle;
            status = "OK";

        } else {
            if (aimingSystem.isTargetTooFar()) {
                status = "TOO_FAR";
            } else if (aimingSystem.isTargetTooClose()) {
                status = "TOO_CLOSE";
            } else {
                status = "IDLE"; // shouldn't happen
            }
        }
    }

    /* ------------------------------------------------T E S T----------------------------------------------- */
    // public static void main(String[] args) {
    //     Pose2d robotPose = new Pose2d(1.0, 1.0, null); // Example robot pose
    //     double turretX = robotPose.getX() + TurretConstants.turretOffsetX;
    //     double turretY = robotPose.getY() + TurretConstants.turretOffsetY;
    //     double turretZ = TurretConstants.turretHeight;
    
    //     // Example chassis velocity (m/s)
    //     double turretVx = 2.0;
    //     double turretVy = 0.5;
    
    //     double targetX = NewFieldConstants.kTestScoringElement.getX();
    //     double targetY = NewFieldConstants.kTestScoringElement.getY();
    //     double targetZ = NewFieldConstants.kTestScoringElement.getZ();
    
    //     calculateOnce(turretX, turretY, turretZ, turretVx, turretVy, targetX, targetY, targetZ);
    
    //     System.out.printf("Status=%s, V=%.2f m/s, Elev=%.2f°, Azim=%.2f°%n",
    //             status, launchVelocity, elevationAngle, azimuthAngle);
    // }
}
