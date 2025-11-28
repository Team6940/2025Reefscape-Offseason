package frc.robot.library.ideasFrom6940.turretAiming.V1;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.library.ideasFrom6940.turretAiming.constants.NewFieldConstants;
import frc.robot.library.ideasFrom6940.turretAiming.constants.TurretConstants;

public class AimCalculatorV1{

    private static TurretAimingSystemV1 aimingSystem = new TurretAimingSystemV1();
    public static double launchVelocity = 0.;
    public static double elevationAngle = 0.;
    public static String status = "IDLE";
        /**
         * Calculate the ballistic solution from turret to target once
         */
        public static void calculateOnce(double turretX, double turretY, double turretZ,
                                         double targetX, double targetY, double targetZ) {
    
            aimingSystem.setTarget(targetX, targetY, targetZ);
            BallisticSolution solution = aimingSystem.calculateOptimalTrajectory(turretX, turretY, turretZ);
    
            // //TODO: Console output (for debugging)
            // System.out.println("turret pos: (" + turretX + ", " + turretY + ", " + turretZ + ")");
            // System.out.println("target pos: (" + targetX + ", " + targetY + ", " + targetZ + ")");
    
            if (solution != null && !Double.isNaN(solution.launchVelocity) && !Double.isNaN(solution.elevationAngle)) {
                // System.out.printf("best solution: VELO=%.1f m/s, PITCH=%.1f°%n",
                //         solution.launchVelocity, solution.elevationAngle);
                launchVelocity = solution.launchVelocity;
                elevationAngle = solution.elevationAngle;
                status = "OK";
    
            } else {
                if (aimingSystem.isTargetTooFar()) {
                    status = "TOO_FAR";
                // System.out.println("WARNING: TARGET TOO FAR");
            } else if (aimingSystem.isTargetTooClose()) {
                status = "TOO_CLOSE";
                // System.out.println("WARNING: TARGET TOO CLOSE");
            } else {
                status = "IDLE"; //WARNING: THIS SHOULDN'T HAPPEN
                // System.out.println("IDLE"); 
            }
        }

    }
    /* ------------------------------------------------T E S T----------------------------------------------- */
    // public static void main(String[] args) {
    //     Pose2d robotPose = new Pose2d(5.0, 3.0, null); // Example robot pose
    //     double turretX = robotPose.getX() + TurretConstants.turretOffsetX;
    //     double turretY = robotPose.getY() + TurretConstants.turretOffsetY;
    //     double turretZ = TurretConstants.turretHeight;

    //     double targetX = NewFieldConstants.kLeftScoringElement_BLUE.getX();
    //     double targetY = NewFieldConstants.kLeftScoringElement_BLUE.getY();
    //     double targetZ = NewFieldConstants.kLeftScoringElement_BLUE.getZ();

    //     calculateOnce(turretX, turretY, turretZ, targetX, targetY, targetZ);
    // }
}
