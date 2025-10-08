package frc.robot.library.ideasFrom6940.TurretAiming;

import frc.robot.library.ideasFrom6940.TurretAiming.Constants.NewFieldConstants;

public class AimingTest {

    public static void main(String[] args) {
        TurretAimingSystem aimingSystem = new TurretAimingSystem();
        
        // 设置目标坐标 //TODO
        aimingSystem.setTarget(NewFieldConstants.kLeftScoringElement_BLUE.getX(),
                               NewFieldConstants.kLeftScoringElement_BLUE.getY(),
                               NewFieldConstants.kLeftScoringElement_BLUE.getZ());
        
        double[][] testPositions = {
            {3, 0, 0},
            {5, 10, 1},
            {10, 5, 1},
            {5, 5, 3},
            {200, 0, 0},
            {NewFieldConstants.kLeftScoringElement_BLUE.getX()-0.1,NewFieldConstants.kLeftScoringElement_BLUE.getY()-0.1, 0}
        };
        
        for (double[] position : testPositions) {
            double turretX = position[0];
            double turretY = position[1];
            double turretZ = position[2];
            
            System.out.println("炮台位置: (" + turretX + ", " + turretY + ", " + turretZ + ")");
            
            BallisticSolution solution = aimingSystem.calculateOptimalTrajectory(
                turretX, turretY, turretZ);
            
            if (solution != null) {
                System.out.printf("最优解: 速度=%.1f m/s, 俯仰角=%.1f°%n", 
                    solution.launchVelocity, solution.elevationAngle);
            } else {
                if (aimingSystem.isTargetTooFar()) {
                    System.out.println("目标距离太远");

                } else if (aimingSystem.isTargetTooClose()) {
                    System.out.println("目标距离太近，请后退");
                }
            }
            System.out.println("---");
        }
    }
}
