package frc.robot.library.ideasFrom6940.TurretAiming;

import frc.robot.library.ideasFrom6940.TurretAiming.Constants.TurretConstants;

/**
 * 弹道解算结果类
 */
class BallisticSolution {
    public final double launchVelocity;  // 出射速度 (m/s)
    public final double elevationAngle;  // 俯仰角 (度)
    
    public BallisticSolution(double velocity, double angle) {
        this.launchVelocity = velocity;
        this.elevationAngle = angle;
    }
}

/**
 * 炮台自动瞄准系统
 * 使用二分法寻找最优弹道解
 */
public class TurretAimingSystem {
    // 物理常量
    private static final double G = 9.8;       // 重力加速度 (m/s²)
    private static final double MAX_ELEVATION = TurretConstants.MAX_ELEVATION;  // 最大俯仰角 (度)
    private static final double MAX_VELOCITY = TurretConstants.MAX_VELOCITY;   // 固定最大出射速度 (m/s)
    
    // 目标坐标（由外部输入）
    private double targetX;
    private double targetY;
    private double targetZ;
    
    // 解算状态标志
    private boolean isTargetTooFar = false;
    private boolean isTargetTooClose = false;
    
    /**
     * 设置目标坐标
     */
    public void setTarget(double x, double y, double z) {
        this.targetX = x;
        this.targetY = y;
        this.targetZ = z;
    }
    
    /**
     * 计算炮台到目标的水平距离和高度差
     */
    private double[] calculateTargetParameters(double turretX, double turretY, double turretZ) {
        double dx = targetX - turretX;
        double dy = targetY - turretY;
        double dz = targetZ - turretZ;
        
        double horizontalDistance = Math.sqrt(dx * dx + dy * dy);  // XY平面距离
        double heightDifference = dz;  // Z方向高度差
        
        return new double[]{horizontalDistance, heightDifference};
    }

    /**
     * 计算最小所需速度
     */
    private double calculateMinimumVelocity(double d, double h) {
        double denominator = -h + Math.sqrt(h * h + d * d);
        if (denominator <= 0) {
            return Double.MAX_VALUE; // 无解
        }
        return d * Math.sqrt(G / denominator);
    }
    
    /**
     * 计算给定速度下的高抛弹道俯仰角和飞行时间
     */
    private double[] calculateHighTrajectory(double v, double d, double h) {
        double A = (G * d * d) / (2 * v * v);
        double discriminant = d * d - 4 * A * (h + A);
        
        if (discriminant < 0) {
            return null;
        }
        
        double u = (d + Math.sqrt(discriminant)) / (2 * A);  // 高抛解
        double thetaRad = Math.atan(u);
        double thetaDeg = Math.toDegrees(thetaRad);
        
        if (thetaDeg < 0 || thetaDeg > MAX_ELEVATION) {
            return null;
        }
        
        double cosTheta = Math.cos(thetaRad);
        double flightTime = d / (v * cosTheta);
        
        return new double[]{thetaDeg, flightTime};
    }
    
    /**
     * 使用二分法寻找最优解
     */
    private BallisticSolution binarySearchOptimalSolution(double d, double h, double vMin, double vMax) {
        int left = (int) Math.ceil(vMin);
        int right = (int) Math.floor(vMax);
        
        double bestVelocity = -1;
        double bestAngle = -1;
        double minTime = Double.MAX_VALUE;
        
        while (left <= right) {
            int mid = (left + right) / 2;
            double[] result = calculateHighTrajectory(mid, d, h);
            
            if (result != null) {
                double flightTime = result[1];
                if (flightTime < minTime) {
                    minTime = flightTime;
                    bestVelocity = mid;
                    bestAngle = result[0];
                }
                right = mid - 1;
            } else {
                left = mid + 1;
            }
        }
        
        if (bestVelocity != -1) {
            return new BallisticSolution(bestVelocity, bestAngle);
        }
        return null;
    }
    
    /**
     * 主解算函数 - 计算最优弹道解
     * 区分“太远”和“太近”
     */
    // public BallisticSolution calculateOptimalTrajectory(double turretX, double turretY, double turretZ) {
    //     isTargetTooFar = false;
    //     isTargetTooClose = false;
        
    //     double[] params = calculateTargetParameters(turretX, turretY, turretZ);
    //     double d = params[0];
    //     double h = params[1];
        
    //     double vMin = calculateMinimumVelocity(d, h);

    //     // 情况1：速度不足 → 太远
    //     if (vMin == Double.MAX_VALUE || vMin > MAX_VELOCITY) {
    //         isTargetTooFar = true;
    //         return null;
    //     }

    //     // 情况2：速度足够，但角度/几何无解 → 太近
    //     BallisticSolution solution = binarySearchOptimalSolution(d, h, vMin, MAX_VELOCITY);
    //     if (solution == null) {
    //         isTargetTooClose = true;
    //     }
    //     return solution;
    // }
    public BallisticSolution calculateOptimalTrajectory(double turretX, double turretY, double turretZ) {
        isTargetTooFar = false;
        isTargetTooClose = false;
    
        double[] params = calculateTargetParameters(turretX, turretY, turretZ);
        double d = params[0];
        double h = params[1];
    
        // 特殊情况：目标在正上方或重合 → 太近
        if (d < 1e-6 && h >= 0) {
            isTargetTooClose = true;
            return null;
        }
    
        double vMin = calculateMinimumVelocity(d, h);
    
        // 情况1：速度不足 → 太远
        if (vMin == Double.MAX_VALUE || vMin > MAX_VELOCITY) {
            isTargetTooFar = true;
            return null;
        }
    
        // 情况2：速度足够，但角度/几何无解 → 太近
        BallisticSolution solution = binarySearchOptimalSolution(d, h, vMin, MAX_VELOCITY);
        if (solution == null) {
            isTargetTooClose = true;
        }
        return solution;
    }
    
    public boolean isTargetTooFar() { return isTargetTooFar; }
    public boolean isTargetTooClose() { return isTargetTooClose; }

}
