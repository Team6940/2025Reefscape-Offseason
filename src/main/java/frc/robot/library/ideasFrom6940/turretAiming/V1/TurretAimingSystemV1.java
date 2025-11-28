package frc.robot.library.ideasFrom6940.turretAiming.V1;

import frc.robot.library.ideasFrom6940.turretAiming.constants.TurretConstants;

class BallisticSolution {
    public final double launchVelocity;  // (m/s)
    public final double elevationAngle;  // elevation angle (degrees)
    
    public BallisticSolution(double velocity, double angle) {
        this.launchVelocity = velocity;
        this.elevationAngle = angle;
    }
}

/**
 * Main Turret Aiming System. Using Binary Search to Find Optimal Ballistic Solution
 */
public class TurretAimingSystemV1 {
    private static final double G = 9.81; //TODO: CHANGE TO LOCAL GRAVITY / JUST JOKING HAHA.
    private static final double MAX_ELEVATION = TurretConstants.MAX_ELEVATION;
    private static final double MAX_VELOCITY = TurretConstants.MAX_VELOCITY;
    
    private double targetX;
    private double targetY;
    private double targetZ;

    private boolean isTargetTooFar = false;
    private boolean isTargetTooClose = false;
    
    /**
     * Sets the target position.
     */
    public void setTarget(double x, double y, double z) {
        this.targetX = x;
        this.targetY = y;
        this.targetZ = z;
    }
    
    /**
     * Calculate horizontal distance and height difference from turret to target.
     * 
     * @return double[] {horizontal distance d, height difference h}
     */
    private double[] calculateTargetParameters(double turretX, double turretY, double turretZ) {
        double dx = targetX - turretX;
        double dy = targetY - turretY;
        double dz = targetZ - turretZ;
        
        double horizontalDistance = Math.sqrt(dx * dx + dy * dy);  // Horizontal distance in XY plane
        double heightDifference = dz;  // Height difference in Z direction
        
        return new double[]{horizontalDistance, heightDifference};
    }

    /**
     * Calculate minimum required velocity
     * @return minimum velocity (m/s) OR Double.MAX_VALUE if no solution
     */
    private double calculateMinimumVelocity(double d, double h) {
        double denominator = -h + Math.sqrt(h * h + d * d);
        if (denominator <= 0) {
            return Double.MAX_VALUE; // no solution
        }
        return d * Math.sqrt(G / denominator);
    }
    
    /**
     * Calculate high-trajectory elevation angle and flight time for given velocity
     */
    private double[] calculateTrajectory(double v, double d, double h) {
        double A = (G * d * d) / (2 * v * v);
        double discriminant = d * d - 4 * A * (h + A);
        
        if (discriminant < 0) {
            return null;
        }
        
        double u = (d + Math.sqrt(discriminant)) / (2 * A);  // TODO: selected high-trajectory solution, change base on needs
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
     * Use binary search to find the optimal solution.
     */
    private BallisticSolution binarySearchOptimalSolution(double d, double h, double vMin, double vMax) {
        int left = (int) Math.ceil(vMin);
        int right = (int) Math.floor(vMax);
        
        double bestVelocity = -1;
        double bestAngle = -1;
        double minTime = Double.MAX_VALUE;
        
        while (left <= right) {
            int mid = (left + right) / 2;
            double[] result = calculateTrajectory(mid, d, h);
            
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
    
    public BallisticSolution calculateOptimalTrajectory(double turretX, double turretY, double turretZ) {
        isTargetTooFar = false;
        isTargetTooClose = false;
    
        double[] params = calculateTargetParameters(turretX, turretY, turretZ);
        double d = params[0];
        double h = params[1];
    
        // Special case: target is directly above or coincident : return too close
        if (d < 1e-6 && h >= 0) {
            isTargetTooClose = true;
            // return null;
            return new BallisticSolution(Double.NaN, Double.NaN);
        }
    
        double vMin = calculateMinimumVelocity(d, h);
    
        // Case 1: insufficient velocity : this means too far
        if (vMin == Double.MAX_VALUE || vMin > MAX_VELOCITY) {
            isTargetTooFar = true;
            // return null;
            return new BallisticSolution(Double.NaN, Double.NaN);
        }
    
        // Case 2: sufficient velocity but no angle/geometry solution : this means too close
        BallisticSolution solution = binarySearchOptimalSolution(d, h, vMin, MAX_VELOCITY);
        if (solution == null) {
            isTargetTooClose = true;
            return new BallisticSolution(Double.NaN, Double.NaN);
        }
        return solution;
    }
    
    public boolean isTargetTooFar() { return isTargetTooFar; }
    public boolean isTargetTooClose() { return isTargetTooClose; }

}
