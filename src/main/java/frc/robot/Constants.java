package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Library.MUtils.SegmentOnTheField;

import java.awt.geom.Point2D;

public class Constants {

    public static class GlobalConstants {
        public final static float INF = (float) Math.pow(10, 6); // this was defined for the 1690 lib
    }

    public static class DriveConstants {
        public final static double kInnerDeadband = 0.004;
        public final static double kOuterDeadband = 0.98; // these were defined for the 1706 lib;
        public static final int IntakeButton = 0;
        public static final double defaultDrivePower = 1.2;

    }

    public static final class ShooterConstants {
        public static final int LeftMotorID = 16;
        public static final int RghtMotorID = 17;
        
        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.22;
        public static final double kS = 0.15;

        public static final InvertedValue LeftInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue RghtInverted = InvertedValue.Clockwise_Positive;
        public static final double ShooterRatio = 50. / 24.; // LCY: 50. :24. // GY: 20. : 10.
    }

    public static final class IntakerConstants {
        public static final int LeftMotorID = 20;
        public static final int RghtMotorID = 21;

        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.22;
        public static final double kS = 0.15;

        public static final InvertedValue LeftInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue RghtInverted = InvertedValue.Clockwise_Positive;

        public static final double IntakerRatio = 50. / 24.; // LCY: 50. :24. // GY: 20. : 10.
        public static final double IntakerVelocityToleranceRPS = 0.2; // LCY: 0.2 // GY: 0.1
    }

    public static final class ArmConstants {
        public static final int ArmMotorID = 18;
        public static final double ArmVelocityToleranceRPS = 0.2;
        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.22;
        public static final double kS = 0.15;
        public static final double kG = 0.0; //gravity
        public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
        public static final double ArmRatio = 50./24.;      //LCY: 50. :24.      //GY: 20. : 10.
        public static final double MaxVelocity = 0.5; // RPS
        public static final double Acceleration = 0.5; // RPS^2

        public static final double MinRadians = Units.degreesToRadians(-90.); // -90 degrees
        public static final double MaxRadians = Units.degreesToRadians(90.); // 90 degrees
    }

    
    public static final class IndexerConstants {
        public static final int MotorID = 19;
        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.22;
        public static final double kS = 0.15;
        public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
        public static final double IntakerRatio = 50. / 24.; 

    }


    public static final class PoseEstimatorConstants {
        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.3, 0.3, 0.1);
        public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
        public static final InterpolatingDoubleTreeMap tAtoDev = new InterpolatingDoubleTreeMap();

        public static final Point2D[] tAtoDevPoints = {
                // new Point2D(0.374, 0.003),
                // new Point2D(0.071, 0.2),
                // new Point2D(0.046, 0.4)
                new Point2D.Double(0.17, 0.08),
                new Point2D.Double(0.12, 0.20),
                new Point2D.Double(0.071, 0.35),
                new Point2D.Double(0.046, 0.4),
        };

    }

    // public static final class AutoConstants {
    //     // Path Following
    //     public static final double followPathTranslationkP = 5.; // TODO
    //     public static final double followPathTranslationkI = 0.; // TODO
    //     public static final double followPathTranslationkD = 0.; // TODO

    //     public static final double followPathRotationkP = 5.; // TODO
    //     public static final double followPathRotationkI = 0.; // TODO
    //     public static final double followPathRotationkD = 0.; // TODO

    //     // Move To Pose
    //     public static final double moveToPoseTranslationkP = 5.; // TODO
    //     public static final double moveToPoseTranslationkI = 0.; // TODO
    //     public static final double moveToPoseTranslationkD = 0.; // TODO

    //     public static final double moveToPoseRotationkP = 5.; // TODO
    //     public static final double moveToPoseRotationkI = 0.; // TODO
    //     public static final double moveToPoseRotationkD = 0.; // TODO

    //     public static final double moveToPoseRotationToleranceRadians = Units.degreesToRadians(3.);
    //     public static final double moveToPoseTranslationToleranceMeters = 0.02;

    //     public static final double maxMoveToSpeed = 3.8;
    //     public static final double maxMoveToAngularVelocity = Units.degreesToRadians(230.);
    //     public static final double IntakePredictionTimeSecs = 0.1;
    // }

    public static final class ManualConstants {
        // public static final double maxSpeed = 0.2; //
    }

    public static final class FieldConstants {

        public static final Translation2d FieldCenter = new Translation2d(17.548225 / 2, 8.0518 / 2.);

        public static final Translation2d BlueReefCenterPos = new Translation2d(4.489323, 8.0518 / 2.);
        public static final Translation2d DReefTranslation12 = new Translation2d(1.31, 0.161);
                                                                                               // Translation2d(1.32,
                                                                                               // 0.162)

        public static final double reefTranslationAdjustmentRange = 0.15;
        public static final double reefRotationAdjustmentRangeDegs = 20;

        public static final Translation2d BlueRightStationCenterPos = new Translation2d(1.2, 1.05);
        public static final Translation2d DStationTranslationRSL = new Translation2d(-0.675, 0.42);
        public static final Rotation2d DStationRotationRSL = Rotation2d.fromRadians(0.935);

        public static Pose2d rotateAroundCenter(Pose2d pose, Translation2d centre, Rotation2d rotation) {
            return new Pose2d(pose.getTranslation().rotateAround(centre, rotation), pose.getRotation().plus(rotation));
        }

        public static final Translation2d BlueRghtStationStPos = new Translation2d(0.51, 1.2);
        public static final Translation2d BlueRghtStationEdPos = new Translation2d(1.51, 0.43);

        public static final Translation2d BlueLeftStationStPos = new Translation2d(BlueRghtStationStPos.getX(),
                FieldCenter.getY() * 2 - BlueRghtStationStPos.getY());
        public static final Translation2d BlueLeftStationEdPos = new Translation2d(BlueRghtStationEdPos.getX(),
                FieldCenter.getY() * 2 - BlueRghtStationEdPos.getY());

        public static final SegmentOnTheField BlueRghtStation = new SegmentOnTheField(BlueRghtStationStPos,
                BlueRghtStationEdPos);
        public static final SegmentOnTheField BlueLeftStation = new SegmentOnTheField(BlueLeftStationStPos,
                BlueLeftStationEdPos);
    }

    public static void initializeConstants() {
        // for (var p : PoseEstimatorConstants.tAtoDevPoints)
        //     PoseEstimatorConstants.tAtoDev.put(p.getX(), p.getY());
    }
}
