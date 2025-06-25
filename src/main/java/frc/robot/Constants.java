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

    public static class IntakerConstants {
        public static final int IntakerMotorID = 16;

        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.22;
        public static final double kS = 0.15;

        public static final InvertedValue IntakerInverted = InvertedValue.Clockwise_Positive;
        public static final double IntakerRatio = 50. / 24.; // LCY: 50. :24. // GY: 20. : 10.

        public static final double IntakerVelocityToleranceRPS = 0;

        public static double intakingRPS;

    }

    public static class IndexerConstants {
        public static final int IndexerLeftMotorID = 16;
        public static final int IndexerRghtMotorID = 16;

        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.22;
        public static final double kS = 0.15;

        public static final InvertedValue LeftInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue RghtInverted = InvertedValue.CounterClockwise_Positive;
        public static final double IndexerRatio = 50. / 24.; // LCY: 50. :24. // GY: 20. : 10.

        public static final double IndexerVelocityToleranceRPS = 0;
        public static final double IntakingRPS = 0;
        public static final double IndexerAligningCurrentThreshold = 0;
        public static final double IndexerFreeSpinCurrentThreshold = 0;

    }

    public static final class ShooterConstants {
        public static final int ShooterMotorID = 17;

        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.22;
        public static final double kS = 0.15;

        public static final double ShooterShootRPSs[] = new double[] {
                0., //These four are for coral scoring
                15.5,
                12.5,
                12.5,
                18.,
                114514. //This is for the algae scoring
        };

        public static final InvertedValue ShooterInverted = InvertedValue.Clockwise_Positive;
        public static final double ShooterRatio = 50. / 24.; // LCY: 50. :24. // GY: 20. : 10.

        public static final double ShooterSpeedTolerence = 0;

        public static final double shooterScoringRPS = 0;

        public static final double IntakingRPS = 0;

        public static final double ShooterFreeSpinCurrentThreshold = 0;

        public static final double ShooterReadyCurrentThreshold = 0; //FREE SPINNING

        public static final double ShooterGrabbingCurrentThreshold = 0; //GOT HOLD OF STUFF //TODO: should add 2 thresholds for the coral and the algae

        public static final double AlgaeIntakingRPS = 0;

    }

    public static final class ArmConstants {
        public static final int ArmMotorID = 18;
        public static final double ArmVelocityToleranceRPS = 0.2;
        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.22;
        public static final double kS = 0.15;
        public static final double kG = 0.0; // gravity
        public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
        public static final double ArmRatio = 50. / 24.; // LCY: 50. :24. //GY: 20. : 10.
        public static final double MaxVelocity = 0.5; // RPS
        public static final double Acceleration = 0.5; // RPS^2

        public static final double MinRadians = Units.degreesToRadians(-90.); // -90 degrees
        public static final double MaxRadians = Units.degreesToRadians(90.); // 90 degrees
        public static final double ArmPositionToleranceRadians = 0;
    }

    public static final class GrArmConstants {
        public static final int GrArmMotorID = 21;
        public static final double GrArmVelocityToleranceRPS = 0.2;
        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.22;
        public static final double kS = 0.15;
        public static final double kG = 0.0; // gravity
        public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
        public static final double GrArmRatio = 50. / 24.; // LCY: 50. :24. //GY: 20. : 10.
        public static final double MaxVelocity = 0.5; // RPS
        public static final double Acceleration = 0.5; // RPS^2

        public static final double MinRadians = Units.degreesToRadians(-90.); // -90 degrees
        public static final double MaxRadians = Units.degreesToRadians(90.); // 90 degrees
        public static final double GrArmPositionToleranceRadians = 0;
        public static double retractedPosition = 0.;
        public static double extendedPosition = 0;
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

    public static final class AutoConstants {
        // Path Following
        public static final double followPathTranslationkP = 3.; // TODO
        public static final double followPathTranslationkI = 0.; // TODO
        public static final double followPathTranslationkD = 0.; // TODO

        public static final double followPathRotationkP = 3.; // TODO
        public static final double followPathRotationkI = 0.; // TODO
        public static final double followPathRotationkD = 0.; // TODO

        // Move To Pose
        public static final double moveToPoseTranslationkP = 5.; // TODO
        public static final double moveToPoseTranslationkI = 0.; // TODO
        public static final double moveToPoseTranslationkD = 0.; // TODO

        public static final double moveToPoseRotationkP = 5.; // TODO
        public static final double moveToPoseRotationkI = 0.; // TODO
        public static final double moveToPoseRotationkD = 0.; // TODO

        public static final double moveToPoseRotationToleranceRadians = Units.degreesToRadians(3.);
        public static final double moveToPoseTranslationToleranceMeters = 0.02;

        public static final double maxMoveToSpeed = 3.8;
        public static final double maxMoveToAngularVelocity = Units.degreesToRadians(230.);
        public static final double IntakePredictionTimeSecs = 0.1;
    }

    public static final class ManualConstants {
        // public static final double maxSpeed = 0.2; //
    }

    public static final class ElevatorConstants {
        public static final double ElevatorHeightTolerence = 0.05;
        public static final double ElevatorVelocityTolerence = 0.2;

        public static final int leftMotorID = 15;
        public static final int rghtMotorID = 14;

        public static final double kP = 10;
        public static final double kI = 0;
        public static final double kD = 0.4;
        public static final double kS = 0.2;
        public static final double kV = 0.06;
        public static final double kG = 0.8;
        public static final double Acceleration = 130.; // 102.
        public static final double MaxVelocity = 32.5; // 32.5
        public static final double MaxHeight = 1.39;

        public static final double MotorToRollerRatio = 3;

        public static final InvertedValue LeftInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue RghtInverted = InvertedValue.CounterClockwise_Positive;

        public static final double RollerRoundToMeters = 0.04 * Math.PI;
        public static final double IntakingHeight = 0;

    }

    public static final class ClimberConstants {

        public static final InvertedValue ClimberInverted = InvertedValue.CounterClockwise_Positive;

        public static final int ClimberMotorID = 17;
        public static final double ClimberkP = 14.;
        public static final double ClimberkI = 0.;
        public static final double ClimberkD = 0.;
        public static final double ClimberkS = 0.1;
        public static final double ClimberRatio = 45.;
        public static final double ClimberkG = 0;
        public static final double ClimberkV = 0;
        public static final double Acceleration = 1.8;
        public static final double MaxVelocity = 1.8;

        public static final double ClimberRotationTolerence = 0.05;

        public static final double ClimberDefaultPos = 0.;
        public static final double ClimberMaxPos = 2.82;
        public static final double ClimberMinPos = -0.875;
        public static final double ClimberExtensionPos = 2.74;
        public static final double ClimberRetractionPos = -0.87;

    }

    public static final class FieldConstants {

        public static final Translation2d FieldCenter = new Translation2d(17.548225 / 2, 8.0518 / 2.);

        public static final Translation2d BlueReefCenterPos = new Translation2d(4.489323, 8.0518 / 2.);
        public static final Translation2d DReefTranslation12 = new Translation2d(1.31, 0.161);
        public static final Translation2d DAlgaeTranslation6 = new Translation2d(1.4,0);//todo

        public static final double reefTranslationAdjustmentRange = 0.15;
        public static final double reefRotationAdjustmentRangeDegs = 20;


        public static final Translation2d BlueRightStationCenterPos = new Translation2d(1.2, 1.05);
        public static final Translation2d DStationTranslationRSL = new Translation2d(-0.675, 0.42);
        public static final Rotation2d DStationRotationRSL = Rotation2d.fromRadians(0.935);

        public static final double armIntakePosition[]={
            0,
            0
        };

        public static double algaeAlignmentDistanceThreshold;

        public static int algaeIntakePushDistance;

        public static double armAlgaeStowPosition;

        public static double pushDistance;

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

        public static final double StationDetectionArea = 0;

        public static final int DChuteX = 0;

        public static final int BlueFirstChuteTranslationX = 0;

        public static final double BargeHeight = 114514.; //TODO: this is a placeholder value, should be replaced with the actual barge height

        public static final double BargeAngle = 1919810.; //TODO: this is a placeholder value, should be replaced with the actual barge angle
        public static final double elevatorAlgaeIntakeHeight[]={
            0,
            0
        };

        public static final double elevatorHeights[] = {
                0.,
                0.53,
                0.475,
                0.82,
                1.39
        };

        public static final double armAngles[] = {
                0.,
                0.53,
                0.475,
                0.82,
                1.39// these data needs to be tuned
        };

        public static final double reefRotationAdjustmentRange[] = {
                0,
                0,
                0,
                0,
                0
        };

        public static final double coralScorePushDistance = 0.2; // meters, this is the distance the robot will push forward after
                                                       // aligning to the reef pose
        public static final double departDistance = 0.1; // meters, this is the distance the robot will move forward
                                                         // after scoring


        //THESE ARE FOR THE CLIMBING  //COPIED FROM 2025 REEFSCAPE
        public static final double ClimbPushDis = 0.;                             //0.48
        public static final double ClimbRetreatToDis = 0.;        //0.272         //0.40
                                                 
        public static final double elevatorClimbHeight = 0.29;
                                                 
        public static final Pose2d BlueClimbPoses[] = {
            new Pose2d(8.3, 7.15, Rotation2d.k180deg),
            new Pose2d(0, 0, Rotation2d.k180deg),
            new Pose2d(0, 0, Rotation2d.k180deg)
        };
    }

    public static void initializeConstants() {
        // for (var p : PoseEstimatorConstants.tAtoDevPoints)
        // PoseEstimatorConstants.tAtoDev.put(p.getX(), p.getY());
    }
}
