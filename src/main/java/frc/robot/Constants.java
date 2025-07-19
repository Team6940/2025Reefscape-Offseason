package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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
    public enum UpperStructureState {
        //prepare->aim; score->score
        ScoreL1(0.05, 0.99),
        ScoreL2(-0.28, 0.99),
        ScoreL3(-0.05, -333.),
        ScoreL4(0.6, -350.),
        PrepareScoreL1(0.05, 0.99),
        PrepareScoreL2(-0.28, 0.99),
        PrepareScoreL3(-0.05, -290),
        PrepareScoreL4(0.6, -290),
        RScoreL1(0.15, -135.),
        RScoreL2(-0.28, -210),
        RScoreL3(-0.05, -207),
        RScoreL4(0.6, -190),
        RPrepareScoreL1(0.2, -150.),
        RPrepareScoreL2(-0.28, -250),
        RPrepareScoreL3(-0.05, -250),
        RPrepareScoreL4(0.6, -250),
        IdleDown(0,0),
        
        IdleGrab(0,0),
        IdleUp(0,0);
  
        public final double elevatorHeightMeters;
        public final double armAngleDegs;


        /**
         * @param elevator_height in meters
         * @param arm_theta in degrees
         */
        UpperStructureState(double elevator_height, double arm_theta) {
          this.elevatorHeightMeters = elevator_height;
          this.armAngleDegs = arm_theta;
        }
    }
    public static class DriveConstants {
        public final static double kInnerDeadband = 0.004;
        public final static double kOuterDeadband = 0.98; // these were defined for the 1706 lib;
        public static final int IntakeButton = 0;
        public static final double defaultDrivePower = 1.2;

    }

    public static class IntakerConstants {
        public static final int IntakerMotorID = 20;

        public static final double kP = 0.3;
        public static final double kI = 0.;
        public static final double kD = 0.;
        public static final double kV = 0.55;
        public static final double kS = 0.227;

        public static final InvertedValue IntakerInverted = InvertedValue.Clockwise_Positive;
        public static final double IntakerRatio = 50./12.; // LCY: 50. :24. // GY: 20. : 10.

        public static final double IntakerVelocityToleranceRPS = 0.5;

        public static final double IntakerHoldingCurrentThreshold = 5.;
        public static final double IntakerFreeSpinCurrentThreshold = 5.;

        public static final double IntakerIntakingRPS= 20.; //TODO: this is a placeholder value, should be replaced with the actual intake rps

        public static final double ReversingRPS = 0;

    }

    public static class IndexerConstants {
        public static final int IndexerLeftMotorID = 22;
        public static final int IndexerRghtMotorID = 21;
        public static final int SensorID = 9;
        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0.;
        public static final double kV = 0.23;
        public static final double kS = 0.6;

        public static final InvertedValue LeftInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue RghtInverted = InvertedValue.CounterClockwise_Positive;
        public static final boolean SensorInverted = true; //TODO: this is a placeholder value, should be replaced with the actual sensor inverted value
        public static final double IndexerRatio = 50. / 24.; // LCY: 50. :24. // GY: 20. : 10.

        public static final double IndexerVelocityToleranceRPS = 0.5;
        public static final double IntakingRPS = 2.;
        public static final double IndexerAligningCurrentThreshold = 5.;
        public static final double IndexerFreeSpinCurrentThreshold = 5.;
        public static final double ReversingRPS = 0;

    }

    public static final class ShooterConstants {
        public static final int ShooterMotorID = 16;

        public static final double kP = 0.3;
        public static final double kI = 0.;
        public static final double kD = 0.;
        public static final double kV = 0.215;
        public static final double kS = 0.4;

        public static final double ShooterShootRPSs[] = new double[] {
                0., //These four are for coral scoring
                15.5,
                12.5,
                12.5,
                18.,
                20. //This is for the algae scoring
        };

        public static final InvertedValue ShooterInverted = InvertedValue.Clockwise_Positive;
        public static final double ShooterRatio = 50. / 24.; // LCY: 50. :24. // GY: 20. : 10.

        public static final double ShooterSpeedTolerence = 0.5;

        public static final double CoralIntakingRPS = -60.;

        public static final double CoralScoringRPS = 20.;

        public static final double AlgaeIntakingRPS = -120.;

        public static final double AlgaeScoringRPS = 19.;

        public static final double HoldingCoralRPS = 5.;

        public static final double HoldingAlgaeRPS = 5.; //TODO: this is a placeholder value, should be replaced with the actual intake rps

        public static final double ShooterIntakeCurrentThreshold = 35;//TODO

        public static final double DroppingRPS = 0;

        public static final int CurrentFilterTaps = 10;

        public static final double ShooterDebouncerTime = 0.1;

        public static final double SupplyCurrentLimit = 50.;


    }

    public static final class ArmConstants {
        public static final int ArmMotorID = 17;
        public static final double ArmVelocityToleranceRPS = 0.2;
        public static final double kP = 78.;
        public static final double kI = 0.;
        public static final double kD = 0.;
        public static final double kV = 0.;
        public static final double kS = 0.32;
        public static final double kG = 0.66; // gravity
        public static final InvertedValue Inverted = InvertedValue.CounterClockwise_Positive;
        public static final double MaxVelocity = 1.6; // RPS
        public static final double Acceleration = 1.3; // RPS^2

        public static final double MinDegs = -270.; // CCW Positive
        public static final double MaxDegs = -90.; 
        public static final double ArmPositionToleranceDegs = 5.;
        public static final double DroppingPositionDegs = 0.;

        public static final int ArmEncoderID = 18;
        public static final double EncoderOffsetDegrees = 1.3;
        public static final SensorDirectionValue EncoderDirection = SensorDirectionValue.Clockwise_Positive; //TODO
        public static final double SecuredPosition = -150.;
        public static final double encoderToMechanismRatio=1.;
    }

    public static final class GrArmConstants {
        public static final int GrArmMotorID = 19;
        public static final double GrArmVelocityToleranceRPS = 0.2;

        public static final double kP = 75.;
        public static final double kI = 0.;
        public static final double kD = 0.;
        public static final double kV = 0.;
        public static final double kS = 0.31;
        public static final double kG = 0.19; // gravity
        public static final InvertedValue Inverted = InvertedValue.CounterClockwise_Positive;
        public static final double GrArmRatio = 4606./65.; // LCY: 50. :24. //GY: 20. : 10.////--->(8. / 56. * 20. / 56.) ^ (-1)
        public static final double MaxVelocity = 4.; // RPS
        public static final double Acceleration = 3.; // RPS^2



        public static final double MinDegs = -66.; //degrees CCW Positive

        public static final double MaxDegs = 90.; 
        public static final double GrArmPositionToleranceDegs = 3.;

        public static final double RetractedPosition = 90.;
        public static final double ExtendedPosition = -66.;
        //public static final double Deadband = 0.24;
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
        public static final double moveToPoseTranslationToleranceMeters = 0.04;

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
        public static final double Acceleration = 20.; // 102.
        public static final double MaxVelocity = 15.; // 32.5
        public static final double MaxHeight = 1.;
        public static final double MinHeight = -0.28;

        public static final double MotorToRollerRatio = 3;

        public static final InvertedValue LeftInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue RghtInverted = InvertedValue.CounterClockwise_Positive;

        public static final double RollerRoundToMeters = 0.04 * Math.PI;
        public static final double IntakingHeight = 0.1;
        public static final double DroppingHeight = 0.;
        public static final double IdleHeight = 0.12;//TODO
        public static final double GrabbingHeight = 0.01;

    }

    public static final class ClimberConstants {

        public static final InvertedValue ClimberInverted = InvertedValue.CounterClockwise_Positive;
        //public static final InvertedValue LockMotorInverted = InvertedValue.CounterClockwise_Positive;

        public static final int ClimberliftMotorID = 23; //TODO
        // public static final int ClimberlockMotorID = 0; 

        public static final double ClimberkP = 14.;
        public static final double ClimberkI = 0.;
        public static final double ClimberkD = 0.;
        public static final double ClimberkS = 0.1;
        public static final double ClimberRatio = 45.;
        public static final double ClimberkG = 0;
        public static final double ClimberkV = 0;
        public static final double ClimberAcceleration = 1.8;
        public static final double ClimberMaxVelocity = 1.8;

        public static final double ClimberRotationTolerence = 0.05;

        public static final double ClimberDefaultPos = 0.;
        public static final double ClimberMaxPos = 2.82;
        public static final double ClimberMinPos = -0.875;
        public static final double ClimberExtensionPos = 2.64;
        public static final double ClimberRetractionPos = -0.96;//tuning needed here


        //public static final double LockMotorRPS = 1.; //TODO: this is a placeholder value, should be replaced with the actual intake rps
        //public static final double LockMotorCurrentThreshold = 0.;

    }

    public static final class FieldConstants {

        public static final Translation2d FieldCenter = new Translation2d(17.548225 / 2, 8.0518 / 2.);
        public static final double L2Fix=0.07;
        public static final Translation2d BlueReefCenterPos = new Translation2d(4.489323, 8.0518 / 2.);
        public static final Translation2d DReefTranslation12 = new Translation2d(2.3, 0.135);
        public static final Translation2d DReefTranslation12Reversed = new Translation2d(1.31, 0.161);
        public static final Translation2d DAlgaeTranslation6 = new Translation2d(1.4,0);//todo
        public static final double CoralScoreRetreatDistance = 0.3; // meters, this is the distance the robot will push
                                                                    // forward after
        // aligning to the reef pose

        public static final double reefTranslationAdjustmentRange = 0.2;
        public static final double reefRotationAdjustmentRangeDegs = 20;


        public static final Translation2d BlueRightStationCenterPos = new Translation2d(1.2, 1.05);
        public static final Translation2d DStationTranslationRSL = new Translation2d(-0.675, 0.42);
        public static final Rotation2d DStationRotationRSL = Rotation2d.fromRadians(0.935);

        public static final double ArmIntakePosition[]={
            -170.,
            -170.,
            -170.,
            -170.,
            -170.,
            -170.,
        };

        public static final double AlgaeAlignmentDistanceThreshold=1.5;

        public static final double AlgaeIntakePushDistance=0.3;

        public static final double ArmStowPosition=-250.;//TODO

        public static final double PushDistance=1.;
        
        public static final double AlgaeScorePushDistance = 0.2; // meters, this is the distance the robot will push forward after aligning to the algae pose

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

        public static final double BargeHeight = 3.; //TODO: this is a placeholder value, should be replaced with the actual barge height

        public static final double BargeAngle = 4.; //TODO: this is a placeholder value, should be replaced with the actual barge angle
        
        public static final double ElevatorAlgaeIntakeHeight[]={ //TODO tuning
            0.13,
            0.45,
            0.13,
            0.45,
            0.13,
            0.45,
        };

        public static final double ElevatorHeights[] = {
                0.,
                0.53,
                0.475,
                0.82,
                1.39
        };

        public static final double ArmAngles[] = {
                0.,
                0.53,
                0.475,
                0.82,
                1.39// these data needs to be tuned
        };

        public static final double ArmAnglesReversed[] = {
            0.,
            0.53,
            0.475,
            0.82,
            1.39// these data needs to be tuned
        };

        public static final double ReefRotationAdjustmentRange[] = {
                0,
                0,
                0,
                0,
                0
        };

        public static final double ElevatorHeightsReversed[] = {
            0.,
            0.53,
            0.475,
            0.82,
            1.39
        };

        public static final double ReefRotationAdjustmentRangeReversed[] = {
            0,
            0,
            0,
            0,
            0
        };



        //THESE ARE FOR THE CLIMBING  //COPIED FROM 2025 REEFSCAPE
        
        public static final double ClimbPushDis = 0.48;                             //0.48
        public static final double ClimbRetreatToDis = 0.4;        //0.272         //0.40
                                                 
        public static final double ElevatorClimbHeight = 0.; //TODO
                                                 
        public static final Pose2d BlueClimbPoses[] = {
            new Pose2d(8.3, 7.15, Rotation2d.k180deg),
            new Pose2d(0, 0, Rotation2d.k180deg),
            new Pose2d(0, 0, Rotation2d.k180deg)
        };

        public static final double AlgaeScoreTransalationX = 7.77; //TODO

        public static final double AlgaeScoreDistanceThreshold = 1.; //TODO

        public static final double AutomaticallyAttachDistanceThreshold = 0; //TODO

        public static final double ArmClimbPositionDegs = -140.; //TODO
        public static final double ElevatorAlgaeScoreHeight = 0.9;
    }

    public static void initializeConstants() {
        for (var p : PoseEstimatorConstants.tAtoDevPoints)
        PoseEstimatorConstants.tAtoDev.put(p.getX(), p.getY());
    }
}
