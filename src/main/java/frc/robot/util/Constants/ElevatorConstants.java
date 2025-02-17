package frc.robot.util.Constants;

import org.xml.sax.SAXNotRecognizedException;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static int kMotorID = 10;
  public static int kFollowerMotorID = 11;
  public static MotorType kMotorType = MotorType.kBrushless;
  public static IdleMode kIdleMode = IdleMode.kBrake;
  public static int kCurrentLimit = 40;
  public static double kMinHeight = 0;
  public static double kMaxHeight = 0;
  public static double kP = 1.5; // TODO: Configure me!
  public static double kI = 0.0;
  public static double kD = 0.08; // TODO: Configure me!
  public static double kVelocityFF = 0.0; // TODO: Configure me!

  public static double kMaxAcceleration = 250;
  public static double kMaxVelocity = 500;
  public static double kTolerance = 1;

  public static double kForwardSoftLimit = 70;
  public static double kReverseSoftLimit = 0;

  // public static double kDrumDiameterM = Units.inchesToMeters(2.635); // Sprocket diameter
  // public static double kDrumCircumference = kDrumDiameterM * Math.PI;

  // public static double kGearRatio = 25 / 1;

  public static double kS = 0.0;
  public static double kG = 0.9;
  public static double kV = 0.0;
  public static double kA = 0.0;

  public static double kPositionConversionFactor = 1;
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

  public static final SparkMaxConfig kElevatorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig kElevatorFollowerConfig = new SparkMaxConfig();

  public enum Positions {
    BOTTOM(0),
    PREINTAKE(0),
    POSTINTAKE(0),
    L1(1),
    L2(0),
    L3(0),
    L4(0),
    SCORE(0),
    BARGE(0);

    public final double position;

    private Positions(double position) {
      this.position = position;
    }

    public double getPos() {
      return this.position;
    }
  };

  static {
    kElevatorFollowerConfig
    .follow(kMotorID, true)
    .idleMode(kIdleMode)
    .smartCurrentLimit(kCurrentLimit);

    kElevatorConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit);

    kElevatorConfig
        .encoder
        .positionConversionFactor(kPositionConversionFactor)
        .velocityConversionFactor(kVelocityConversionFactor);

    kElevatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP, 0.0, kD, kVelocityFF)
        .outputRange(-1, 1);

    kElevatorConfig
        .closedLoop
        .maxMotion
        .maxVelocity(kMaxVelocity)
        .maxAcceleration(kMaxAcceleration)
        .allowedClosedLoopError(kTolerance);
  }
}