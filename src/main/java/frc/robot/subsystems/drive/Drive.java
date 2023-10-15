package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.StateMachineSubsystemBase;

public class Drive extends StateMachineSubsystemBase {
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);

  public final State DISABLED, ARCADE, TANK;

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
  Timer t = new Timer();
  double last_t;
  double openLoopSpeedScaling;

  /** Creates a new Drive. */
  private Drive(DriveIO io) {
    super("Drive");
    this.io = io;

    openLoopSpeedScaling = 1.0;

    DISABLED = new State("DISABLED") {

      @Override
      public void init() {
        stop();
      }

      @Override
      public void periodic() {

      }

      @Override
      public void exit() {

      }
    };

    ARCADE = new State("ARCADE") {
      @Override
      public void periodic() {
        driveArcade(-OI.DR.getLeftY() * openLoopSpeedScaling, -OI.DR.getRightX() * openLoopSpeedScaling);
      }
    };

    TANK = new State("TANK") {
      @Override
      public void periodic() {
        drivePercent(-OI.DR.getLeftY() * openLoopSpeedScaling, -OI.DR.getRightY() * openLoopSpeedScaling);
      }
    };

    setCurrentState(DISABLED);
    t.start();
    last_t = t.get();
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);
  }

  @Override
  public void outputPeriodic() {

    double time = t.get();
    Logger.getInstance().recordOutput("Drive/Delta", time - last_t);
    last_t = time;

    // Update odometry and log the new pose
    odometry.update(new Rotation2d(-inputs.gyroYawRad), getLeftPositionMeters(), getRightPositionMeters());
    Logger.getInstance().recordOutput("Drive/Odometry", getPose());
  }

  /** Run open loop at the specified percentage. */
  public void drivePercent(double leftPercent, double rightPercent) {
    io.setVoltage(leftPercent * 12.0, rightPercent * 12.0);
  }

  /** Run open loop based on stick positions. */
  public void driveArcade(double xSpeed, double zRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
  }

  public void setOpenLoopSpeedScaling(double scaling) {
    this.openLoopSpeedScaling = scaling;
  }

  /** Stops the drive. */
  public void stop() {
    io.setVoltage(0.0, 0.0);
  }

  /** Returns the current odometry pose in meters. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Returns the position of the left wheels in meters. */
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the position of the right wheels in meters. */
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  public double getLeftVelocityMeters() {
    return inputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  public double getRightVelocityMeters() {
    return inputs.rightVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  // Singleton
  private static Drive singleton;

  public static Drive getInstance() {
    if (singleton == null) {
      switch (Constants.currentMode) {
        // Real robot, instantiate hardware IO implementations
        case REAL:
          singleton = new Drive(new DriveIOSparkMax());
          break;

        // Sim robot, instantiate physics sim IO implementations
        case SIM:
          singleton = new Drive(new DriveIOSim());
          break;

        // Replayed robot, disable IO implementations
        default:
          singleton = new Drive(new DriveIO() {
          });
          break;
      }
    }
    return singleton;
  }
}
