package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Motor;
import frc.robot.Constants.CANID;
import frc.robot.Constants.MKINTAKE;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final Motor motor = Motor.getInstance();
  private final PIDController bottomIntake;
  private final PIDController topIntake;

  private DigitalInput bottomSwitch;
  private DigitalInput topSwitch;

  private TalonFX topLeft = motor.motor(
      CANID.topLeftIntakeCANID,
      MKINTAKE.intakeNeutralMode,
      0,
      MKINTAKE.pidf,
      MKINTAKE.bottomLeftInverted,
      "rio");
  private TalonFX topRight = motor.motor(
      CANID.topRightIntakeCANID,
      MKINTAKE.intakeNeutralMode,
      0,
      MKINTAKE.pidf,
      MKINTAKE.bottomLeftInverted,
      "rio");
  private TalonFX toprollers =  = motor.motor(
    CANID.bottomLeftIntakeCANID,
    MKINTAKE.intakeNeutralMode,
    0,
    MKINTAKE.pidf,
    MKINTAKE.bottomLeftInverted,
    "rio");
  private TalonFX bottomrollers =  = motor.motor(
    CANID.bottomLeftIntakeCANID,
    MKINTAKE.intakeNeutralMode,
    0,
    MKINTAKE.pidf,
    MKINTAKE.bottomLeftInverted,
    "rio");
  private TalonFX bottomLeft = motor.motor(
      CANID.bottomLeftIntakeCANID,
      MKINTAKE.intakeNeutralMode,
      0,
      MKINTAKE.pidf,
      MKINTAKE.bottomLeftInverted,
      "rio");
  private TalonFX bottomRight = motor.motor(
      CANID.bottomRightIntakeCANID,
      MKINTAKE.intakeNeutralMode,
      0,
      MKINTAKE.pidf,
      MKINTAKE.bottomLeftInverted,
      "rio");

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}