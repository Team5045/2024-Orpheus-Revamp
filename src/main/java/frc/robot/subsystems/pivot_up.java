package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.pivotUpConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/** A robot arm subsystem that moves with a motion profile. */
public class pivot_up extends ProfiledPIDSubsystem {
  private final TalonFX m_motor_right = new TalonFX(pivotUpConstants.kMotorPort_Right);
  private final TalonFX m_motor_left = new TalonFX(pivotUpConstants.kMotorPort_Left);
  private final TalonFX m_encoder = m_motor_left;

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          pivotUpConstants.kSVolts, pivotUpConstants.kGVolts,
          pivotUpConstants.kVVoltSecondPerRad, pivotUpConstants.kAVoltSecondSquaredPerRad);

  public pivot_up() {
    super(
        new ProfiledPIDController(
            pivotUpConstants.kP,
            pivotUpConstants.kI,
            pivotUpConstants.kD,
            new TrapezoidProfile.Constraints(
                pivotUpConstants.kMaxVelocityRadPerSecond,
                pivotUpConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    setGoal(pivotUpConstants.kPivotOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {

    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    m_motor_right.setVoltage(output + feedforward);
    m_motor_left.setVoltage(-1 * (output + feedforward));
  }

  @Override
  public double getMeasurement() {
    return (m_encoder.getDifferentialAveragePosition().getValueAsDouble());
  }
}
