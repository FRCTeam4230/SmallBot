// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Limelight;
import frc.robot.Constants.driving;

/** An example command that uses an example subsystem. */
public class Drive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_subsystem;
  private Limelight lime = Limelight.getInstance();
  private Limelight.TargetDistancer targetDistancer = lime.new TargetDistancer(9.5, 32.1, 52);

  private NetworkTableEntry log = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DB")
      .getEntry("String 0");
  private NetworkTableEntry log2 = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DB")
      .getEntry("String 1");
  private NetworkTableEntry log3 = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DB")
      .getEntry("String 2");
  // private NetworkTableEntry log4 =
  // NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DB").getEntry("String
  // 3");

  private NetworkTableEntry aimDistance = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DB")
      .getEntry("Slider 0");

  private static final int maxHorizDegrees = 27;
  private static final double turnBase = 0.4;
  private static final double turnSpeed = 1.0 - turnBase;
  private static final double moveScale = 30;
  private static final double moveBase = 0.2;
  private static final double moveSpeed = 1.0 - moveBase;

  private static final double distanceRange = 4;
  private static final double turnRange = 1;

  private Drivetrain.Encoders.Target target;

  /**
   * Creates a new Drive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(Drivetrain subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lime.leds.auto();
    target = m_subsystem.encoders.new Target(-5, 5);

    aimDistance.setDouble(60);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (false) {
      final double lDist = target.getLeftDistance();
      final double rDist = target.getRightDistance();

      m_subsystem.tankDrive(((1 - 1.0 / (1 + Math.abs(lDist) / 2)) * 0.5 + 0.3) * Math.signum(lDist),
          ((1 - 1.0 / (1 + Math.abs(rDist) / 2)) * 0.7 + 0.3) * Math.signum(rDist));

      log.setString("" + target.getLeftDistance());
      log2.setString("" + m_subsystem.encoders.right.getDistance());
      log3.setBoolean(target.isWithinRange(0.1) && target.hasReached());

      return;
    }

    // read values
    double x = lime.getX();

    double goalDistance = aimDistance.getDouble(60);
    double currentDistance = targetDistancer.get();
    double distanceOffset = goalDistance - currentDistance;

    log.setString("" + distanceOffset);
    log2.setString("" + currentDistance);

    double speed = 1 - 1 / (1 + Math.abs(distanceOffset) / moveScale);
    log3.setString("" + speed);

    if (!lime.hasTarget()) {
      m_subsystem.stop();
      return;
    }

    m_subsystem.arcadeDrive(
        Math.abs(distanceOffset) < distanceRange ? 0
            : (speed * moveSpeed + moveBase) * Math.signum(distanceOffset) * driving.direction,
        Math.abs(x) < turnRange ? 0 : x / maxHorizDegrees * turnSpeed + Math.signum(x) * turnBase);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
    lime.leds.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return target.isWithinRange(0.1) && target.hasLeftReached();
    // return false;
    // return timer.get() > 3;
  }
}
