// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Limelight;
import frc.robot.Limelight.Pipeline;

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
  // private NetworkTableEntry log4 = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DB").getEntry("String 3");

  private NetworkTableEntry aimDistance = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DB")
      .getEntry("Slider 0");

  private static final int MAXHORIZDEGREES = 27;
  private static final double TURNSPEED = 0.5;
  private static final double TURNBASE = 0.3;
  private static final double MOVESPEED = 1.0;
  private static final double MOVEBASE = 0.2;

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

    aimDistance.setDouble(60);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //read values
    double x = lime.getX();

    double goalDistance = aimDistance.getDouble(60);
    double currentDistance = targetDistancer.get();
    double distanceOffset = goalDistance - currentDistance;

    log.setString("" + distanceOffset);
    log2.setString("" + currentDistance);

    double speed = -Math.signum(distanceOffset) * (1 - 1 / (1 + Math.abs(distanceOffset) / 13));
    log3.setString("" + speed);

    m_subsystem.arcadeDrive(speed * MOVESPEED,
        Math.abs(x) < 3 ? 0 : x / MAXHORIZDEGREES * TURNSPEED + Math.signum(x) * TURNBASE);
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
    return false;
    // return timer.get() > 3;
  }
}
