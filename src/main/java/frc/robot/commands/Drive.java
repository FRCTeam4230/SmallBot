// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class Drive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_subsystem;
  // private Timer timer;
  private NetworkTable limeTable;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  private static final int MAXHORIZDEGREES = 27;
  private static final double MAXVERTDEGREES = 20.5;
  private static final double TURNSPEED = 0.5;
  private static final double TURNBASE = 0.3;
  private static final double MOVESPEED = 0.5;
  private static final double MOVEBASE = 0.2;
  // private static final double MOVESPEED = 0;
  // private static final double MOVEBASE = 0;

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
    limeTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limeTable.getEntry("tx");
    ty = limeTable.getEntry("ty");
    ta = limeTable.getEntry("ta");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (timer.get() < 1)
    //   m_subsystem.arcadeDrive(0.5, 0);
    // else if (timer.get() < 2)
    //   m_subsystem.tankDrive(0, 0.5);
    // else if (timer.get() < 3)
    //   m_subsystem.tankDrive(0.5, 0);
    // m_subsystem.arcadeDrive(0, Math.signum(limeTable.getEntry("tx").getDouble(0)));

    //read values
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    m_subsystem.arcadeDrive(-y / MAXVERTDEGREES * MOVESPEED - Math.signum(y) * MOVEBASE,
        x / MAXHORIZDEGREES * TURNSPEED + Math.signum(x) * TURNBASE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return timer.get() > 3;
  }
}
