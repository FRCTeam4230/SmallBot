// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants;
import frc.robot.Constants.CANId;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private WPI_TalonSRX bLeft, fRight;
  private WPI_VictorSPX fLeft, bRight;
  private SpeedControllerGroup lGroup, rGroup;
  private DifferentialDrive driveSys;

  public final Encoders encoders = new Encoders();

  public Drivetrain() {
    bLeft = new WPI_TalonSRX(CANId.kLeftMotorBackPort);
    bLeft.setInverted(true);
    fLeft = new WPI_VictorSPX(CANId.kLeftMotorFrontPort);
    fLeft.setInverted(true);

    fRight = new WPI_TalonSRX(CANId.kRightMotorFrontPort);
    fRight.setInverted(true);
    bRight = new WPI_VictorSPX(CANId.kRightMotorBackPort);
    bRight.setInverted(true);

    lGroup = new SpeedControllerGroup(fLeft, bLeft);
    rGroup = new SpeedControllerGroup(fRight, bRight);

    driveSys = new DifferentialDrive(lGroup, rGroup);
  }

  public class Encoders {
    public final Encoder right = new Encoder(0, 1);
    public final Encoder left = new Encoder(2, 3, true);

    private Encoders() {
      left.setDistancePerPulse(1.0 / Constants.encoders.pulsesPerRotation);
      right.setDistancePerPulse(1.0 / Constants.encoders.pulsesPerRotation);

      left.reset();
      right.reset();
    }

    public void reset() {
      left.reset();
      right.reset();
    }

    public double getDifference() {
      return getDifference(new Mark(0, 0));
    }

    public double getDifference(Mark mark) {
      final double rightChange = right.getDistance() - mark.getRightPos();
      final double leftChange = left.getDistance() - mark.getLeftPos();

      return leftChange - rightChange;
    }

    private double angleToDistanceDifference(double angle) {
      System.out.println("before: " + angle);
      angle = angle % 360;
      if (angle < 0) {
        angle = 360 + angle;
      }
      System.out.println("after: " + angle);
      return (angle - (angle > 180 ? 360 : 0)) / 360 * Constants.encoders.distancePerRotation;
    }

    public class Mark {
      private double leftPos;
      private double rightPos;

      public Mark() {
        this(0, 0, true);
      }

      public Mark(double angle) {
        this(-angleToDistanceDifference(angle) / 2, angleToDistanceDifference(angle) / 2, true);
      }

      private Mark(double lPos, double rPos) {
        this(lPos, rPos, false);
      }

      private Mark(double lPos, double rPos, boolean relative) {
        leftPos = lPos + (relative ? left.getDistance() : 0);
        rightPos = rPos + (relative ? right.getDistance() : 0);
      }

      public double getLeftPos() {
        return leftPos;
      }

      public double getRightPos() {
        return rightPos;
      }

      public double getLeftDistance() {
        return left.getDistance() - leftPos;
      }

      public double getRightDistance() {
        return right.getDistance() - rightPos;
      }
    }

    public class Target {
      private Mark targetMark;
      private int rightDirection, leftDirection;

      public Target(double distance) {
        this(new Mark(), distance, distance);
      }

      public Target(double leftDistance, double rightDistance) {
        this(new Mark(), leftDistance, rightDistance);
      }

      public Target(Mark target) {
        this(new Mark(), target);
      }

      private Target(Mark starting, double leftDistance, double rightDistance) {
        this(starting, new Mark(starting.getLeftPos() + leftDistance, starting.getRightPos() + rightDistance));
      }

      private Target(Mark starting, Mark target) {
        targetMark = target;

        rightDirection = (int) Math.signum(starting.getRightPos() - targetMark.getRightPos());
        leftDirection = (int) Math.signum(starting.getLeftPos() - targetMark.getLeftPos());
      }

      public double getLeftDistance() {
        return -targetMark.getLeftDistance();
      }

      public double getRightDistance() {
        return -targetMark.getRightDistance();
      }

      public boolean hasLeftReached() {
        return getLeftDistance() * leftDirection >= 0;
      }

      public boolean hasRightReached() {
        return getRightDistance() * rightDirection >= 0;
      }

      public boolean hasReached() {
        return hasLeftReached() && hasRightReached();
      }

      public boolean isLeftInRange(double range) {
        return Math.abs(getLeftDistance() * leftDirection) <= range;
      }

      public boolean isRightInRange(double range) {
        return Math.abs(getRightDistance() * rightDirection) <= range;
      }

      public boolean isWithinRange(double range) {
        return isLeftInRange(range) && isRightInRange(range);
      }
    }
  }

  public void tankDrive(double lSpeed, double rSpeed) {
    driveSys.tankDrive(lSpeed, rSpeed);
  }

  public void arcadeDrive(double fwd, double rot) {
    driveSys.arcadeDrive(fwd, rot);
  }

  public void stop() {
    driveSys.arcadeDrive(0, 0);
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
