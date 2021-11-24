package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.driving;
import frc.robot.Constants.driving.speeds;

public class Controls {
  private static Controls instance = null;

  public static final double defaultSpeedMult = 0.64;
  public static final double fastSpeedMult = 1.00;

  public static final double defaultRotMult = 0.50;
  public static final double fastRotMult = 1.00;

  public final XboxController controller = new XboxController(0);

  public final Tank tank = new Tank();
  public final Arcade arcade = new Arcade();

  public static Controls getInstance() {
    if (instance == null) {
      instance = new Controls();
    }
    return instance;
  }

  public void adjustRumble() {
    controller.setRumble(RumbleType.kLeftRumble, controller.getStickButton(Hand.kLeft) ? 0.5 : 0);
    controller.setRumble(RumbleType.kRightRumble, controller.getStickButton(Hand.kRight) ? 0.5 : 0);
  }

  public boolean joystickPressed(boolean leftJoystick) {
    return controller.getStickButton(leftJoystick ? Hand.kLeft : Hand.kRight);
  }

  public double getXPow(boolean leftJoystick, int pow) {
    final int sign = pow % 2 == 1 ? 1 : (int) Math.signum(pow);

    return Math.pow(controller.getX(leftJoystick ? Hand.kLeft : Hand.kRight), pow) * sign;
  }

  public double getYPow(boolean leftJoystick, int pow) {
    final int sign = pow % 2 == 1 ? 1 : (int) Math.signum(pow);

    return Math.pow(controller.getY(leftJoystick ? Hand.kLeft : Hand.kRight), pow) * sign;
  }

  public class Tank {
    public double getLeft() {
      return getYPow(true, driving.power) * (joystickPressed(true) ? speeds.fast.move : speeds.normal.move);
    }

    public double getRight() {
      return getYPow(false, driving.power) * (joystickPressed(false) ? speeds.fast.move : speeds.normal.move);
    }

    private Tank() {
    }
  }

  public class Arcade {
    public double getMove() {
      return getYPow(true, driving.power) * (joystickPressed(true) ? speeds.fast.move : speeds.normal.move);
    }

    public double getTurn() {
      return getXPow(false, driving.power) * (joystickPressed(false) ? speeds.fast.move : speeds.normal.move);
    }

    private Arcade() {
    }
  }

  // private to force getInstance()
  private Controls() {
  }
}
