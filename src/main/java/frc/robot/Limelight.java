package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
  private static Limelight instance = null;

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private final NetworkTableEntry tx = table.getEntry("tx");
  private final NetworkTableEntry ty = table.getEntry("ty");
  private final NetworkTableEntry ta = table.getEntry("ta");
  private final NetworkTableEntry tv = table.getEntry("tv");
  private final NetworkTableEntry ts = table.getEntry("ts");
  private final NetworkTableEntry tl = table.getEntry("tl");
  private final NetworkTableEntry tshort = table.getEntry("tshort");
  private final NetworkTableEntry tlong = table.getEntry("tlong");
  private final NetworkTableEntry thor = table.getEntry("thor");
  private final NetworkTableEntry tvert = table.getEntry("tvert");

  public final LEDs leds = new LEDs();
  public final Pipeline pipeline = new Pipeline();
  public final Snapshots snapshots = new Snapshots();
  public final StreamingMode streamingMode = new StreamingMode();
  public final Processing processing = new Processing();

  public class TargetDistancer {
    private final double heightDifference;
    private final double angle;

    public TargetDistancer(double cameraHeight, double cameraAngle, double targetHeight) {
      heightDifference = targetHeight - cameraHeight;
      angle = Math.toRadians(cameraAngle);
    }

    public double get() {
      return heightDifference / Math.tan(angle + Math.toRadians(getY()));
    }

    public double getHeight() {
      return heightDifference;
    }

    public double getAngle() {
      return Math.toDegrees(angle);
    }
  }

  public static Limelight getInstance() {
    if (instance == null) {
      instance = new Limelight();
    }
    return instance;
  }

  public NetworkTable getTable() {
    return table;
  }

  public NetworkTableEntry getEntry(String key) {
    return table.getEntry(key);
  }

  public double getY() {
    return ty.getDouble(0);
  }

  public double getX() {
    return tx.getDouble(0);
  }

  public double getArea() {
    return ta.getDouble(0);
  }

  public boolean hasTarget() {
    return tv.getDouble(0) != 0;
  }

  public double getSkew() {
    return ts.getDouble(0);
  }

  public double getLatency() {
    return tl.getDouble(0);
  }

  public double getShortSide() {
    return tshort.getDouble(0);
  }

  public double getLongSide() {
    return tlong.getDouble(0);
  }

  public double getHorizontalSide() {
    return thor.getDouble(0);
  }

  public double getVertalSide() {
    return tvert.getDouble(0);
  }

  public class Pipeline {
    private final NetworkTableEntry pipeline = table.getEntry("pipeline");

    public void set(int pipelineNum) {
      if (pipelineNum < 0 || pipelineNum > 9) {
        return;
      }

      pipeline.setNumber(pipelineNum);
    }

    public int get() {
      return (int) pipeline.getDouble(-1);
    }

    private Pipeline() {
    }
  }

  public class LEDs {
    private final NetworkTableEntry LEDMode = table.getEntry("ledMode");

    public void auto() {
      setMode(0);
    }

    public void disable() {
      setMode(1);
    }

    public void blink() {
      setMode(2);
    }

    public void forceOn() {
      setMode(3);
    }

    public int get() {
      return (int) LEDMode.getDouble(-1);
    }

    private void setMode(int modeNum) {
      LEDMode.setNumber(modeNum);
    }

    private LEDs() {
    }
  }

  public class Snapshots {
    private final NetworkTableEntry snapshot = table.getEntry("snapshot");

    public boolean isEnabled() {
      return snapshot.getDouble(0) != 0;
    }

    public void enable(boolean enabled) {
      snapshot.setNumber(enabled ? 1 : 0);
    }

    private Snapshots() {
    }
  }

  public class StreamingMode {
    private final NetworkTableEntry stream = table.getEntry("stream");

    public int get() {
      return (int) stream.getDouble(-1);
    }

    public boolean isPIP() {
      return get() > 0;
    }

    public void setNormal() {
      stream.setNumber(1);
    }

    public void setPIP(boolean main) {
      stream.setNumber(main ? 1 : 2);
    }

    private StreamingMode() {
    }
  }

  public class Processing {
    private final NetworkTableEntry camMode = table.getEntry("camMode");

    public boolean isEnabled() {
      return camMode.getDouble(1) == 0;
    }

    public void enable(boolean enabled) {
      camMode.setNumber(enabled ? 0 : 1);
    }

    private Processing() {
    }
  }

  // private to force getInstance()
  private Limelight() {
  }
}
