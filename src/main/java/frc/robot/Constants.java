// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CANId {
        public static final int kLeftMotorFrontPort = 2;
        public static final int kLeftMotorBackPort = 1;
        public static final int kRightMotorFrontPort = 3;
        public static final int kRightMotorBackPort = 4;
    }

    public static final class encoders {
        public static final int pulsesPerRotation = 2048;
        public static final double distancePerRotation = 7.0 - 0.15;
    }

    public static class driving {
        public static final boolean useArcadeControls = true;

        public static int power = 2;
        public static final int direction = -1; // set to 1 or -1

        public static class speeds {
            public static class normal {
                public static double move = 0.64;
                public static double turn = 0.5;
            }

            public static class fast {
                public static double move = 1.0;
                public static double turn = 1.0;
            }
        }
    }
}
