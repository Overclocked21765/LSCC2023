package org.firstinspires.ftc.teamcode.common;

public class Constants {
    public static class Drive{
        public static final double DRIVE_POWER_MODIFIER = 1; // drive power
        public static final double ROTATION_CONSTANT = 1; // turning power

        public static final double BFLY_MIN = 0.01;
        public static final double BFLY_MAX = 0.5;

        public static final double ODO_MIN = 0.01;
        public static final double ODO_MAX = 0.4;
    }

    public static class Intake{
        public static final double INTAKE_POWER = 1;
    }

    public static class Outtake{
        public static final double OUTTAKE_ARM_MIN = 0.01;
        public static final double OUTTAKE_ARM_MAX = 0.4;
        public static final double OUTTAKE_ROTATE_MIN = 0.01;
        public static final double OUTTAKE_ROTATE_MAX = 0.3;
        public static final double OUTTAKE_POWER = 0.5;
    }

    public static class Lift{
        public static final double DEPOSIT_POSITION = 450;
        public static final double GROUND_POSITION = 20;
    }

    public static class TSECLimb{
        public static final int CLIMB = 500;
        public static final int ZONE_ONE = 200;
        public static final int ZONE_TWO = 300;
        public static final int ZONE_THREE = 400;
        public static final double GRIP_MIN = 0.01;
        public static final double GRIP_MAX = 0.4;
    }
}
