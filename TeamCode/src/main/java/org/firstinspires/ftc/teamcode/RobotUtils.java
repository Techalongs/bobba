package org.firstinspires.ftc.teamcode;

public class RobotUtils {

    public final static int REV_CORE_HEX_MOTOR_TICKS_PER_REVOLUTION = 288;

    public static int degreesToTicksRevCoreHex(double degrees) {
        int ticks = (int) ((degrees / 360.0) * REV_CORE_HEX_MOTOR_TICKS_PER_REVOLUTION);
        return ticks;
    }

}
