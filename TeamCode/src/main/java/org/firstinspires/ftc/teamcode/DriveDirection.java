package org.firstinspires.ftc.teamcode;

public enum DriveDirection {

    FORWARD(1, 1, 1, 1),
    LEFT(-1, 1, 1, -1),
    RIGHT(1, -1, -1, 1),
    BACKWARD(-1, -1, -1, -1);

    private final int frontLeftFactor;
    private final int frontRightFactor;
    private final int backLeftFactor;
    private final int backRightFactor;

    DriveDirection(int frontLeftFactor, int frontRightFactor, int backLeftFactor, int backRightFactor) {
        this.frontLeftFactor = frontLeftFactor;
        this.frontRightFactor = frontRightFactor;
        this.backLeftFactor = backLeftFactor;
        this.backRightFactor = backRightFactor;
    }

    public int getFrontLeftFactor() {
        return frontLeftFactor;
    }

    public int getFrontRightFactor() {
        return frontRightFactor;
    }

    public int getBackLeftFactor() {
        return backLeftFactor;
    }

    public int getBackRightFactor() {
        return backRightFactor;
    }

}
