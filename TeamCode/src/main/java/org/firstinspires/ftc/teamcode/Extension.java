package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Extension {

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private final TouchSensor topLimitSensor;
    private final TouchSensor bottomLimitSensor;
    private double upSpeedLimiter = 0.5;
    private double downSpeedLimiter = 0.5;
    private DcMotor.RunMode currentRunMode;
    private double holdPower = 0.75;
    private int holdTargetPosition;

    public Extension(DcMotor leftMotor, DcMotor rightMotor, TouchSensor topLimitSensor, TouchSensor bottomLimitSensor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.topLimitSensor = topLimitSensor;
        this.bottomLimitSensor = bottomLimitSensor;
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        currentRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    }

    public void move(double power) {
        double adjustedPower = 0.0;
        if (power < 0.0 && !atBottomLimit()) {
            adjustedPower = power * downSpeedLimiter;
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            holdTargetPosition = getCurrentPosition();
        } else if (power > 0.0 && !atTopLimit()) {
            adjustedPower = power * upSpeedLimiter;
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            holdTargetPosition = getCurrentPosition();
        } else {
            setTargetPosition(holdTargetPosition);
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            adjustedPower = holdPower;
        }
        leftMotor.setPower(adjustedPower);
        rightMotor.setPower(adjustedPower);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        if (currentRunMode != runMode) {
            leftMotor.setMode(runMode);
            rightMotor.setMode(runMode);
            currentRunMode = runMode;
        }
    }

    public boolean atBottomLimit() {
        return bottomLimitSensor != null && bottomLimitSensor.isPressed();
    }

    public boolean atTopLimit() {
        return topLimitSensor != null && topLimitSensor.isPressed();
    }

    public int getCurrentPosition() {
        return leftMotor.getCurrentPosition();
    }

    public void setTargetPosition(int position) {
        leftMotor.setTargetPosition(position);
        rightMotor.setTargetPosition(position);
    }

    public double getPower() {
        return leftMotor.getPower();
    }

    public void setSpeedLimiter(double speedLimiter) {
        setUpSpeedLimiter(speedLimiter);
        setDownSpeedLimiter(speedLimiter);
    }

    public void setUpSpeedLimiter(double upSpeedLimiter) {
        this.upSpeedLimiter = upSpeedLimiter;
    }

    public double getUpSpeedLimiter() {
        return upSpeedLimiter;
    }

    public void setDownSpeedLimiter(double downSpeedLimiter) {
        this.downSpeedLimiter = downSpeedLimiter;
    }

    public double getDownSpeedLimiter() {
        return downSpeedLimiter;
    }

    public TouchSensor getTopLimitSensor() {
        return topLimitSensor;
    }

    public TouchSensor getBottomLimitSensor() {
        return bottomLimitSensor;
    }

    public DcMotor getLeftMotor() {
        return leftMotor;
    }

    public DcMotor getRightMotor() {
        return rightMotor;
    }

}
