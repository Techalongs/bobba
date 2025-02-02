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
    private double holdPower = 0.5;
    private int holdThresholdPosition = 800;
    private int holdTargetPosition;
    private DcMotor.RunMode currentRunMode;
    private String status = "None";
    private String statusDetail = "";

    public Extension(DcMotor leftMotor, DcMotor rightMotor, TouchSensor topLimitSensor, TouchSensor bottomLimitSensor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.topLimitSensor = topLimitSensor;
        this.bottomLimitSensor = bottomLimitSensor;
        setTargetPosition(0);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, true);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER, true);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move(double power) {
        this.moveNew(power);
    }

    public void moveNew(double power) {
        double adjustedPower = 0.0;
        if (power < 0.0 && !atBottomLimit()) {
            adjustedPower = power * downSpeedLimiter;
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            holdTargetPosition = getCurrentPosition();
        } else if (power > 0.0 && !atTopLimit()) {
            adjustedPower = power * upSpeedLimiter;
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            holdTargetPosition = getCurrentPosition();
        } else if (atBottomLimit() || (power == 0.0 && (getCurrentPosition() <= holdThresholdPosition))) {
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            adjustedPower = 0.0;
        } else if (atTopLimit() || (power == 0.0 && (getCurrentPosition() > holdThresholdPosition))) {
            setTargetPosition(holdTargetPosition);
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            adjustedPower = holdPower;
        }
        leftMotor.setPower(adjustedPower);
        rightMotor.setPower(adjustedPower);
        statusDetail = "Power=" + power + ", Adjusted Power=" + adjustedPower + ", Position=" + getCurrentPosition() + ", Hold Target=" + holdTargetPosition;
    }

    public void moveOld(double power) {
        double adjustedPower = 0.0;
        if (power == 0.0 && atBottomLimit()) {
            // Do nothing
            status = "At Bottom Limit";
        } else if (power < 0.0 && !atBottomLimit()) {
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

    public void moveOld2(double power) {
        double adjustedPower = 0.0;
        if (power == 0.0 && atBottomLimit()) {
            // Do nothing
            status = "At Bottom Limit";
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.setPower(0);
        } else if (power < 0.0 && !atBottomLimit()) {
            status = "Lowering";
            holdTargetPosition = getCurrentPosition();
            adjustedPower = power * downSpeedLimiter;
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.setPower(adjustedPower);
        } else if (power > 0.0 && !atTopLimit()) {
            status = "Raising";
            holdTargetPosition = getCurrentPosition();
            adjustedPower = power * upSpeedLimiter;
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.setPower(adjustedPower);
        } else {
            status = "Doing Nothing";
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.setPower(0);

            // Do nothing
//            setTargetPosition(holdTargetPosition);
//            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//            adjustedPower = holdPower;
        }
//        leftMotor.setPower(adjustedPower);
//        rightMotor.setPower(adjustedPower);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        this.setRunMode(runMode, false);
    }

    public void setRunMode(DcMotor.RunMode runMode, boolean force) {
        if (force || currentRunMode != runMode) {
            leftMotor.setMode(runMode);
            rightMotor.setMode(runMode);
            currentRunMode = runMode;
        }
    }

    public DcMotor.RunMode getCurrentRunMode() {
        return currentRunMode;
    }

    public boolean atBottomLimit() {
        return bottomLimitSensor != null && bottomLimitSensor.isPressed();
    }

    public boolean atTopLimit() {
        return topLimitSensor != null && topLimitSensor.isPressed();
    }

    public int getCurrentPosition() {
        if (leftMotor != null) {
            return leftMotor.getCurrentPosition();
        } else if (rightMotor != null) {
            return rightMotor.getCurrentPosition();
        } else {
            throw new RuntimeException("Both motors are null!");
        }
    }

    public void setTargetPosition(int position) {
        if (leftMotor != null) {
            leftMotor.setTargetPosition(position);
        }
        if (rightMotor != null) {
            rightMotor.setTargetPosition(position);
        }
    }

    public double getPower() {
        if (leftMotor != null) {
            return leftMotor.getPower();
        } else if (rightMotor != null) {
            return rightMotor.getPower();
        } else {
            throw new RuntimeException("Both motors are null!");
        }
    }

    public void setPower(double power) {
        if (leftMotor != null) {
            leftMotor.setPower(power);
        }
        if (rightMotor != null) {
            rightMotor.setPower(power);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (leftMotor != null) {
            leftMotor.setZeroPowerBehavior(behavior);
        } else if (rightMotor != null) {
            rightMotor.setZeroPowerBehavior(behavior);
        }
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

    public String getStatus(){
        return status;
    }

    public String getStatusDetail() {
        return statusDetail;
    }

    public void move2(double power) {
        double adjustedPower = 0.0;
        if (power < 0.0 && !atBottomLimit()) {
            adjustedPower = power * downSpeedLimiter;
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            holdTargetPosition = getCurrentPosition();

            leftMotor.setPower(adjustedPower);
            rightMotor.setPower(adjustedPower);

            status = "Lowering";
            statusDetail = "Power=" + power + ", Adjusted Power=" + adjustedPower + ", Position=" + getCurrentPosition();

        } else if (power > 0.0 && !atTopLimit()) {
            adjustedPower = power * upSpeedLimiter;
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            holdTargetPosition = getCurrentPosition();

            leftMotor.setPower(adjustedPower);
            rightMotor.setPower(adjustedPower);

            status = "Raising";
            statusDetail = "Power=" + power + ", Adjusted Power=" + adjustedPower + ", Position=" + getCurrentPosition();

        } else {
            if (getCurrentPosition() > holdThresholdPosition) {
                setTargetPosition(holdTargetPosition);
                setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                adjustedPower = holdPower;

                leftMotor.setPower(adjustedPower);
                rightMotor.setPower(adjustedPower);

                status = "Holding";
                statusDetail = "Power=" + power + ", Adjusted Power=" + adjustedPower + ", Position=" + getCurrentPosition();
            } else {
                adjustedPower = 0;
                setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                holdTargetPosition = getCurrentPosition();

                leftMotor.setPower(adjustedPower);
                rightMotor.setPower(adjustedPower);

                status = "Doing Absolutely Nothing";
                statusDetail = "Power=" + power + ", Adjusted Power=" + adjustedPower + ", Position=" + getCurrentPosition();
            }

        }
        leftMotor.setPower(adjustedPower);
        rightMotor.setPower(adjustedPower);
    }

}
