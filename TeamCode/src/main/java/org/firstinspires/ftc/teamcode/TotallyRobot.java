package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TotallyRobot {

    private final int forwardTicksPerInch = (int) Math.round(1000.0 / 18.0);
    private final int sidewaysTicksPerInch = (int) Math.round(1000.0 / 15.0);

    private LinearOpMode opMode;
    private IMU imu;
    private DcMotor motorFrontRightB0;
    private DcMotor motorFrontLeftB1;
    private DcMotor motorBackRightB2;
    private DcMotor motorBackLeftB3;
//    private DcMotor motorArmHinge;
    private Elbow elbow;
    private Extension extension;
    private Servo servoClawWristHinge;
    private Servo servoClawFinger1;
    private Servo servoClawFinger2;

    private double driveSpeedLimiter = 0.5;
    private double extensionSpeedLimiter = 0.5;
    private double armHingeSpeedForwordLimiter = 0.5;
    private double armHingeSpeedBackwardsLimiter = 0.5;
    private double clawWristSpeedLimiter = 0.7;
    private double clawFingerSpeedOpenLimiter = 0.8;
    private double currentBatteryVoltage = 0.0;
    private double alertMinBatteryVoltage = 12.0;

    // Wrist variables for testing out wrist stuff
    private double wristMinPosition = 0.0;
    private double wristMaxPosition = 0.5;
    private double wristTicksPerCycle = 0.01;
    private boolean useFieldCentricDriving = false;

    public void settingBotUp(LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        // Wheels
        motorFrontRightB0 = initBasicDcMotor("fr0");
        motorFrontLeftB1 = initBasicDcMotor("fl1");
        motorBackRightB2 = initBasicDcMotor("br2");
        motorBackLeftB3 = initBasicDcMotor("bl3");
        motorFrontLeftB1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeftB3.setDirection(DcMotorSimple.Direction.REVERSE);

        // Elbow
        elbow = new Elbow(this.opMode, (DcMotorEx) initBasicDcMotor("mah"));

        // Wrist
        servoClawWristHinge = hardwareMap.get(Servo.class, "scwh");
        servoClawWristHinge.setDirection(Servo.Direction.REVERSE);

        // Claw
        servoClawFinger1 = hardwareMap.get(Servo.class,"scf1");
        servoClawFinger2 = hardwareMap.get(Servo.class,"scf2");
        servoClawFinger1.setDirection(Servo.Direction.REVERSE);

        TouchSensor sensorExtensionBottom = hardwareMap.get(TouchSensor.class, "sebl");
//        sensorArmHingeTop = hardwareMap.get(TouchSensor.class, "saht");

        extension = new Extension(
                initBasicDcMotor("mel"),
                initBasicDcMotor("mer"),
                null,
                sensorExtensionBottom
        );
        extension.getLeftMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        extension.getRightMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        extension.setSpeedLimiter(extensionSpeedLimiter);
        extension.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoClawWristHinge.setPosition(wristMaxPosition);

        updateBatteryVoltage();


//            // Driving
//            if (useFieldCentricDriving) {
//                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//                this.driveFieldCentric(driveSpeedLimiter, gamepad1, botHeading);
//            } else {
//                this.driveRobotCentric(driveSpeedLimiter, gamepad1);
//            }

            // Elbow
//            elbow.update();

//            // Extension
//            double extensionPower = -gamepad2.right_stick_y;
//            motorExtensions.move(extensionPower);

//            // Wrist
//            double wristPositionControl = gamepad2.left_stick_y;
//            double wristGetPosition = servoClawWristHinge.getPosition();
//            if (wristPositionControl > 0.0) {
//                double wristNewPosition = (wristTicksPerCycle * wristPositionControl) + wristGetPosition;
//                if (wristNewPosition > wristMaxPosition) {
//                    wristNewPosition = wristMaxPosition;
//                }
//                servoClawWristHinge.setPosition(wristNewPosition);
//            } else if (wristPositionControl < 0) {
//                double wristNewPosition = (wristTicksPerCycle * wristPositionControl) + wristGetPosition;
//                if (wristNewPosition < wristMinPosition) {
//                    wristNewPosition = wristMinPosition;
//                }
//                servoClawWristHinge.setPosition(wristNewPosition);
//            }

//            // Claw
//            double clawRightTrigger = gamepad2.right_trigger;
//            if (clawRightTrigger > 0) {
//                closeClaws();
//            }
//            double clawLeftTrigger = gamepad2.left_trigger;
//            if (clawLeftTrigger > 0) {
//                openClaws();
//            }
//
//            // Display
//            this.displayData();
//        }
    }

    public void openClaws() {
        servoClawFinger1.setPosition(1);
        servoClawFinger2.setPosition(1);
    }

    public void closeClaws() {
        servoClawFinger1.setPosition(0);
        servoClawFinger2.setPosition(0);
    }

    public void clawHingeUp() {
        servoClawWristHinge.setPosition(0);
    }

    public void clawHingeDown() {
        servoClawWristHinge.setPosition(1);
    }

    public void setWheelsMode(DcMotor.RunMode runMode) {
        motorFrontLeftB1.setMode(runMode);
        motorFrontRightB0.setMode(runMode);
        motorBackLeftB3.setMode(runMode);
        motorBackRightB2.setMode(runMode);
    }

    public void setWheelsPower(double power) {
        this.setWheelsPower(power, power, power, power);
    }

    public void setWheelsPower(double frontLeft, double frontRight, double backLeft, double backRight) {
        motorFrontLeftB1.setPower(frontLeft);
        motorFrontRightB0.setPower(frontRight);
        motorBackLeftB3.setPower(backLeft);
        motorBackRightB2.setPower(backRight);
    }

    public void setWheelsPower(double frontLeft, double frontRight, double backLeft, double backRight, double limiter) {
        motorFrontLeftB1.setPower(frontLeft * limiter);
        motorFrontRightB0.setPower(frontRight * limiter);
        motorBackLeftB3.setPower(backLeft * limiter);
        motorBackRightB2.setPower(backRight * limiter);
    }

    public void setWheelsTargetPosition(int frontLeft, int frontRight, int backLeft, int backRight) {
        motorFrontLeftB1.setTargetPosition(frontLeft);
        motorFrontRightB0.setTargetPosition(frontRight);
        motorBackLeftB3.setTargetPosition(backLeft);
        motorBackRightB2.setTargetPosition(backRight);
    }

    public boolean isWheelsBusy() {
        return motorFrontLeftB1.isBusy() && motorFrontRightB0.isBusy() && motorBackLeftB3.isBusy() && motorBackRightB2.isBusy();
    }

    public void driveTicks(DriveDirection direction, int ticks, double power, long waitMillis) {
        this.setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setWheelsMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.setWheelsTargetPosition(
                ticks * direction.getFrontLeftFactor(),
                ticks * direction.getFrontRightFactor(),
                ticks * direction.getBackLeftFactor(),
                ticks * direction.getBackRightFactor()
        );
        this.setWheelsPower(power);
        this.waitMillis(waitMillis);
        this.setWheelsPower(0.0);
    }


    public void driveTicks(DriveDirection direction, int ticks, double power) {
        this.setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setWheelsMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.setWheelsTargetPosition(
                ticks * direction.getFrontLeftFactor(),
                ticks * direction.getFrontRightFactor(),
                ticks * direction.getBackLeftFactor(),
                ticks * direction.getBackRightFactor()
        );
        this.setWheelsPower(power);
        while (isWheelsBusy()) {
            this.waitMillis(250);
        }
        this.setWheelsPower(0.0);
    }

    public void driveInches(DriveDirection direction, double inches, double power, long waitMillis) {
        int ticks = this.convertInchesToTicks(direction, inches);
        this.driveTicks(direction, ticks, power, waitMillis);
    }

    public void driveInches(DriveDirection direction, double inches, double power) {
        int ticks = this.convertInchesToTicks(direction, inches);
        this.driveTicks(direction, ticks, power);
    }

    public void driveRobotCentric(Gamepad gamepad) {
        float flPower = (-gamepad.left_stick_y + gamepad.right_stick_x) + gamepad.left_stick_x;
        float frPower = (-gamepad.left_stick_y - gamepad.right_stick_x) - gamepad.left_stick_x;
        float blPower = (-gamepad.left_stick_y + gamepad.right_stick_x) - gamepad.left_stick_x;
        float brPower = (-gamepad.left_stick_y - gamepad.right_stick_x) + gamepad.left_stick_x;
        this.setWheelsPower(flPower, frPower, blPower, brPower, driveSpeedLimiter);
    }

    public void driveFieldCentric(Gamepad gamepad, double botHeading) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPower = (rotY + rotX + rx) / denominator;
        double blPower = (rotY - rotX + rx) / denominator;
        double frPower = (rotY - rotX - rx) / denominator;
        double brPower = (rotY + rotX - rx) / denominator;

        this.setWheelsPower(flPower, frPower, blPower, brPower, driveSpeedLimiter);
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : opMode.hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    /**
     * Update battery voltage every 60 seconds.
     */
    private void updateBatteryVoltage() {
        if (opMode.getRuntime() % 60 == 0) {
            currentBatteryVoltage = getBatteryVoltage();
        }
    }

    public void loopUpdate() {
        this.updateBatteryVoltage();
        elbow.update();
        this.updateTelemetry();
    }

    public void updateTelemetry() {
        Telemetry telemetry = opMode.telemetry;
        if (currentBatteryVoltage < alertMinBatteryVoltage) {
            telemetry.addData("Voltage", "%.2f", currentBatteryVoltage);
            telemetry.addData("VOLTAGE ALERT", "VOLTAGE ALERT");
        }
//        telemetry.addData("F1 Pos", servoClawFinger1.getPosition());
//        telemetry.addData("F2 Pos", servoClawFinger2.getPosition());
//        telemetry.addData("Ext Status", motorExtensions.getStatus());
//        telemetry.addData("Ext Status Detail", motorExtensions.getStatusDetail());
//        telemetry.addData("Ext Position", motorExtensions.getCurrentPosition());
//        telemetry.addData("Ext RunMode", motorExtensions.getCurrentRunMode());
//        telemetry.addData("LEFT Power", motorExtensions.getLeftMotor().getPower());
//        telemetry.addData("LEFT RunMode", motorExtensions.getLeftMotor().getMode());
//        telemetry.addData("LEFT Targ Pos", motorExtensions.getLeftMotor().getTargetPosition());
//        telemetry.addData("LEFT Curr Pos", motorExtensions.getLeftMotor().getCurrentPosition());
//        telemetry.addData("LEFT Dir", motorExtensions.getLeftMotor().getDirection());
//        telemetry.addData("RIGHT Power", motorExtensions.getRightMotor().getPower());
//        telemetry.addData("RIGHT RunMode", motorExtensions.getRightMotor().getMode());
//        telemetry.addData("RIGHT Targ Pos", motorExtensions.getRightMotor().getTargetPosition());
//        telemetry.addData("RIGHT Curr Pos", motorExtensions.getRightMotor().getCurrentPosition());
//        telemetry.addData("RIGHT Dir", motorExtensions.getRightMotor().getDirection());
//        telemetry.addData("----", "----");
//        telemetry.addData("Ext Top Sensor Pressed", motorExtensions.atTopLimit());
//        telemetry.addData("Ext Bottom Sensor Pressed", motorExtensions.atBottomLimit());
//        telemetry.addData("AH Top Sensor Pressed", sensorArmHingeTop.isPressed());
//
//        telemetry.addData(" ", "  ");
//        telemetry.addData("AH Top Sensor", sensorArmHingeTop);
//        telemetry.addData("Bottom Sensor", motorExtensions.getBottomLimitSensor());
//        telemetry.addData("Front Left Power", motorFrontLeftB1.getPower());
//        telemetry.addData("Front Right Power", motorFrontRightB0.getPower());
//        telemetry.addData("Back Left Power", motorBackLeftB3.getPower());
//        telemetry.addData("Back Right Power", motorBackRightB2.getPower());
//        telemetry.addData("Front Left Position", motorFrontLeftB1.getCurrentPosition());
//        telemetry.addData("Front Right Position", motorFrontRightB0.getCurrentPosition());
//        telemetry.addData("Back Left Position", motorBackLeftB3.getCurrentPosition());
//        telemetry.addData("Back Right Position", motorBackRightB2.getCurrentPosition());
//        for (String caption : extraData.keySet()) {
//            telemetry.addData(caption, extraData.get(caption));
//        }
//        elbow.updateTelemetry(telemetry);
//        telemetry.update();
    }

//    public void updateTelemetry(String caption, Object value) {
//        telemetry.addData(caption, value);
//        telemetry.update();
//    }

    public DcMotor initBasicDcMotor(String name) {
        DcMotor motor = opMode.hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public void waitMillis(long millis) {
        long stop = System.currentTimeMillis() + millis;
        while (System.currentTimeMillis() <= stop) {
            opMode.sleep(100);
        }
    }

    public int convertInchesToTicks(DriveDirection direction, double inches) {
        int ticks = 0;
        if (direction == DriveDirection.FORWARD) {
            ticks = (int) Math.round((double) forwardTicksPerInch * inches);
        } else if (direction == DriveDirection.LEFT) {
            ticks = (int) Math.round((double) sidewaysTicksPerInch * inches);
        } else if (direction == DriveDirection.RIGHT) {
            ticks = (int) Math.round((double) sidewaysTicksPerInch * inches);
        } else if (direction == DriveDirection.BACKWARD) {
            ticks = (int) Math.round((double) forwardTicksPerInch * inches);
        } else {
            ticks = (int) Math.round((double) forwardTicksPerInch * inches);
        }
        return ticks;
    }

}
