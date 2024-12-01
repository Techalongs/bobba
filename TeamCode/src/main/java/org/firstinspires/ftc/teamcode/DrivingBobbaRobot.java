package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "BobbaTotalyCode!")
public class DrivingBobbaRobot extends LinearOpMode {

    private DcMotor motorFrontRightB0;
    private DcMotor motorFrontLeftB1;
    private DcMotor motorBackRightB2;
    private DcMotor motorBackLeftB3;
    private DcMotor motorArmHinge;
    private Extension motorExtensions;
    private Servo servoClawWristHinge;
    private Servo servoClawFinger1;
    private Servo servoClawFinger2;

    private double driveSpeedLimiter = 0.5;
    private double extensionSpeedLimiter = 0.1;
    private double armHingeSpeedForwordLimiter = 0.5;
    private double armHingeSpeedBackwardsLimiter = 0.5;
    private double clawWristSpeedLimiter = 0.7;
    private double clawFingerSpeedOpenLimiter = 0.8;
    private double runtimeSeconds = 0.0;
    private double currentBatteryVoltage = 0.0;
    private double alertMinBatteryVoltage = 12.0;

    // Wrist variables for testing out wrist stuff
    private double wristMinPosition = 0.0;
    private double wristMaxPosition = 0.5;
    private double wristTicksPerCycle = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        // Wheels
        motorFrontRightB0 = initBasicDcMotor("fr0");
        motorFrontLeftB1 = initBasicDcMotor("fl1");
        motorBackRightB2 = initBasicDcMotor("br2");
        motorBackLeftB3 = initBasicDcMotor("bl3");
        motorFrontLeftB1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeftB3.setDirection(DcMotorSimple.Direction.REVERSE);

        // Elbow
        motorArmHinge = initBasicDcMotor("mah");

        // Wrist
        servoClawWristHinge = hardwareMap.get(Servo.class, "scwh");
        servoClawWristHinge.setDirection(Servo.Direction.REVERSE);

        // Claw
        servoClawFinger1 = hardwareMap.get(Servo.class,"scf1");
        servoClawFinger2 = hardwareMap.get(Servo.class,"scf2");
        servoClawFinger1.setDirection(Servo.Direction.REVERSE);

        // TODO: This doesn't seem to be working
        TouchSensor sensorExtensionBottom = hardwareMap.get(TouchSensor.class, "sebl");
        motorExtensions = new Extension(
                initBasicDcMotor("mer"),
                initBasicDcMotor("mel"),
                null,
                sensorExtensionBottom
        );
        motorExtensions.getLeftMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtensions.getRightMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtensions.setSpeedLimiter(extensionSpeedLimiter);
        servoClawWristHinge.setPosition(wristMaxPosition);

        // Starting
        waitForStart();
        resetRuntime();
        updateBatteryVoltage();
        updateTelemetry("Status", "Initialized");

        while (opModeIsActive()) {
            runtimeSeconds = getRuntime();
            updateBatteryVoltage();

            // Wheels
            this.drive(driveSpeedLimiter, gamepad1);

            // Extension
            double extensionPower = 0.0;
            if (gamepad2.dpad_up) {
                extensionPower = 1.0;
            } else if (gamepad2.dpad_down) {
                extensionPower = -1.0;
            }
            motorExtensions.move(extensionPower);

            // Elbow
            double armHingePower = -gamepad2.right_stick_y;
            if (armHingePower > 0) {
                motorArmHinge.setPower(armHingePower * armHingeSpeedForwordLimiter);
            } else {
                motorArmHinge.setPower(armHingePower * armHingeSpeedBackwardsLimiter);
            }

            // Wrist
            double wristPositionControl = gamepad2.left_stick_y;
            double wristGetPosition = servoClawWristHinge.getPosition();
            if (wristPositionControl > 0.0) {
                double wristNewPosition = (wristTicksPerCycle * wristPositionControl) + wristGetPosition;
                if (wristNewPosition > wristMaxPosition) {
                    wristNewPosition = wristMaxPosition;
                }
                servoClawWristHinge.setPosition(wristNewPosition);
            } else if (wristPositionControl < 0) {
                double wristNewPosition = (wristTicksPerCycle * wristPositionControl) + wristGetPosition;
                if (wristNewPosition < wristMinPosition) {
                    wristNewPosition = wristMinPosition;
                }
                servoClawWristHinge.setPosition(wristNewPosition);
            }

            // Claw
            double clawButtonRT = gamepad2.right_trigger;
            if (clawButtonRT > 0) {
                closeClaws();
            }

            double clawLeftTrigger = gamepad2.left_trigger;
            if (clawLeftTrigger > 0) {
                openClaws();
            }

            // Display
            this.displayData();
        }
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

    public void drive(double limiter, Gamepad gamepad) {
        float flPower = (-gamepad.left_stick_y + gamepad.right_stick_x) + gamepad.left_stick_x;
        float frPower = (-gamepad.left_stick_y - gamepad.right_stick_x) - gamepad.left_stick_x;
        float blPower = (-gamepad.left_stick_y + gamepad.right_stick_x) - gamepad.left_stick_x;
        float brPower = (-gamepad.left_stick_y - gamepad.right_stick_x) + gamepad.left_stick_x;
        motorFrontLeftB1.setPower(flPower * limiter);
        motorFrontRightB0.setPower(frPower * limiter);
        motorBackLeftB3.setPower(blPower * limiter);
        motorBackRightB2.setPower(brPower * limiter);
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
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
    private double updateBatteryVoltage() {
        if (runtimeSeconds % 60 == 0) {
            currentBatteryVoltage = getBatteryVoltage();
        }
        return currentBatteryVoltage;
    }

    public void displayData() {
        telemetry.addData("Status", "Running");
        if (currentBatteryVoltage < alertMinBatteryVoltage) {
            telemetry.addData("Voltage", "%.2f", currentBatteryVoltage);
            telemetry.addData("VOLTAGE ALERT", "VOLTAGE ALERT");
        }
        telemetry.addData("Extension Position", motorExtensions.getCurrentPosition());
        telemetry.addData("Top Sensor Pressed", motorExtensions.atBottomLimit());
        telemetry.addData("Bottom Sensor Pressed", motorExtensions.atBottomLimit());

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

        telemetry.update();
    }

    public void updateTelemetry(String caption, Object value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }

    public DcMotor initBasicDcMotor(String name) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

}

