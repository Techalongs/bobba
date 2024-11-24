package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BobbaTotalyCode!")
public class DrivingBobbaRobot extends LinearOpMode {

    private DcMotor motorFrontRightB0;
    private DcMotor motorFrontLeftB1;
    private DcMotor motorBackRightB2;
    private DcMotor motorBackLeftB3;
    private DcMotor motorArmHinge;
    private DcMotor motorExtension1;
    private DcMotor motorExtension2;
    private Servo servoClawWristHinge;
    private Servo servoClawFinger1;
    private Servo servoClawFinger2;

    private double driveSpeedLimiter = 0.5;
    private double extensionLeftUpSpeedLimiter = 0.4;
    private double extensionLeftDownSpeedLimiter = 0.4;
    private double extensionRightUpSpeedLimiter = extensionLeftUpSpeedLimiter;
    private double extensionRightDownSpeedLimiter = extensionLeftDownSpeedLimiter;
    private double armHingeSpeedForwordLimiter = 0.5;
    private double armHingeSpeedBackwardsLimiter = 0.5;
    private double clawWristSpeedLimiter = 0.7;
    private double clawFingerspeedOpenLimiter = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        // Motors
        motorFrontRightB0 = hardwareMap.get(DcMotor.class, "fr0");
        motorFrontLeftB1 = hardwareMap.get(DcMotor.class, "fl1");
        motorBackRightB2 = hardwareMap.get(DcMotor.class, "br2");
        motorBackLeftB3 = hardwareMap.get(DcMotor.class, "bl3");
        motorExtension1 = hardwareMap.get(DcMotor.class, "mer");
        motorExtension2 = hardwareMap.get(DcMotor.class, "mel");
        motorArmHinge = hardwareMap.get(DcMotor.class, "mah");
        servoClawWristHinge = hardwareMap.get(Servo.class, "scwh");
        servoClawFinger1 = hardwareMap.get(Servo.class,"scf1");
        servoClawFinger2 = hardwareMap.get(Servo.class,"scf2");

        motorBackLeftB3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRightB2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeftB1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRightB0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmHinge.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeftB3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRightB2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeftB1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRightB0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExtension1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExtension2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmHinge.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeftB1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeftB3.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtension1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtension2.setDirection(DcMotorSimple.Direction.REVERSE);
        servoClawFinger1.setDirection(Servo.Direction.REVERSE);

        motorFrontLeftB1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRightB0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeftB3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRightB2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmHinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Telemetry
        updateTelemetry("Status", "Initialized");
        waitForStart();

        while (opModeIsActive()) {
            // Wheels
            this.drive(driveSpeedLimiter, gamepad1);

            // Extensions
            double extensionPower = -gamepad2.right_stick_y;
            if (extensionPower > 0) {
                motorExtension1.setPower(extensionPower * extensionLeftUpSpeedLimiter);
                motorExtension2.setPower(extensionPower * extensionRightUpSpeedLimiter);
            } else {
                motorExtension1.setPower(extensionPower * extensionLeftDownSpeedLimiter);
                motorExtension2.setPower(extensionPower * extensionRightDownSpeedLimiter);
            }

            // Arm
            double armHingePower = -gamepad2.left_stick_y;
            if (armHingePower > 0) {
                motorArmHinge.setPower(armHingePower * armHingeSpeedForwordLimiter);
            } else {
                motorArmHinge.setPower(armHingePower * armHingeSpeedBackwardsLimiter);
            }

            double clawButtonRT = gamepad2.right_trigger;
            if (clawButtonRT > 0) {
                closeClaws();
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

    public void displayData() {
        telemetry.addData("Status", "Running");
        telemetry.addData("Right Trigger", gamepad2.right_trigger);
        telemetry.addData("claw get position", servoClawFinger1.getPosition());
        telemetry.addData("claw get position2", servoClawFinger2.getPosition());

        telemetry.addData("Front Left Power", motorFrontLeftB1.getPower());
        telemetry.addData("Front Right Power", motorFrontRightB0.getPower());
        telemetry.addData("Back Left Power", motorBackLeftB3.getPower());
        telemetry.addData("Back Right Power", motorBackRightB2.getPower());

        telemetry.addData("Front Left Position", motorFrontLeftB1.getCurrentPosition());
        telemetry.addData("Front Right Position", motorFrontRightB0.getCurrentPosition());
        telemetry.addData("Back Left Position", motorBackLeftB3.getCurrentPosition());
        telemetry.addData("Back Right Position", motorBackRightB2.getCurrentPosition());

//        for (String caption : extraData.keySet()) {
//            telemetry.addData(caption, extraData.get(caption));
//        }

        telemetry.update();
    }

    public void updateTelemetry(String caption, Object value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }

}

