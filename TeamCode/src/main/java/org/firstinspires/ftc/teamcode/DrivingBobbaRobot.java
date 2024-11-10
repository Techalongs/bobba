package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "BobbaTotalyCode!")
public class DrivingBobbaRobot extends LinearOpMode {

    private DcMotor motorFrontRightB0;
    private DcMotor motorFrontLeftB1;
    private DcMotor motorBackRightB2;
    private DcMotor motorBackLeftB3;
    private DcMotor motorArmHinge;
    private DcMotor motorExtension;
    private double extentsionSpeedUpLimiter = 0.5;
    private double extentsionSpeedDownLimiter = 0.6;
    private double armHingeSpeedForwordLimiter = 0.5;
    private double armHingeSpeedBackwardsLimiter = 0.6;

    private double driveSpeed = 0.5;
    private double limiter = 1.0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Motors
        motorFrontRightB0 = hardwareMap.get(DcMotor.class, "fr0");
        motorFrontLeftB1 = hardwareMap.get(DcMotor.class, "fl1");
        motorBackRightB2 = hardwareMap.get(DcMotor.class, "br2");
        motorBackLeftB3 = hardwareMap.get(DcMotor.class, "bl3");
        motorExtension = hardwareMap.get(DcMotor.class, "me");
        motorArmHinge = hardwareMap.get(DcMotor.class, "mah");

        motorBackLeftB3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRightB2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeftB1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRightB0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmHinge.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeftB3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRightB2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeftB1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRightB0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeftB1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeftB3.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeftB1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRightB0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeftB3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRightB2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmHinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Telemetry
        updateTelemetry("Status", "Initialized");
        waitForStart();

        while (opModeIsActive()) {
            this.drive(limiter, gamepad1);
            motorExtension.setPower(-gamepad2.right_stick_y * extentsionSpeedUpLimiter);
            double armPower = -gamepad2.left_stick_y;
            if (armPower > 0) motorArmHinge.setPower(armPower * armHingeSpeedForwordLimiter);
            else motorArmHinge.setPower(armPower * armHingeSpeedBackwardsLimiter);

            this.displayData();

        }
    }

    public void updateTelemetry(String caption, Object value) {
        telemetry.addData(caption, value);
        telemetry.update();
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

}

