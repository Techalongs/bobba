package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "BabbaTotalyCode!")
public class DrivingBobbaRobot extends LinearOpMode {

    private DcMotor frontRightMotorB0;
    private DcMotor frontLeftMotorB1;
    private DcMotor backRightMotorB2;
    private DcMotor backLeftMotorB3;

    @Override
    public void runOpMode() throws InterruptedException {
        // Motors
        frontRightMotorB0 = hardwareMap.get(DcMotor.class, "fr0");
        frontLeftMotorB1 = hardwareMap.get(DcMotor.class, "fl1");
        backRightMotorB2 = hardwareMap.get(DcMotor.class, "br2");
        backLeftMotorB3 = hardwareMap.get(DcMotor.class, "bl3");

        backLeftMotorB3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotorB2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotorB1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotorB0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftMotorB3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotorB2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotorB1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotorB0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotorB1.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotorB3.setDirection(DcMotorSimple.Direction.REVERSE);

        // Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            frontRightMotorB0.setPower(-gamepad1.left_stick_y-gamepad1.right_stick_x);
            frontLeftMotorB1.setPower(-gamepad1.left_stick_y+gamepad1.right_stick_x);
            backRightMotorB2.setPower(-gamepad1.left_stick_y-gamepad1.right_stick_x);
            backLeftMotorB3.setPower(-gamepad1.left_stick_y+gamepad1.right_stick_x);
        }
    }

}
