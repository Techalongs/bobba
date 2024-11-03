package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
