package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BobbaTotalyCode!")
public class DrivingBobbaRobot extends LinearOpMode {

    private DcMotor motorFrontRightB0;
    private DcMotor motorFrontLeftB1;
    private DcMotor motorBackRightB2;
    private DcMotor motorBackLeftB3;

    @Override
    public void runOpMode() throws InterruptedException {
        // Motors
        motorFrontRightB0 = hardwareMap.get(DcMotor.class, "fr0");
        motorFrontLeftB1 = hardwareMap.get(DcMotor.class, "fl1");
        motorBackRightB2 = hardwareMap.get(DcMotor.class, "br2");
        motorBackLeftB3 = hardwareMap.get(DcMotor.class, "bl3");

        motorBackLeftB3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRightB2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeftB1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRightB0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeftB3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRightB2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeftB1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRightB0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeftB1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeftB3.setDirection(DcMotorSimple.Direction.REVERSE);

        // Telemetry
        updateTelemetry("Status", "Initialized");
        waitForStart();

        while (opModeIsActive()) {
            motorFrontRightB0.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
            motorFrontLeftB1.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            motorBackRightB2.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
            motorBackLeftB3.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
        }
    }

    public void updateTelemetry(String caption, Object value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }

}
