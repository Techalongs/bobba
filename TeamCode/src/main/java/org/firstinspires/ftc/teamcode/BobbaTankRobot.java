package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class BobbaTankRobot extends LinearOpMode {

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
        telemetry.addData("status", "Initialized");
        telemetry.update();
        waitForStart();
    }

}
