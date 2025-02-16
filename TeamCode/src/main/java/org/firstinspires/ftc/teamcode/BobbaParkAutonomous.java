package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TotallyParkAuto")
public class BobbaParkAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        TotallyRobot robot = new TotallyRobot();
        updateTelemetry("Status", "Initializing");
        robot.settingBotUp(this);

        updateTelemetry("Status", "Waiting for Start");
        waitForStart();
        this.waitMillis(1000);

        updateTelemetry("Status", "Started");

        if (opModeIsActive()) {
            robot.driveInches(DriveDirection.FORWARD, 3, 0.3);
            robot.driveInches(DriveDirection.RIGHT, 24, 0.3);
        }
    }

    public void waitMillis(long millis) {
        long stop = System.currentTimeMillis() + millis;
        while (System.currentTimeMillis() < stop) {
            this.sleep(100);
        }
    }

    public void updateTelemetry(String caption, Object value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }

}
