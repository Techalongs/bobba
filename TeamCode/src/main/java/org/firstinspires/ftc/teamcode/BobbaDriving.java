package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Bobba Driving (Do Not Use)")
public class BobbaDriving extends LinearOpMode {

    private IMU imu;
    private TotallyRobot robot;
    private Gamepad driverGamepad;
    private Gamepad operatorGamepad;

    private Elbow elbow;
    private Extension motorExtensions;
    private Servo servoClawWristHinge;
    private Servo servoClawFinger1;
    private Servo servoClawFinger2;
//    private TouchSensor sensorArmHingeTop;

    private double driveSpeedLimiter = 0.5;
    private double extensionSpeedLimiter = 0.5;
    private double armHingeSpeedForwordLimiter = 0.5;
    private double armHingeSpeedBackwardsLimiter = 0.5;
    private double clawWristSpeedLimiter = 0.7;
    private double clawFingerSpeedOpenLimiter = 0.8;

    // Wrist variables for testing out wrist stuff
    private double wristMinPosition = 0.0;
    private double wristMaxPosition = 0.5;
    private double wristTicksPerCycle = 0.01;
    private boolean useFieldCentricDriving = false;

    @Override
    public void runOpMode() throws InterruptedException {

        TotallyRobot robot = new TotallyRobot();
        updateTelemetry("Status", "Initializing");
        robot.settingBotUp(this);

        updateTelemetry("Status", "Waiting for Start");
        waitForStart();
        resetRuntime();
        updateTelemetry("Status", "Started");

        while (opModeIsActive()) {

            if (gamepad1.options && gamepad1.left_bumper && !useFieldCentricDriving) {
                useFieldCentricDriving = true;
                telemetry.speak("Field Centric Power Mode Initiated!");
                imu.resetYaw();
            } else if (gamepad1.options && gamepad1.right_bumper && useFieldCentricDriving) {
                useFieldCentricDriving = false;
                telemetry.speak("Robot Centric Driving");
            } else if (gamepad1.options) {
                imu.resetYaw();
            }

            // Driving
            if (useFieldCentricDriving) {
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                robot.driveFieldCentric(gamepad1, botHeading);
            } else {
                robot.driveRobotCentric(gamepad1);
            }

            // Elbow
            elbow.update();

            // Extension
            double extensionPower = -gamepad2.right_stick_y;
            motorExtensions.move(extensionPower);

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
            double clawRightTrigger = gamepad2.right_trigger;
            if (clawRightTrigger > 0) {
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

    public void displayData() {
        telemetry.addData("Status", "Running");
//        telemetry.addData("F1 Pos", servoClawFinger1.getPosition());
//        telemetry.addData("F2 Pos", servoClawFinger2.getPosition());
        telemetry.addData("Ext Status", motorExtensions.getStatus());
        telemetry.addData("Ext Status Detail", motorExtensions.getStatusDetail());
        telemetry.addData("Ext Position", motorExtensions.getCurrentPosition());
        telemetry.addData("Ext RunMode", motorExtensions.getCurrentRunMode());
        telemetry.addData("LEFT Power", motorExtensions.getLeftMotor().getPower());
        telemetry.addData("LEFT RunMode", motorExtensions.getLeftMotor().getMode());
        telemetry.addData("LEFT Targ Pos", motorExtensions.getLeftMotor().getTargetPosition());
        telemetry.addData("LEFT Curr Pos", motorExtensions.getLeftMotor().getCurrentPosition());
        telemetry.addData("LEFT Dir", motorExtensions.getLeftMotor().getDirection());
        telemetry.addData("RIGHT Power", motorExtensions.getRightMotor().getPower());
        telemetry.addData("RIGHT RunMode", motorExtensions.getRightMotor().getMode());
        telemetry.addData("RIGHT Targ Pos", motorExtensions.getRightMotor().getTargetPosition());
        telemetry.addData("RIGHT Curr Pos", motorExtensions.getRightMotor().getCurrentPosition());
        telemetry.addData("RIGHT Dir", motorExtensions.getRightMotor().getDirection());
//        telemetry.addData("----", "----");
        telemetry.addData("Ext Top Sensor Pressed", motorExtensions.atTopLimit());
        telemetry.addData("Ext Bottom Sensor Pressed", motorExtensions.atBottomLimit());
//        telemetry.addData("AH Top Sensor Pressed", sensorArmHingeTop.isPressed());

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

//        } wrist 70 claw
        elbow.updateTelemetry(telemetry);

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

