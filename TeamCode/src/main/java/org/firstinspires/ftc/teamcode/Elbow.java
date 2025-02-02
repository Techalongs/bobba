package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Some of this is copied from the GoBilda example code for DC motors.
 */
public class Elbow {

    private OpMode opMode;
    private Gamepad gamepad;
    public DcMotorEx motor = null;
    private int velocity = 200;
    private double power = 0.75;
    private boolean useVelocity = true;

    final double ELBOW_DEFAULT_POSITION = 0;
    final double ELBOW_UP_POSITION  = 35;
    final double ELBOW_PICKUP_POSITION  = 75;

    double targetArmPositionDegrees = ELBOW_DEFAULT_POSITION;
    int targetArmPositionTicks = (int) ELBOW_DEFAULT_POSITION;

    public Elbow(OpMode opMode, DcMotorEx elbowMotor) {
        this.opMode = opMode;
        this.gamepad = opMode.gamepad2;
        this.motor = elbowMotor;
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setCurrentAlert(5, CurrentUnit.AMPS);

        motor.setTargetPosition(RobotUtils.degreesToTicksRevCoreHex(ELBOW_DEFAULT_POSITION));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPositionTolerance(20);
    }

    public void update() {
        if(gamepad.right_bumper){
            this.setTargetArmPositionDegrees(ELBOW_PICKUP_POSITION);
        } else if (gamepad.left_bumper){
            this.setTargetArmPositionDegrees(ELBOW_DEFAULT_POSITION);
        } else if (gamepad.dpad_left) {
            this.setTargetArmPositionDegrees(ELBOW_DEFAULT_POSITION);
        }
        boolean moveArm = false;

        if (targetArmPositionDegrees >= 50 && motor.getCurrentPosition() <= 50) {
            if (!motor.isMotorEnabled()) {
                motor.setMotorEnable();
            }
            motor.setTargetPosition(targetArmPositionTicks);
            moveArm = true;
            if (useVelocity) {
                motor.setVelocity(velocity);
            } else {
                motor.setPower(power);
            }
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (targetArmPositionDegrees < 50 && motor.getCurrentPosition() >= 10) {
            if (!motor.isMotorEnabled()) {
                motor.setMotorEnable();
            }
            motor.setTargetPosition(targetArmPositionTicks);
            moveArm = true;
            if (useVelocity) {
                motor.setVelocity(velocity);
            } else {
                motor.setPower(power);
            }
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else {
            moveArm = false;
            motor.setPower(0);
            motor.setVelocity(0);
            if (motor.isMotorEnabled()) {
                motor.setMotorDisable();
            }

        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        if (useVelocity) {
            telemetry.addData("E Use Velocity: ", velocity);
        } else {
            telemetry.addData("E Use Power: ", power);
        }
        telemetry.addData("E Code Target (d): ", targetArmPositionDegrees);
        telemetry.addData("E Code Target (t): ", targetArmPositionTicks);
        telemetry.addData("E Motor Target (t): ", motor.getTargetPosition());
        telemetry.addData("E Motor Current (t): ", motor.getCurrentPosition());
        telemetry.addData("E Motor Mode: ", motor.getMode());
        telemetry.addData("E Motor Power: ", motor.getPower());
        telemetry.addData("E Motor Velocity: ", motor.getVelocity());
        telemetry.addData("E Motor Enabled: ", motor.isMotorEnabled());
        telemetry.addData("E Motor Target Pos Allow: ", motor.getTargetPositionTolerance());
    }

    public void setTargetArmPositionDegrees(double degrees) {
        targetArmPositionDegrees = degrees;
        targetArmPositionTicks = RobotUtils.degreesToTicksRevCoreHex(degrees);
    }

}
