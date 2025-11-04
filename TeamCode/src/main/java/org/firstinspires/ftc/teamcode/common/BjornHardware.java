package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Helper that owns references to the robot hardware and applies the shared defaults.
 */
public final class BjornHardware {
    public final DcMotor frontLeft;
    public final DcMotor frontRight;
    public final DcMotor backLeft;
    public final DcMotor backRight;
    public final DcMotorEx intake;
    public final DcMotorEx wheel;
    public final Servo lift;
    public final DistanceSensor frontTof;
    public final IMU imu;

    private BjornHardware(HardwareMap map) {
        frontLeft  = map.get(DcMotor.class, BjornConstants.Motors.FRONT_LEFT);
        frontRight = map.get(DcMotor.class, BjornConstants.Motors.FRONT_RIGHT);
        backLeft   = map.get(DcMotor.class, BjornConstants.Motors.BACK_LEFT);
        backRight  = map.get(DcMotor.class, BjornConstants.Motors.BACK_RIGHT);
        intake     = map.get(DcMotorEx.class, BjornConstants.Motors.INTAKE);
        wheel      = map.get(DcMotorEx.class, BjornConstants.Motors.WHEEL);
        lift       = map.get(Servo.class, BjornConstants.Servos.LIFT);
        frontTof   = map.get(DistanceSensor.class, BjornConstants.Sensors.TOF_FRONT);
        imu        = map.get(IMU.class, BjornConstants.Sensors.IMU);
    }

    /**
     * Configure and return hardware for TeleOp use.
     */
    public static BjornHardware forTeleOp(HardwareMap map) {
        BjornHardware hardware = new BjornHardware(map);
        hardware.configureDriveMotors();
        hardware.configureMechanisms();
        hardware.resetWheelEncoder();
        return hardware;
    }

    /**
     * Configure and return hardware for Autonomous use.
     * Drive motors are left untouched for Pedro follower control.
     */
    public static BjornHardware forAutonomous(HardwareMap map) {
        BjornHardware hardware = new BjornHardware(map);
        hardware.configureMechanisms();
        return hardware;
    }

    private void configureDriveMotors() {
        configureDriveMotor(frontLeft);
        configureDriveMotor(frontRight);
        configureDriveMotor(backLeft);
        configureDriveMotor(backRight);
    }

    private void configureDriveMotor(DcMotor motor) {
        motor.setDirection(BjornConstants.Motors.DRIVE_DIRECTION);
        motor.setZeroPowerBehavior(BjornConstants.Motors.DRIVE_ZERO_POWER);
    }

    private void configureMechanisms() {
        intake.setDirection(BjornConstants.Motors.INTAKE_DIRECTION);
        intake.setZeroPowerBehavior(BjornConstants.Motors.INTAKE_ZERO_POWER);
    }

    public void resetWheelEncoder() {
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
