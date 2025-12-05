package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

/**
 * Helper that owns references to the robot hardware and applies the shared
 * defaults.
 */
public final class BjornHardware {
    public final DcMotor frontLeft;
    public final DcMotor frontRight;
    public final DcMotor backLeft;
    public final DcMotor backRight;
    public final DcMotorEx intake;
    public final DcMotorEx wheel;
    public final DcMotorEx wheel2;
    public final Servo lift;
    public final Servo lift2;
    public final DistanceSensor frontTof;
    public final IMU imu;
    private final VoltageSensor batterySensor;

    private BjornHardware(HardwareMap map) {
        frontLeft = map.get(DcMotor.class, BjornConstants.Motors.FRONT_LEFT);
        frontRight = map.get(DcMotor.class, BjornConstants.Motors.FRONT_RIGHT);
        backLeft = map.get(DcMotor.class, BjornConstants.Motors.BACK_LEFT);
        backRight = map.get(DcMotor.class, BjornConstants.Motors.BACK_RIGHT);
        intake = map.get(DcMotorEx.class, BjornConstants.Motors.INTAKE);
        wheel = map.get(DcMotorEx.class, BjornConstants.Motors.WHEEL);
        wheel2 = map.get(DcMotorEx.class, BjornConstants.Motors.WHEEL2);
        lift = map.get(Servo.class, BjornConstants.Servos.LIFT);
        lift2 = map.get(Servo.class, BjornConstants.Servos.LIFT2);
        frontTof = map.get(DistanceSensor.class, BjornConstants.Sensors.TOF_FRONT);
        imu = map.get(IMU.class, BjornConstants.Sensors.IMU);
        batterySensor = firstVoltageSensor(map);
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
        wheel2.setDirection(BjornConstants.Motors.WHEEL2_DIRECTION);
        wheel.setDirection(BjornConstants.Motors.WHEEL_DIRECTION);
        if (lift2 != null)
            lift2.setDirection(Servo.Direction.REVERSE); // Typical dual servo setup
    }

    public void resetWheelEncoder() {
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLiftPosition(double position) {
        if (lift != null)
            lift.setPosition(position + BjornConstants.Servos.LIFT_OFFSET);
        if (lift2 != null)
            lift2.setPosition(position + BjornConstants.Servos.LIFT2_OFFSET);
    }

    public double getBatteryVoltage() {
        if (batterySensor == null) {
            return BjornConstants.Power.NOMINAL_BATT_V;
        }
        try {
            return batterySensor.getVoltage();
        } catch (Exception ignored) {
            return BjornConstants.Power.NOMINAL_BATT_V;
        }
    }

    public VoltageSensor getVoltageSensor() {
        return batterySensor;
    }

    /**
     * Scales drive power based on battery voltage to prevent brownouts.
     * Logic derived from JULESData5.csv showing drops to ~10.9V.
     */
    public double getOptimizedDrivePower(double power) {
        double voltage = getBatteryVoltage();
        // Simple linear scaling: 100% at 12V+, linear drop to 50% at 10V
        double minV = 10.0;
        double maxV = 12.0;
        double scale = (voltage - minV) / (maxV - minV);
        scale = Math.max(0.5, Math.min(1.0, scale)); // Clamp between 0.5 and 1.0
        return power * scale;
    }

    private static VoltageSensor firstVoltageSensor(HardwareMap map) {
        List<VoltageSensor> sensors = map.getAll(VoltageSensor.class);
        return sensors.isEmpty() ? null : sensors.get(0);
    }
}
