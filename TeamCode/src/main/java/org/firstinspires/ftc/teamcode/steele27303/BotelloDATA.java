package org.firstinspires.ftc.teamcode.steele27303;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BotelloDATA")
public class BotelloDATA extends OpMode {

    // Drive motors
    private DcMotor BackL, BackR, FrontL, FrontR;

    // Mechanisms (MotorEx from FTCLib for advanced velocity data)
    private MotorEx Intake, Wheel;

    // Panels FTC-style telemetry
    private final Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // IMU for field-centric control
    private IMU imu;

    // Toggles
    private boolean isIntakeOn = false;
    private boolean isWheelOn = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;

    // Mechanism encoder math (REV-41-1600: 28 CPR at motor * gear ratio)
    private static final double MOTOR_ENCODER_CPR = 28.0; // REV HD Hex
    private static final double INTAKE_GEAR_RATIO = 20.0; // TODO set your ratio
    private static final double WHEEL_GEAR_RATIO = 1.0;  // TODO set your ratio

    private static final double INTAKE_TPR = MOTOR_ENCODER_CPR * INTAKE_GEAR_RATIO; // e.g., 560 for 20:1
    private static final double WHEEL_TPR = MOTOR_ENCODER_CPR * WHEEL_GEAR_RATIO;  // e.g., 28 for 1:1

    @Override
    public void init() {
        // Map drive motors
        BackL = hardwareMap.get(DcMotor.class, "BackL");
        BackR = hardwareMap.get(DcMotor.class, "BackR");
        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");

        // Initialize mechanism motors using FTCLib's MotorEx
        Intake = new MotorEx(hardwareMap, "Intake");
        Wheel = new MotorEx(hardwareMap, "Wheel");

        // Set ticks per revolution for FTCLib encoders to calculate revolutions
        Intake.encoder.setTicksPerRev(INTAKE_TPR);
        Wheel.encoder.setTicksPerRev(WHEEL_TPR);

        // Drive motor directions
        FrontL.setDirection(DcMotor.Direction.REVERSE);
        BackL.setDirection(DcMotor.Direction.REVERSE);
        FrontR.setDirection(DcMotor.Direction.REVERSE);
        BackR.setDirection(DcMotor.Direction.REVERSE);

        // Mechanism motor directions (using FTCLib's method)
        Intake.setInverted(true);
        Wheel.setInverted(true);

        // Brake behavior
        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // Wheel is float

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        panels.addData("Status", "FTCLib init complete (velocity graphs)");
        panels.update();
    }

    @Override
    public void loop() {
        // --- Field-Centric Drive ---
        if (gamepad1.dpad_up) imu.resetYaw();

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double fl = (rotY + rotX + rx) / denominator;
        double bl = (rotY - rotX + rx) / denominator;
        double fr = (rotY - rotX - rx) / denominator;
        double br = (rotY + rotX - rx) / denominator;

        FrontL.setPower(fl);
        FrontR.setPower(fr);
        BackL.setPower(bl);
        BackR.setPower(br);

        // --- Mechanism Toggles ---
        if (gamepad1.a && !aWasPressed) isIntakeOn = !isIntakeOn;
        if (gamepad1.b && !bWasPressed) isWheelOn = !isWheelOn;
        aWasPressed = gamepad1.a;
        bWasPressed = gamepad1.b;

        Intake.setPower(isIntakeOn ? 1.0 : 0.0);
        Wheel.setPower(isWheelOn ? 1.0 : 0.0);

        // --- Telemetry Data ---
        double intakeTps = Intake.getVelocity();
        double wheelTps = Wheel.getVelocity();
        double intakeCorrectedTps = Intake.getCorrectedVelocity();
        double wheelCorrectedTps = Wheel.getCorrectedVelocity();
        double intakeRpm = toRPM(intakeTps, INTAKE_TPR);
        double wheelRpm = toRPM(wheelTps, WHEEL_TPR);
        double intakeRevolutions = Intake.encoder.getRevolutions();
        double wheelRevolutions = Wheel.encoder.getRevolutions();
        double batteryV = getBatteryVoltage();

        // Graph raw velocity (ticks/sec)
        panels.addData("Intake_TPS", intakeTps);
        panels.addData("Wheel_TPS", wheelTps);

        // Graph corrected velocity from FTCLib
        panels.addData("Intake_Corrected_TPS", intakeCorrectedTps);
        panels.addData("Wheel_Corrected_TPS", wheelCorrectedTps);

        // Graph RPM
        panels.addData("Intake_RPM", intakeRpm);
        panels.addData("Wheel_RPM", wheelRpm);

        // Graph total revolutions
        panels.addData("Intake_Revolutions", intakeRevolutions);
        panels.addData("Wheel_Revolutions", wheelRevolutions);

        // Graph voltage and heading
        panels.addData("Battery_V", batteryV);
        panels.addData("Heading_deg", Math.toDegrees(botHeading));

        panels.update();
    }

    private double toRPM(double tps, double tpr) {
        return (tpr <= 0) ? 0.0 : (tps / tpr) * 60.0;
    }

    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.getAll(VoltageSensor.class)) {
            double v = s.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return (min == Double.POSITIVE_INFINITY) ? 0.0 : min;
    }
}
