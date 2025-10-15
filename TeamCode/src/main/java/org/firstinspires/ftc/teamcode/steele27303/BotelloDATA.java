package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp(name = "BotelloDATA")
public class BotelloDATA extends OpMode {

    // Drive motors (DcMotor — no velocity reads on drive)
    private DcMotor BackL, BackR, FrontL, FrontR;

    // Mechanisms (DcMotorEx so we can read velocity)
    private DcMotorEx Intake, Wheel;

    // Panels FTC-style telemetry
    private final Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // IMU for field-centric control
    private IMU imu;

    // Toggles
    private boolean isIntakeOn = false;
    private boolean isWheelOn  = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;

    // Mechanism encoder math (REV-41-1600: 28 CPR at motor * gear ratio)
    private static final double MOTOR_ENCODER_CPR = 28.0; // REV HD Hex
    private static final double INTAKE_GEAR_RATIO = 20.0; // TODO set your ratio
    private static final double WHEEL_GEAR_RATIO  = 1.0;  // TODO set your ratio

    private static final double INTAKE_TPR = MOTOR_ENCODER_CPR * INTAKE_GEAR_RATIO; // e.g., 560 for 20:1
    private static final double WHEEL_TPR  = MOTOR_ENCODER_CPR * WHEEL_GEAR_RATIO;  // e.g., 28 for 1:1

    @Override
    public void init() {
        // Map drive (DcMotor)
        BackL  = hardwareMap.get(DcMotor.class,   "BackL");
        BackR  = hardwareMap.get(DcMotor.class,   "BackR");
        FrontL = hardwareMap.get(DcMotor.class,   "FrontL");
        FrontR = hardwareMap.get(DcMotor.class,   "FrontR");

        // Map mechanisms (DcMotorEx for velocity)
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel  = hardwareMap.get(DcMotorEx.class, "Wheel");

        // Directions (match your latest program)
        FrontL.setDirection(DcMotor.Direction.REVERSE);
        BackL.setDirection(DcMotor.Direction.REVERSE);
        FrontR.setDirection(DcMotor.Direction.REVERSE);
        BackR.setDirection(DcMotor.Direction.REVERSE);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake behavior
        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        FrontL.setZeroPowerBehavior(brake);
        FrontR.setZeroPowerBehavior(brake);
        BackL.setZeroPowerBehavior(brake);
        BackR.setZeroPowerBehavior(brake);
        Intake.setZeroPowerBehavior(brake);
        // Wheel left as float per your original comment

        // IMU setup (RIGHT / UP as you specified)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        panels.addData("Status", "Field‑Centric init (velocity graphs)");
        panels.update();
    }

    @Override
    public void loop() {
        // Reset yaw with D‑Pad Up (your mapping)
        if (gamepad1.dpad_up) imu.resetYaw();

        // Sticks
        double y  = -gamepad1.left_stick_y;  // forward/back (FTC Y is inverted)
        double x  =  gamepad1.left_stick_x;  // strafe
        double rx =  gamepad1.right_stick_x; // rotate

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field-centric transform
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1; // counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double fl = (rotY + rotX + rx) / denominator;
        double bl = (rotY - rotX + rx) / denominator;
        double fr = (rotY - rotX - rx) / denominator;
        double br = (rotY + rotX - rx) / denominator;

        FrontL.setPower(fl);  FrontR.setPower(fr);
        BackL.setPower(bl);   BackR.setPower(br);

        // Toggles (unchanged)
        if (gamepad1.a && !aWasPressed) isIntakeOn = !isIntakeOn;
        if (gamepad1.b && !bWasPressed) isWheelOn  = !isWheelOn;
        aWasPressed = gamepad1.a;
        bWasPressed = gamepad1.b;

        Intake.setPower(isIntakeOn ? 1.0 : 0.0);
        Wheel.setPower(isWheelOn ? 1.0 : 0.0);

        // --- Velocity + RPM for mechanisms ---
        double intakeTps = safeVel(Intake);                    // ticks/sec
        double wheelTps  = safeVel(Wheel);
        double intakeRpm = toRPM(intakeTps, INTAKE_TPR);       // RPM
        double wheelRpm  = toRPM(wheelTps,  WHEEL_TPR);
        double batteryV  = getBatteryVoltage();

        // Graph raw velocity (ticks/sec)
        panels.addData("Intake_TPS", intakeTps);
        panels.addData("Wheel_TPS",  wheelTps);
        // Graph RPM
        panels.addData("Intake_RPM", intakeRpm);
        panels.addData("Wheel_RPM",  wheelRpm);
        // Graph voltage and heading
        panels.addData("Battery_V",  batteryV);
        panels.addData("Heading_deg", Math.toDegrees(botHeading));

        // (Optional) power traces to correlate with RPM/voltage
        panels.addData("FL_power", fl);
        panels.addData("FR_power", fr);
        panels.addData("BL_power", bl);
        panels.addData("BR_power", br);

        panels.update();
    }

    private static double toRPM(double tps, double tpr) { return (tpr <= 0) ? 0.0 : (tps / tpr) * 60.0; }
    private static double safeVel(DcMotorEx m){ try { return m.getVelocity(); } catch(Exception e){ return 0.0; } }

    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.getAll(VoltageSensor.class)) {
            double v = s.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return (min == Double.POSITIVE_INFINITY) ? 0.0 : min;
    }
}
