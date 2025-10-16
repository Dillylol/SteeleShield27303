package org.firstinspires.ftc.teamcode.steele27303;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BotelloDATA")
public class BotelloDATA extends OpMode {

    // Drive motors (stay DcMotor – no velocity reads)
    private DcMotor BackL, BackR, FrontL, FrontR;

    // Mechanisms (use DcMotorEx so we can read velocity)
    private DcMotorEx Intake, Wheel;

    // Panels telemetry
    private final Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // IMU for field-centric
    private IMU imu;

    // Toggles
    private boolean isIntakeOn = false;
    private boolean isWheelOn  = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;

    // REV-41-1600 (28 CPR) * gear ratio => ticks per output revolution
    private static final double MOTOR_ENCODER_CPR = 28.0;
    private static final double INTAKE_GEAR_RATIO = 20.0; // TODO: set yours
    private static final double WHEEL_GEAR_RATIO  = 1.0;  // TODO: set yours

    private static final double INTAKE_TPR = MOTOR_ENCODER_CPR * INTAKE_GEAR_RATIO; // 560 for 20:1
    private static final double WHEEL_TPR  = MOTOR_ENCODER_CPR * WHEEL_GEAR_RATIO;  // 28 for 1:1

    @Override
    public void init() {
        // Map drive
        BackL  = hardwareMap.get(DcMotor.class, "BackL");
        BackR  = hardwareMap.get(DcMotor.class, "BackR");
        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");

        // Map mechanisms as DcMotorEx
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel  = hardwareMap.get(DcMotorEx.class, "Wheel");

        // Directions same as before
        FrontL.setDirection(DcMotor.Direction.REVERSE);
        BackL.setDirection(DcMotor.Direction.REVERSE);
        FrontR.setDirection(DcMotor.Direction.REVERSE);
        BackR.setDirection(DcMotor.Direction.REVERSE);

        // If your mechanisms need to spin the other way, flip here:
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Wheel.setDirection(DcMotor.Direction.REVERSE);

        // Brake behavior
        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        FrontL.setZeroPowerBehavior(brake);
        FrontR.setZeroPowerBehavior(brake);
        BackL.setZeroPowerBehavior(brake);
        BackR.setZeroPowerBehavior(brake);
        Intake.setZeroPowerBehavior(brake);
        // Wheel left FLOAT if you prefer; otherwise:
        // Wheel.setZeroPowerBehavior(brake);

        // IMU setup (use your RIGHT/UP mount)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        panels.addData("Status", "Init complete (SDK motors)");
        panels.update();
    }

    @Override
    public void loop() {
        // Reset yaw with D-Pad Up
        if (gamepad1.dpad_up) imu.resetYaw();

        // Field-centric drive
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x * 1.1; // compensate strafing
        double rx =  gamepad1.right_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double fl = (rotY + rotX + rx) / denom;
        double bl = (rotY - rotX + rx) / denom;
        double fr = (rotY - rotX - rx) / denom;
        double br = (rotY + rotX - rx) / denom;

        FrontL.setPower(fl);  FrontR.setPower(fr);
        BackL.setPower(bl);   BackR.setPower(br);

        // Toggles
        if (gamepad1.a && !aWasPressed) isIntakeOn = !isIntakeOn;
        if (gamepad1.b && !bWasPressed) isWheelOn  = !isWheelOn;
        aWasPressed = gamepad1.a;
        bWasPressed = gamepad1.b;

        Intake.setPower(isIntakeOn ? 1.0 : 0.0);
        Wheel.setPower(isWheelOn ? 1.0 : 0.0);

        // Telemetry/graphs (TPS + RPM + volts)
        double intakeTps = safeVel(Intake);                 // ticks/sec
        double wheelTps  = safeVel(Wheel);
        double intakeRpm = toRPM(intakeTps, INTAKE_TPR);    // RPM
        double wheelRpm  = toRPM(wheelTps,  WHEEL_TPR);
        double batteryV  = getBatteryVoltage();

        panels.addData("Intake_TPS", intakeTps);
        panels.addData("Wheel_TPS",  wheelTps);
        panels.addData("Intake_RPM", intakeRpm);
        panels.addData("Wheel_RPM",  wheelRpm);
        panels.addData("Battery_V",  batteryV);
        panels.addData("Heading_deg", Math.toDegrees(heading));
        panels.addData("FL_power", fl);
        panels.addData("FR_power", fr);
        panels.addData("BL_power", bl);
        panels.addData("BR_power", br);

        panels.update();
    }

    private static double safeVel(DcMotorEx m) {
        try { return m.getVelocity(); } catch (Exception e) { return 0.0; }
    }

    private static double toRPM(double tps, double tpr) {
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
