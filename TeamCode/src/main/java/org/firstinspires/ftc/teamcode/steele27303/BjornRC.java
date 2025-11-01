package org.firstinspires.ftc.teamcode.steele27303;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.Arrays;

@TeleOp(name = "BjornRC (DualPad)")
public class BjornRC extends OpMode {

    // Drive motors (DcMotor – no velocity reads)
    private DcMotor BackL, BackR, FrontL, FrontR;

    // Mechanisms (DcMotorEx for velocity)
    private DcMotorEx Intake, Wheel;

    // Lift servo (no motion in init)
    private Servo Lift;

    // Panels telemetry
    private final Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // IMU (initialized but NOT used for drive math here)
    private IMU imu;

    // ToF sensor
    private DistanceSensor tofFront;

    // ===== Dual-Gamepad Redundancy =====
    // Gamepad2 always has full control of Intake & Wheel (redundancy).
    // Gamepad1 D-Pad LEFT toggles Gamepad2 Full-Drive mode (master switch).
    private boolean g2DriveMaster = false; // when true, G2 drives robot-centric
    private boolean g1DpadLeftPrev = false; // edge detect

    // Toggles
    private boolean isWheelOn  = false;   // B toggles ramped (dynamic) mode
    private boolean g1BPrev = false, g2BPrev = false;

    // Button edges for trims / scan / idle
    private boolean g1RbPrev=false, g1LbPrev=false, g1DpadUpPrev=false;
    private boolean g2RbPrev=false, g2LbPrev=false, g2DpadUpPrev=false;

    // Motor/encoder constants
    private static final double MOTOR_ENCODER_CPR = 28.0;
    private static final double INTAKE_GEAR_RATIO = 20.0; // TODO set yours
    private static final double WHEEL_GEAR_RATIO  = 1.0;  // TODO set yours
    private static final double INTAKE_TPR = MOTOR_ENCODER_CPR * INTAKE_GEAR_RATIO; // 560 for 20:1
    private static final double WHEEL_TPR  = MOTOR_ENCODER_CPR * WHEEL_GEAR_RATIO;  // 28 for 1:1

    // Servo positions
    private static final double LIFT_LOWERED_POS = 0.10;
    private static final double LIFT_RAISED_POS  = 0.65;

    // Dynamic lift readiness
    private static final double READY_TOL_RPM = 250.0;       // ± band
    private static final double READY_MIN_TARGET_RPM = 2000.0;
    private static final long   READY_HOLD_MS = 1000;         // dwell

    private boolean liftIsRaised = false;
    private long    readyBandEnterMs = -1;

    // RPM control
    private double targetWheelRPM = 2825.0;      // dynamic target
    private static final double STEP_SMALL = 25; // RB/LB trims only
    private static final double RPM_MIN    = 0.0;
    private static final double RPM_MAX    = 3800.0;

    // Always-on idle spin (keeps momentum)
    private boolean idleSpinEnabled = false;     // toggled by D-Pad Up (g1 or g2)
    private static final double IDLE_RPM = 2000.0; // choose your idle speed

    // Linear fit RPM ≈ m*Dft + b
    private static final double M_RPM_PER_FT = 116.4042383594456;
    private static final double B_RPM_OFFSET = 2084.2966941424975;

    // ToF scan state
    private static final int SCAN_SAMPLES = 10;
    private int scanLeft = 0;
    private final double[] scanBuf = new double[SCAN_SAMPLES];
    private double lastScanFt = Double.NaN; // for telemetry
    private static final double SENSOR_TO_CANNON_OFFSET_FT = 0.0; // front-mounted

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);

        BackL  = hardwareMap.get(DcMotor.class, "lr");
        BackR  = hardwareMap.get(DcMotor.class, "rr");
        FrontL = hardwareMap.get(DcMotor.class, "lf");
        FrontR = hardwareMap.get(DcMotor.class, "rf");

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel  = hardwareMap.get(DcMotorEx.class, "Wheel");

        Lift   = hardwareMap.get(Servo.class, "Lift");
        tofFront = hardwareMap.get(DistanceSensor.class, "TOF");

        FrontL.setDirection(DcMotor.Direction.REVERSE);
        BackL.setDirection(DcMotor.Direction.REVERSE);
        FrontR.setDirection(DcMotor.Direction.REVERSE);
        BackR.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        //Wheel.setDirection(DcMotor.Direction.REVERSE);

        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        FrontL.setZeroPowerBehavior(brake);
        FrontR.setZeroPowerBehavior(brake);
        BackL.setZeroPowerBehavior(brake);
        BackR.setZeroPowerBehavior(brake);
        Intake.setZeroPowerBehavior(brake);

        Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // No servo movement in init
        liftIsRaised = false;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        panels.addData("Status", "Init complete (DualPad redundancy ready, robot-centric)");
        panels.update();
    }

    @Override
    public void start() {
        // Safe default: keep lift closed at TeleOp start
        try { Lift.setPosition(LIFT_LOWERED_POS); } catch (Exception ignored) {}
        liftIsRaised = false;
    }

    @Override
    public void loop() {
        // ===== Master toggle: gamepad1 D-Pad LEFT =====
        if (gamepad1.dpad_left && !g1DpadLeftPrev) {
            g2DriveMaster = !g2DriveMaster; // switch drive control source
        }
        g1DpadLeftPrev = gamepad1.dpad_left;

        // ===== Select active driving pad (robot-centric) =====
        boolean useG2ForDrive = g2DriveMaster;
        double y  = -(useG2ForDrive ? gamepad2.left_stick_y  : gamepad1.left_stick_y);
        double x  =  (useG2ForDrive ? gamepad2.left_stick_x  : gamepad1.left_stick_x) * 1.1;
        double rx =  (useG2ForDrive ? gamepad2.right_stick_x : gamepad1.right_stick_x);

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double fl = (y + x + rx) / denom;
        double bl = (y - x + rx) / denom;
        double fr = (y - x - rx) / denom;
        double br = (y + x - rx) / denom;
        FrontL.setPower(fl);  FrontR.setPower(fr);
        BackL.setPower(bl);   BackR.setPower(br);

        // ===== Intake (G2 has full control; if neutral, fall back to G1) =====
        double intakeCmdG2 = gamepad2.a ? 1.0 : (gamepad2.x ? -1.0 : 0.0);
        double intakeCmdG1 = gamepad1.a ? 1.0 : (gamepad1.x ? -1.0 : 0.0);
        double intakeCmd = (intakeCmdG2 != 0.0) ? intakeCmdG2 : intakeCmdG1;
        Intake.setPower(intakeCmd);

        // ===== Idle spin toggle on D-Pad Up (either pad) =====
        if (gamepad1.dpad_up && !g1DpadUpPrev) idleSpinEnabled = !idleSpinEnabled;
        if (gamepad2.dpad_up && !g2DpadUpPrev) idleSpinEnabled = !idleSpinEnabled;
        g1DpadUpPrev = gamepad1.dpad_up; g2DpadUpPrev = gamepad2.dpad_up;

        // ===== Manual ToF scan on Y (either pad can request) =====
        if (gamepad1.y || gamepad2.y) {
            beginScan();
        }

        // ===== Ramp to dynamic on B (either pad toggles). Pre-scan on the pad that pressed. =====
        if (gamepad1.b && !g1BPrev) {
            beginScan();
            isWheelOn = !isWheelOn;
            if (isWheelOn && !Double.isNaN(lastScanFt)) {
                targetWheelRPM = clamp(rpmFromFeet(lastScanFt), RPM_MIN, RPM_MAX);
            }
        }
        if (gamepad2.b && !g2BPrev) {
            beginScan();
            isWheelOn = !isWheelOn;
            if (isWheelOn && !Double.isNaN(lastScanFt)) {
                targetWheelRPM = clamp(rpmFromFeet(lastScanFt), RPM_MIN, RPM_MAX);
            }
        }
        g1BPrev = gamepad1.b; g2BPrev = gamepad2.b;

        // Small trims (either pad)
        if (gamepad1.right_bumper && !g1RbPrev) adjustTargetRPM(+STEP_SMALL);
        if (gamepad1.left_bumper  && !g1LbPrev) adjustTargetRPM(-STEP_SMALL);
        if (gamepad2.right_bumper && !g2RbPrev) adjustTargetRPM(+STEP_SMALL);
        if (gamepad2.left_bumper  && !g2LbPrev) adjustTargetRPM(-STEP_SMALL);
        g1RbPrev = gamepad1.right_bumper; g1LbPrev = gamepad1.left_bumper;
        g2RbPrev = gamepad2.right_bumper; g2LbPrev = gamepad2.left_bumper;

        // If scanning, collect one sample per loop and compute target on completion
        if (scanLeft > 0) {
            double inches = safeTofInches(tofFront);
            scanBuf[SCAN_SAMPLES - scanLeft] = inches;
            scanLeft--;
            if (scanLeft == 0) {
                double inchesMed = median(scanBuf);
                if (inchesMed > 6 && inchesMed <= 96) { // up to ~8 ft
                    double ft = inchesMed / 12.0 + SENSOR_TO_CANNON_OFFSET_FT;
                    lastScanFt = ft;
                    if (isWheelOn) targetWheelRPM = clamp(rpmFromFeet(ft), RPM_MIN, RPM_MAX);
                }
            }
        }

        // ===== Apply wheel control =====
        final double wheelTps  = safeVel(Wheel);
        final double wheelRpm  = toRPM(wheelTps, WHEEL_TPR);

        if (Wheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double v = getBatteryVoltage();
        double scale = 12.5 / Math.max(10.0, v);

        if (isWheelOn) {
            // Dynamic mode: hold targetWheelRPM
            double targetTps = (targetWheelRPM / 60.0) * WHEEL_TPR * scale;
            Wheel.setVelocity(targetTps);
        } else if (idleSpinEnabled) {
            // Idle momentum mode
            double idleTps = (IDLE_RPM / 60.0) * WHEEL_TPR * scale;
            Wheel.setVelocity(idleTps);
        } else {
            Wheel.setPower(0.0);
        }

        // ===== Dynamic Auto-Lift (target band + dwell) =====
        long nowMs = (long)(getRuntime() * 1000.0);
        boolean inBand = isWheelOn && (targetWheelRPM >= READY_MIN_TARGET_RPM)
                && (Math.abs(wheelRpm - targetWheelRPM) <= READY_TOL_RPM);

        if (inBand) {
            if (readyBandEnterMs < 0) readyBandEnterMs = nowMs;
            long dwell = nowMs - readyBandEnterMs;
            if (!liftIsRaised && dwell >= READY_HOLD_MS) {
                try { Lift.setPosition(LIFT_RAISED_POS); } catch (Exception ignored) {}
                liftIsRaised = true;
            }
        } else {
            readyBandEnterMs = -1;
            if (liftIsRaised) {
                try { Lift.setPosition(LIFT_LOWERED_POS); } catch (Exception ignored) {}
                liftIsRaised = false;
            }
        }

        // ===== Driver Station (minimal) =====
        double tofNowIn = safeTofInches(tofFront);
        double tofNowFt = (tofNowIn > 0) ? tofNowIn/12.0 : Double.NaN;
        double distFtDisplay = (!Double.isNaN(lastScanFt)) ? lastScanFt
                : (!Double.isNaN(tofNowFt) ? tofNowFt : Double.NaN);
        telemetry.addData("Active Driver", g2DriveMaster ? "Gamepad2" : "Gamepad1");
        telemetry.addData("G2 Drive Master", g2DriveMaster);
        telemetry.addData("Wheel RPM tgt", "%.0f", isWheelOn ? targetWheelRPM : (idleSpinEnabled ? IDLE_RPM : 0.0));
        telemetry.addData("Wheel RPM act", "%.0f", wheelRpm);
        telemetry.addData("Distance (ft)", (Double.isNaN(distFtDisplay) ? "—" : String.format("%.2f", distFtDisplay)));
        telemetry.addData("Mode", isWheelOn ? "DYNAMIC" : (idleSpinEnabled ? "IDLE" : "OFF"));
        telemetry.update();

        // ===== Panels (debug) =====
        panels.addData("ActiveDriver", g2DriveMaster ? "G2" : "G1");
        panels.addData("G2DriveMaster", g2DriveMaster);
        panels.addData("Wheel_ON(DYN)",  isWheelOn);
        panels.addData("IdleSpin", idleSpinEnabled);
        panels.addData("RPM_target", targetWheelRPM);
        panels.addData("Battery_V",  getBatteryVoltage());
        panels.addData("LiftRaised", liftIsRaised);
        panels.update();
    }

    // ------------ Helpers ------------

    private void beginScan() {
        Arrays.fill(scanBuf, 0.0);
        scanLeft = SCAN_SAMPLES;
    }

    private static double rpmFromFeet(double ft) { return M_RPM_PER_FT * ft + B_RPM_OFFSET; }

    private void adjustTargetRPM(double delta) { targetWheelRPM = clamp(targetWheelRPM + delta, RPM_MIN, RPM_MAX); }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private static double median(double[] a) {
        double[] b = Arrays.copyOf(a, a.length);
        Arrays.sort(b);
        int n = b.length;
        return (n % 2 == 1) ? b[n/2] : 0.5*(b[n/2 - 1] + b[n/2]);
    }

    private static double safeVel(DcMotorEx m) { try { return m.getVelocity(); } catch (Exception e) { return 0.0; } }

    private static double toRPM(double tps, double tpr) { return (tpr <= 0) ? 0.0 : (tps / tpr) * 60.0; }

    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.getAll(VoltageSensor.class)) {
            double v = s.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return (min == Double.POSITIVE_INFINITY) ? 0.0 : min;
    }

    private static double safeTofInches(DistanceSensor ds) {
        try {
            double d = ds.getDistance(DistanceUnit.INCH);
            return (Double.isNaN(d) || d <= 0) ? -1.0 : d;
        } catch (Exception e) { return -1.0; }
    }
}
// Certified Dylen Vasquez Design
