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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.Arrays;

@TeleOp(name = "BjornTele")
public class BjornTele extends OpMode {

    // Drive motors (stay DcMotor – no velocity reads)
    private DcMotor BackL, BackR, FrontL, FrontR;

    // Mechanisms (use DcMotorEx so we can read velocity)
    private DcMotorEx Intake, Wheel;

    // Auto-lift servo (stays lowered, lifts only when wheel velocity >= threshold)
    private Servo Lift;

    // Panels telemetry
    private final Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // IMU for field-centric
    private IMU imu;

    // ToF distance sensor at the very front (matches how you measured distances)
    private DistanceSensor tofFront;

    // Toggles
    private boolean isWheelOn  = false;   // B toggles this
    private boolean bWasPressed = false;

    // Button edges for tuning / scan
    private boolean rbPrev=false, lbPrev=false, dpadUpPrev=false, dpadDownPrev=false;

    // REV-41-1600 (28 CPR) * gear ratio => ticks per output revolution
    private static final double MOTOR_ENCODER_CPR = 28.0;
    private static final double INTAKE_GEAR_RATIO = 20.0; // TODO: set yours
    private static final double WHEEL_GEAR_RATIO  = 1.0;  // TODO: set yours

    private static final double INTAKE_TPR = MOTOR_ENCODER_CPR * INTAKE_GEAR_RATIO; // 560 for 20:1
    private static final double WHEEL_TPR  = MOTOR_ENCODER_CPR * WHEEL_GEAR_RATIO;  // 28 for 1:1

    // ===== Servo config =====
    private static final double LIFT_LOWERED_POS = 0.10; // resting (default)
    private static final double LIFT_RAISED_POS  = 0.85; // raised when wheel fast enough

    // ===== Dynamic lift readiness (opens when wheel RPM ≈ target for a short hold time)
    private static final double READY_TOL_RPM = 250.0;      // ± band around target
    private static final double READY_MIN_TARGET_RPM = 2000.0; // don't open for tiny targets
    private static final long   READY_HOLD_MS = 500;        // must be in-band this long

    private boolean liftIsRaised = false;    // current servo state
    private long    readyBandEnterMs = -1;   // time we first entered the band

    // ===== Tunable RPM hold =====
    private double targetWheelRPM = 2825.0;     // default near your mid preset
    private static final double STEP_SMALL = 25;   // RB/LB only (big steps removed)
    private static final double RPM_MIN    = 0.0;
    private static final double RPM_MAX    = 3800.0; // sanity clamp

    // Linear fit from your field tests (distance in FEET taken from front of robot)
    // RPM ≈ m*Dft + b
    private static final double M_RPM_PER_FT = 83.3;
    private static final double B_RPM_OFFSET = 2364.0;

    // ToF scan state
    private static final int SCAN_SAMPLES = 10;
    private int scanLeft = 0;
    private final double[] scanBuf = new double[SCAN_SAMPLES];
    private double lastScanFt = Double.NaN; // for telemetry
    // If the cannon were offset behind the sensor, you'd add it here; for you it's 0:
    private static final double SENSOR_TO_CANNON_OFFSET_FT = 0.0;

    @Override
    public void init() {
        // Make DS telemetry snappier
        telemetry.setMsTransmissionInterval(50);

        // Map drive
        BackL  = hardwareMap.get(DcMotor.class, "lr");
        BackR  = hardwareMap.get(DcMotor.class, "rr");
        FrontL = hardwareMap.get(DcMotor.class, "lf");
        FrontR = hardwareMap.get(DcMotor.class, "rf");

        // Map mechanisms as DcMotorEx
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel  = hardwareMap.get(DcMotorEx.class, "Wheel");

        // Map servo
        Lift   = hardwareMap.get(Servo.class, "Lift");

        // Map ToF by config name (set this to your actual config device name)
        tofFront = hardwareMap.get(DistanceSensor.class, "TOF");

        // Directions
        FrontL.setDirection(DcMotor.Direction.REVERSE);
        BackL.setDirection(DcMotor.Direction.REVERSE);
        FrontR.setDirection(DcMotor.Direction.REVERSE);
        BackR.setDirection(DcMotor.Direction.REVERSE);

        Intake.setDirection(DcMotor.Direction.REVERSE);
        Wheel.setDirection(DcMotor.Direction.REVERSE);

        // Brakes
        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        FrontL.setZeroPowerBehavior(brake);
        FrontR.setZeroPowerBehavior(brake);
        BackL.setZeroPowerBehavior(brake);
        BackR.setZeroPowerBehavior(brake);
        Intake.setZeroPowerBehavior(brake);
        Wheel.setZeroPowerBehavior(brake);

        // Encoders for velocity control
        Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // *** RULE COMPLIANT: No servo movement in init ***
        // Do NOT call Lift.setPosition() here.
        // We'll command it in start()/loop() only.
        liftIsRaised = false; // internal flag only

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        panels.addData("Status", "Init complete: ToF Scan RPM Assist (no servo move)");
        panels.update();
    }

    @Override
    public void start() {
        // Safe default at TeleOp start (now movement is allowed): keep Lift lowered
        if (Lift != null) {
            try { Lift.setPosition(LIFT_LOWERED_POS); } catch (Exception ignored) {}
        }
        liftIsRaised = false;
    }

    @Override
    public void loop() {
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

        // ===== Mechanisms =====
        // Intake: A = normal intake, X = reverse, else off
        double intakeCmd = gamepad1.a ? 0.5 : (gamepad1.x ? -0.5 : 0.0);
        Intake.setPower(intakeCmd);

        // Wheel toggle on B:
        if (gamepad1.b && !bWasPressed) isWheelOn = !isWheelOn;
        bWasPressed = gamepad1.b;

        // Small trims only (±25) on RB/LB
        if (gamepad1.right_bumper && !rbPrev) adjustTargetRPM(+STEP_SMALL);
        if (gamepad1.left_bumper  && !lbPrev) adjustTargetRPM(-STEP_SMALL);
        rbPrev = gamepad1.right_bumper;
        lbPrev = gamepad1.left_bumper;

        // ===== ToF SCAN on D-Pad Up (edge trigger) =====
        if (gamepad1.dpad_up && !dpadUpPrev) { beginScan(); }
        dpadUpPrev = gamepad1.dpad_up;

        // ===== IMU reset on D-Pad Down (edge) =====
        if (gamepad1.dpad_down && !dpadDownPrev) { imu.resetYaw(); }
        dpadDownPrev = gamepad1.dpad_down;

        // If scanning, collect one sample per loop
        if (scanLeft > 0) {
            double inches = safeTofInches(tofFront);
            scanBuf[SCAN_SAMPLES - scanLeft] = inches;
            scanLeft--;
            if (scanLeft == 0) {
                double inchesMed = median(scanBuf);
                // Basic validity & clamp
                if (inchesMed > 6 && inchesMed <= 120) { // accept up to ~8 ft
                    double ft = inchesMed / 12.0 + SENSOR_TO_CANNON_OFFSET_FT;
                    lastScanFt = ft;
                    targetWheelRPM = clamp(rpmFromFeet(ft), RPM_MIN, RPM_MAX);
                } // else ignore bad scan (leave target as-is)
            }
        }

        // ===== Apply hold or stop =====
        final double wheelTps  = safeVel(Wheel);
        final double wheelRpm  = toRPM(wheelTps, WHEEL_TPR);

        if (isWheelOn) {
            if (Wheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            // Optional light battery compensation
            double v = getBatteryVoltage();
            double scale = 12.5 / Math.max(10.0, v);
            double targetTps = (targetWheelRPM / 60.0) * WHEEL_TPR * scale;
            Wheel.setVelocity(targetTps);
        } else {
            Wheel.setPower(0.0);
        }

        // ===== Dynamic Auto-Lift based on readiness (target RPM ± tol for hold time) =====
        long nowMs = (long)(getRuntime() * 1000.0);
        boolean inBand = isWheelOn && (targetWheelRPM >= READY_MIN_TARGET_RPM)
                && (Math.abs(wheelRpm - targetWheelRPM) <= READY_TOL_RPM);

        if (inBand) {
            if (readyBandEnterMs < 0) readyBandEnterMs = nowMs;
            long dwell = nowMs - readyBandEnterMs;
            if (!liftIsRaised && dwell >= READY_HOLD_MS) {
                // Open after stable dwell time
                try { Lift.setPosition(LIFT_RAISED_POS); } catch (Exception ignored) {}
                liftIsRaised = true;
            }
        } else {
            readyBandEnterMs = -1; // left the band; reset timer
            if (liftIsRaised) {
                try { Lift.setPosition(LIFT_LOWERED_POS); } catch (Exception ignored) {}
                liftIsRaised = false;
            }
        }

        // ======== DRIVER STATION (minimal) ========
        // Prefer last scanned distance; fall back to live ToF
        double tofNowIn = safeTofInches(tofFront);
        double tofNowFt = (tofNowIn > 0) ? tofNowIn/12.0 : Double.NaN;
        double distFtDisplay = (!Double.isNaN(lastScanFt)) ? lastScanFt
                : (!Double.isNaN(tofNowFt) ? tofNowFt : Double.NaN);

        telemetry.addData("Wheel RPM tgt", "%.0f", targetWheelRPM);
        telemetry.addData("Wheel RPM act", "%.0f", wheelRpm);
        telemetry.addData("Distance (ft)", (Double.isNaN(distFtDisplay) ? "—" : String.format("%.2f", distFtDisplay)));
        telemetry.update();

        // ===== Panels (for graphs / deeper debug) =====
        panels.addData("Wheel_ON",  isWheelOn);
        panels.addData("RPM_target", targetWheelRPM);
        panels.addData("RPM_meas",   wheelRpm);
        panels.addData("ToF_in_now", tofNowIn);
        panels.addData("ToF_ft_lastScan", lastScanFt);
        panels.addData("Battery_V",  getBatteryVoltage());
        panels.addData("Lift_Pos",   Lift.getPosition());
        panels.addData("LiftRaised", liftIsRaised);
        panels.addData("ReadyBand", Math.abs(wheelRpm - targetWheelRPM));
        panels.update();
    }

    // ------------ Helpers ------------

    private void beginScan() {
        Arrays.fill(scanBuf, 0.0);
        scanLeft = SCAN_SAMPLES;
    }

    private static double rpmFromFeet(double ft) {
        return M_RPM_PER_FT * ft + B_RPM_OFFSET;
    }

    private void adjustTargetRPM(double delta) {
        targetWheelRPM = clamp(targetWheelRPM + delta, RPM_MIN, RPM_MAX);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double median(double[] a) {
        double[] b = Arrays.copyOf(a, a.length);
        Arrays.sort(b);
        int n = b.length;
        return (n % 2 == 1) ? b[n/2] : 0.5*(b[n/2 - 1] + b[n/2]);
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

    private static double safeTofInches(DistanceSensor ds) {
        try {
            double d = ds.getDistance(DistanceUnit.INCH);
            return (Double.isNaN(d) || d <= 0) ? -1.0 : d;
        } catch (Exception e) {
            return -1.0;
        }
    }
}
// Certified Dylen Vasquez Design
