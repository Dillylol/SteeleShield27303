package org.firstinspires.ftc.teamcode.dataCollectors;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "BotelloRPMTest", group = "Tuning")
public class BjornTele_TuneRPM extends OpMode {

    // ---------- DRIVE (robot-centric) ----------
    private DcMotor FrontL, FrontR, BackL, BackR;

    // ---------- MECHS ----------
    private DcMotorEx Intake, Wheel;   // read velocity from Wheel
    private Servo Lift;
    private DistanceSensor tofFront;

    // ---------- AUX ----------
    private final Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    private IMU imu;

    // ---------- ENCODER / GEARING ----------
    private static final double MOTOR_CPR = 28.0;     // built-in
    private static final double WHEEL_RATIO = 1.0;    // set yours
    private static final double WHEEL_TPR = MOTOR_CPR * WHEEL_RATIO; // 28 for 1:1

    // ---------- FLYWHEEL (manual only in this tuner) ----------
    private double targetWheelRPM = 2600.0;
    private static final double STEP_FINE = 25.0;
    private static final double STEP_COARSE = 100.0;
    private static final double RPM_MIN = 0;
    private static final double RPM_MAX = 4000;

    // ---------- LAUNCH DETECTION ----------
    private double lastWheelRpm = 0.0;
    private double lastSampleTime = 0.0;
    private static final double DROP_RPM_PER_S = 1500.0; // tune: sudden drop threshold
    private static final double MIN_RPM_FOR_DETECT = 1200.0;
    private static final double REFRACTORY_MS = 700.0;  // ignore repeats for this long
    private double lastLaunchTime = -9999;

    // ---------- FALSE-POSITIVE SUPPRESSOR (spin-up warmup) ----------
    private static final double WARMUP_MS = 5000.0;   // disable detection for 5s after B turns wheel ON
    private double spinupStartTime = -1.0;            // runtime() when wheel was toggled ON

    // ---------- PENDING REVIEW (Yes/No poll after each accounted throw) ----------
    private boolean reviewPending = false;
    private double pDistFt = Double.NaN;
    private double pRequiredRPM = Double.NaN;
    private double pErrorRPM = Double.NaN;
    private double pWheelRpmAtDetect = Double.NaN;

    // ---------- REGRESSION ACCUMULATORS ----------
    // (1) requiredRPM vs distance(ft)
    private long n_req = 0;
    private double Sx_req = 0, Sy_req = 0, Sxx_req = 0, Sxy_req = 0;
    // (2) errorRPM vs distance(ft)
    private long n_err = 0;
    private double Sx_err = 0, Sy_err = 0, Sxx_err = 0, Sxy_err = 0;

    // Stats
    private long samplesKept = 0, samplesIgnored = 0;

    // ---------- BUTTON EDGE ----------
    private boolean rbPrev, lbPrev, dupPrev, ddownPrev, yPrev, aPrev, xPrev, bPrev, startPrev, backPrev;

    @Override public void init() {
        telemetry.setMsTransmissionInterval(50);

        // Motors match your existing map
        FrontL = hardwareMap.get(DcMotor.class, "lf");
        FrontR = hardwareMap.get(DcMotor.class, "rf");
        BackL  = hardwareMap.get(DcMotor.class, "lr");
        BackR  = hardwareMap.get(DcMotor.class, "rr");

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel  = hardwareMap.get(DcMotorEx.class, "Wheel");
        Lift   = hardwareMap.get(Servo.class, "Lift");
        tofFront = hardwareMap.get(DistanceSensor.class, "TOF");

        // Directions (copying your pattern)
        FrontL.setDirection(DcMotor.Direction.REVERSE);
        BackL.setDirection(DcMotor.Direction.REVERSE);
        FrontR.setDirection(DcMotor.Direction.REVERSE);
        BackR.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Wheel.setDirection(DcMotor.Direction.REVERSE);

        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        FrontL.setZeroPowerBehavior(brake);
        FrontR.setZeroPowerBehavior(brake);
        BackL.setZeroPowerBehavior(brake);
        BackR.setZeroPowerBehavior(brake);
        Intake.setZeroPowerBehavior(brake);

        Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // No lift motion in init
        try { Lift.setPosition(0.10); } catch (Exception ignored) {}

        // IMU present but not used for drive math (robot-centric requested).
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        panels.addData("Status", "TuneRPM init (robot-centric, manual wheel, ToF logging)");
        panels.update();
    }

    @Override public void loop() {
        // ---------- 1) ROBOT-CENTRIC MECANUM ----------
        double y  = -gamepad1.left_stick_y;  // forward
        double x  =  gamepad1.left_stick_x;  // strafe
        double rx =  gamepad1.right_stick_x; // rotate

        double denom = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
        double fl = (y + x + rx) / denom;
        double bl = (y - x + rx) / denom;
        double fr = (y - x - rx) / denom;
        double br = (y + x - rx) / denom;

        FrontL.setPower(fl);
        BackL.setPower(bl);
        FrontR.setPower(fr);
        BackR.setPower(br);

        // ---------- 2) INTAKE (hold-to-run; off otherwise; suppressed during review) ----------
        if (reviewPending) {
            // While the Yes/No poll is up, keep intake off so A can safely mean "No"
            Intake.setPower(0.0);
        } else {
            if (gamepad1.a && !gamepad1.x) {
                Intake.setPower(+1.0);   // hold A to intake forward
            } else if (gamepad1.x && !gamepad1.a) {
                Intake.setPower(-1.0);   // hold X to intake reverse
            } else {
                Intake.setPower(0.0);    // neither pressed => off
            }
        }

        // ---------- 3) FLYWHEEL CONTROLS ----------
        final double now = getRuntime();

        // B toggles ON/OFF, RB/LB => fine +/-; DPAD up/down => coarse +/-
        if (edgeDownB()) {
            if (Wheel.getPower() > 0.01 || Wheel.getVelocity() > 1) {
                Wheel.setPower(0);
                spinupStartTime = -1.0; // stop: cancel warmup
            } else {
                holdRPM(targetWheelRPM);
                spinupStartTime = now;   // start 5s warmup window
            }
        }
        if (edgeDownRB()) { adjustTarget(STEP_FINE);  if (isOn()) holdRPM(targetWheelRPM); }
        if (edgeDownLB()) { adjustTarget(-STEP_FINE); if (isOn()) holdRPM(targetWheelRPM); }
        if (edgeDownDUp())   { adjustTarget(STEP_COARSE);  if (isOn()) holdRPM(targetWheelRPM); }
        if (edgeDownDDown()) { adjustTarget(-STEP_COARSE); if (isOn()) holdRPM(targetWheelRPM); }

        // ---------- 4) MEASUREMENTS ----------
        final double wheelTps = safeVel(Wheel);
        final double wheelRpm = toRPM(wheelTps, WHEEL_TPR);

        double dt = Math.max(1e-3, now - lastSampleTime);
        double rpmSlope = (wheelRpm - lastWheelRpm) / dt; // rpm per second
        lastSampleTime = now;
        lastWheelRpm = wheelRpm;

        double tofIn = safeTofInches(tofFront);
        double distFt = (tofIn > 0) ? (tofIn / 12.0) : Double.NaN;

        // ---------- 5) LAUNCH DETECT with SPIN-UP SUPPRESSION ----------
        boolean warmupOver = (spinupStartTime < 0) || ((now - spinupStartTime) * 1000.0 >= WARMUP_MS);
        boolean suddenDrop = (wheelRpm > MIN_RPM_FOR_DETECT) && (rpmSlope <= -DROP_RPM_PER_S);
        boolean refractory = (now - lastLaunchTime) * 1000.0 < REFRACTORY_MS;

        // Only capture a new sample if not reviewing, not in warmup, etc.
        if (!reviewPending && warmupOver && suddenDrop && !refractory && isOn() && !Double.isNaN(distFt)) {
            lastLaunchTime = now;

            // compute candidate values
            double errorRPM = targetWheelRPM - wheelRpm;
            double requiredRPM = targetWheelRPM + errorRPM; // i.e., 2*target - actual

            boolean plausible = requiredRPM >= 500 && requiredRPM <= 5000 && distFt >= 0.5 && distFt <= 20.0;
            if (plausible) {
                // Stage for review (Yes/No poll)
                reviewPending = true;
                pDistFt = distFt;
                pRequiredRPM = requiredRPM;
                pErrorRPM = errorRPM;
                pWheelRpmAtDetect = wheelRpm;
            } else {
                samplesIgnored++;
            }
        }

        // ---------- 5.5) REVIEW HANDLER (Yes/No) ----------
        // Y => YES (mark as missed, add to final calc). A => NO (discard sample)
        if (reviewPending) {
            // YES
            if (edgeDownY()) {
                // commit staged values to regressions
                n_req++;
                Sx_req  += pDistFt;
                Sy_req  += pRequiredRPM;
                Sxx_req += pDistFt * pDistFt;
                Sxy_req += pDistFt * pRequiredRPM;

                n_err++;
                Sx_err  += pDistFt;
                Sy_err  += pErrorRPM;
                Sxx_err += pDistFt * pDistFt;
                Sxy_err += pDistFt * pErrorRPM;

                samplesKept++;

                // one-shot console + driver station print
                telemetry.addLine(String.format("FAILED at: (RPM=%.0f, dist=%.2fft)", pWheelRpmAtDetect, pDistFt));
                panels.addData("LastFail", String.format("RPM=%.0f, ft=%.2f", pWheelRpmAtDetect, pDistFt));
                panels.update();

                clearPending();
            }
            // NO
            else if (edgeDownA()) {
                samplesIgnored++;
                clearPending();
            }
        }

        // ---------- 6) QUICK CONTROLS ----------
        // START = clear all samples
        if (edgeDownStart()) { clearFits(); }
        // BACK = re-apply current fit to set target live (optional convenience)
        if (edgeDownBack() && n_req >= 2 && !Double.isNaN(distFt)) {
            double m = slope(n_req, Sx_req, Sy_req, Sxx_req, Sxy_req);
            double b = intercept(n_req, Sx_req, Sy_req, m);
            targetWheelRPM = clamp(b + m * distFt, RPM_MIN, RPM_MAX);
            if (isOn()) holdRPM(targetWheelRPM);
        }

        // ---------- 7) TELEMETRY ----------
        double mReq = (n_req >= 2) ? slope(n_req, Sx_req, Sy_req, Sxx_req, Sxy_req) : Double.NaN;
        double bReq = (n_req >= 2) ? intercept(n_req, Sx_req, Sy_req, mReq) : Double.NaN;

        double mErr = (n_err >= 2) ? slope(n_err, Sx_err, Sy_err, Sxx_err, Sxy_err) : Double.NaN;
        double bErr = (n_err >= 2) ? intercept(n_err, Sx_err, Sy_err, mErr) : Double.NaN;

        telemetry.addLine("=== Tuner: Manual Wheel + Launch Logging ===");
        telemetry.addData("Wheel tgt/act", "%.0f / %.0f rpm", targetWheelRPM, wheelRpm);
        telemetry.addData("Distance(ft)", (Double.isNaN(distFt) ? "—" : String.format("%.2f", distFt)));
        telemetry.addData("DropSlope(rpm/s)", "%.0f", rpmSlope);
        telemetry.addData("Samples kept/ignored", "%d / %d", samplesKept, samplesIgnored);
        if (!warmupOver && isOn()) {
            double msLeft = Math.max(0.0, WARMUP_MS - (now - spinupStartTime) * 1000.0);
            telemetry.addData("Spin-up", "WARMING (%.0f ms left)", msLeft);
        }
        telemetry.addLine("-- Fit: requiredRPM vs distance --> paste into main");
        telemetry.addData("M_RPM_PER_FT", (Double.isNaN(mReq) ? "n/a" : String.format("%.1f", mReq)));
        telemetry.addData("B_RPM_OFFSET", (Double.isNaN(bReq) ? "n/a" : String.format("%.1f", bReq)));
        telemetry.addLine("-- Fit: errorRPM vs distance (optional correction view)");
        telemetry.addData("m_err", (Double.isNaN(mErr) ? "n/a" : String.format("%.1f", mErr)));
        telemetry.addData("b_err", (Double.isNaN(bErr) ? "n/a" : String.format("%.1f", bErr)));
        if (reviewPending) {
            telemetry.addLine("\n>>> Missed? Y = YES (add + print fail),  A = NO (discard)");
            telemetry.addData("staged_dist_ft", String.format("%.2f", pDistFt));
            telemetry.addData("staged_req_rpm", String.format("%.0f", pRequiredRPM));
            telemetry.addData("staged_err_rpm", String.format("%.0f", pErrorRPM));
        }
        telemetry.addLine("Controls: B=toggle wheel | RB/LB=±25 | Dpad U/D=±100 | START=clear | BACK=apply fit to current distance");
        telemetry.update();

        panels.addData("RPM_tgt", targetWheelRPM);
        panels.addData("RPM_act", wheelRpm);
        panels.addData("ToF_in", tofIn);
        panels.addData("n_req", n_req);
        panels.addData("M_fit", mReq);
        panels.addData("B_fit", bReq);
        if (!warmupOver && isOn()) {
            double msLeft = Math.max(0.0, WARMUP_MS - (now - spinupStartTime) * 1000.0);
            panels.addData("SpinUp", String.format("warming %.0fms", msLeft));
        }
        if (reviewPending) {
            panels.addData("Poll", "Missed? Y=add, A=discard");
            panels.addData("stg_ft", pDistFt);
            panels.addData("stg_req", pRequiredRPM);
        }
        panels.update();
    }

    // ---------- Helpers ----------
    private boolean isOn() { return Wheel.getPower() > 0.01 || Wheel.getVelocity() > 1; }
    private void holdRPM(double rpm) {
        if (Wheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        double v = getBatteryVoltage();
        double scale = 12.5 / Math.max(10.0, v);
        double tps = (rpm / 60.0) * WHEEL_TPR * scale;
        Wheel.setVelocity(tps);
    }
    private static double safeVel(DcMotorEx m) { try { return m.getVelocity(); } catch (Exception e) { return 0.0; } }
    private static double toRPM(double tps, double tpr) { return (tpr <= 0) ? 0.0 : (tps / tpr) * 60.0; }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
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

    private void adjustTarget(double delta) { targetWheelRPM = clamp(targetWheelRPM + delta, RPM_MIN, RPM_MAX); }
    private void clearFits() {
        n_req = 0; Sx_req = Sy_req = Sxx_req = Sxy_req = 0;
        n_err = 0; Sx_err = Sy_err = Sxx_err = Sxy_err = 0;
        samplesKept = samplesIgnored = 0;
        clearPending();
    }
    private void clearPending() {
        reviewPending = false;
        pDistFt = pRequiredRPM = pErrorRPM = pWheelRpmAtDetect = Double.NaN;
    }
    private static double slope(long n, double Sx, double Sy, double Sxx, double Sxy) {
        double denom = n * Sxx - Sx * Sx;
        return (denom == 0) ? Double.NaN : (n * Sxy - Sx * Sy) / denom;
    }
    private static double intercept(long n, double Sx, double Sy, double m) {
        return Double.isNaN(m) ? Double.NaN : (Sy - m * Sx) / n;
    }

    // Button edges
    private boolean edgeDownRB(){ boolean now=gamepad1.right_bumper; boolean r=now && !rbPrev; rbPrev=now; return r; }
    private boolean edgeDownLB(){ boolean now=gamepad1.left_bumper;  boolean r=now && !lbPrev; lbPrev=now; return r; }
    private boolean edgeDownDUp(){ boolean now=gamepad1.dpad_up;     boolean r=now && !dupPrev; dupPrev=now; return r; }
    private boolean edgeDownDDown(){ boolean now=gamepad1.dpad_down; boolean r=now && !ddownPrev; ddownPrev=now; return r; }
    private boolean edgeDownY(){ boolean now=gamepad1.y; boolean r=now && !yPrev; yPrev=now; return r; }
    private boolean edgeDownA(){ boolean now=gamepad1.a; boolean r=now && !aPrev; aPrev=now; return r; }
    private boolean edgeDownX(){ boolean now=gamepad1.x; boolean r=now && !xPrev; xPrev=now; return r; }
    private boolean edgeDownB(){ boolean now=gamepad1.b; boolean r=now && !bPrev; bPrev=now; return r; }
    private boolean edgeDownStart(){ boolean now=gamepad1.start; boolean r=now && !startPrev; startPrev=now; return r; }
    private boolean edgeDownBack(){ boolean now=gamepad1.back; boolean r=now && !backPrev; backPrev=now; return r; }
}
