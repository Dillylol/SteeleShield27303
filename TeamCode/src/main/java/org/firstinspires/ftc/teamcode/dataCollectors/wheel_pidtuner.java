package org.firstinspires.ftc.teamcode.dataCollectors;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * FlywheelPIDTuner (Simple) — one‑job tool that computes a single feedforward F (kF_native)
 * from an open‑loop power→velocity sweep and PRINTS the final value to telemetry.
 *
 * No files, no JSON, no per‑band PID. EMA smoothing exists but is OFF by default.
 *
 * Controls (gamepad1)
 *   A = Run F fit (power sweep → linear fit) and print FINAL kF
 *   X = Re-run fit (same as A)
 *   B = Abort / stop wheel
 *   LS (press) = Toggle EMA smoothing (OFF default)
 *   RS (press) = Adjust EMA alpha (+0.05; hold LB to −0.05)
 */
@TeleOp(name = "FlywheelPIDTuner", group = "Tuning")
public class wheel_pidtuner extends OpMode {

    // ---- Hardware ----
    private DcMotorEx wheel;
    private final Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    private List<LynxModule> hubs;

    // ---- Config ----
    private boolean battComp = true; // scale target for voltage during fit (printed along with kF)
    private double tpr = 28.0;       // overwritten from motor type
    private double gearRatio = 1.0;  // motor revs per flywheel rev
    private double effectiveTPR = 28.0; // ticks per flywheel rev

    // ---- EMA (OFF by default) ----
    private boolean emaEnabled = false;
    private double velEma = 0.0;
    private double emaAlpha = 0.25;

    // ---- Results ----
    private double kF_native = 0.0;  // PWM-per-TPS in FTC native units

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);

        wheel = hardwareMap.get(DcMotorEx.class, "Wheel");

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        MotorConfigurationType type = wheel.getMotorType();
        tpr = type.getTicksPerRev();
        effectiveTPR = tpr * gearRatio;

        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setPower(0);

        panels.addData("FlywheelPIDTuner (Simple)", String.format(Locale.US,
                "TPR=%.1f (effective=%.1f)", tpr, effectiveTPR));
        panels.update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        // Controls
        if (gamepad1.left_stick_button) emaEnabled = !emaEnabled;
        if (gamepad1.right_stick_button) {
            if (gamepad1.left_bumper) emaAlpha = Math.max(0.05, emaAlpha - 0.05);
            else emaAlpha = Math.min(0.6, emaAlpha + 0.05);
        }
        if (gamepad1.b) abort();
        if (gamepad1.a || gamepad1.x) runFitAndPrint();

        double tps = safeTPS(wheel);
        double rpm = tpsToRpm(tps);
        double batt = getBatteryVoltage();
        double amps = 0; try { amps = wheel.getCurrent(CurrentUnit.AMPS); } catch (Throwable ignored) {}

        telemetry.addData("RPM", "%.0f", rpm);
        telemetry.addData("TPS", "%.0f", tps);
        telemetry.addData("Batt(V)", "%.2f", batt);
        telemetry.addData("EMA", emaEnabled ? ("ON α="+String.format(Locale.US,"%.2f",emaAlpha)) : "OFF");
        telemetry.addData("kF_native", String.format(Locale.US, "%.5f", kF_native));
        telemetry.update();

        panels.addData("RPM", rpm);
        panels.addData("TPS", tps);
        panels.addData("kF_native", kF_native);
        panels.addData("EMA", emaEnabled ? ("ON α="+String.format(Locale.US,"%.2f",emaAlpha)) : "OFF");
        panels.update();
    }

    private void runFitAndPrint() {
        // Open-loop sweep and linear fit: TPS = a*power + b
        double[] powers = new double[]{0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85};
        List<Double> xs = new ArrayList<>();
        List<Double> ys = new ArrayList<>();

        for (double p : powers) {
            holdPowerAndSample(p, 0.9); // settle
            double meanTPS = sampleMeanTPS(0.35);
            xs.add(p);
            ys.add(meanTPS);
            telemetry.addData("fit", "p=%.2f tps=%.0f", p, meanTPS);
            telemetry.update();
        }
        double[] fit = linearFit(xs, ys);
        double slopeTPSperPower = fit[0];

        if (slopeTPSperPower > 100) kF_native = 32767.0 / slopeTPSperPower; else kF_native = 0.0;

        // Print the ONLY thing we care about.
        telemetry.clearAll();
        telemetry.addLine("=== Flywheel kF (native) ===");
        telemetry.addData("kF_native", String.format(Locale.US, "%.6f", kF_native));
        telemetry.addLine("Paste idea: Wheel.setVelocityPIDFCoefficients(P, I, D, kF_native);");
        telemetry.update();

        panels.addData("kF_native", String.format(Locale.US, "%.6f", kF_native));
        panels.addData("Hint", "Use setVelocityPIDFCoefficients(..., kF_native)");
        panels.update();

        wheel.setPower(0);
    }

    // ---- Helpers ----

    private void holdPowerAndSample(double power, double settleSeconds) {
        timer.reset();
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setPower(power);
        while (opModeIsActive() && timer.seconds() < settleSeconds) idleOnce();
    }

    private double sampleMeanTPS(double windowSeconds) {
        double sum = 0; int n = 0; timer.reset();
        while (opModeIsActive() && timer.seconds() < windowSeconds) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            sum += safeTPS(wheel); n++;
            idleOnce();
        }
        return (n > 0) ? (sum / n) : 0;
    }

    private double[] linearFit(List<Double> xs, List<Double> ys) {
        double sx=0, sy=0, sxx=0, sxy=0; int n = Math.min(xs.size(), ys.size());
        for (int i=0;i<n;i++){ double x=xs.get(i), y=ys.get(i); sx+=x; sy+=y; sxx+=x*x; sxy+=x*y; }
        double denom = n*sxx - sx*sx;
        if (denom == 0) return new double[]{0,0};
        double a = (n*sxy - sx*sy)/denom; // slope
        double b = (sy - a*sx)/n;         // intercept (unused)
        return new double[]{a,b};
    }

    private double rpmToTps(double rpm){ return (rpm/60.0)*effectiveTPR; }
    private double tpsToRpm(double tps){ return (effectiveTPR>0)? (tps*60.0/effectiveTPR):0.0; }

    private double safeTPS(DcMotorEx m){
        try {
            double raw = m.getVelocity();
            if (!emaEnabled) return raw;
            velEma = velEma + emaAlpha * (raw - velEma);
            return velEma;
        } catch (Throwable t){
            return emaEnabled ? velEma : 0.0;
        }
    }

    private void abort(){ wheel.setPower(0); }
    private boolean opModeIsActive(){ return true; }
    private void idleOnce(){ try { Thread.sleep(5); } catch (InterruptedException ignored) {} }

    private double getBatteryVoltage(){
        try { return hardwareMap.voltageSensor.iterator().next().getVoltage(); }
        catch (Throwable ignored){ return 12.0; }
    }
}
