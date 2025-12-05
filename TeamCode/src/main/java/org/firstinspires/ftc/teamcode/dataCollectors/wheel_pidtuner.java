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
 * FlywheelPIDTuner (Dual) — computes a single feedforward F (kF_native)
 * from an open-loop power→velocity sweep and PRINTS the final value.
 *
 * Now supports two flywheel motors:
 *   - Maps "Wheel" and (optionally) "Wheel2"
 *   - Drives both with the same power
 *   - Uses the AVERAGE TPS of available motors for the fit
 *   - Shows per-motor RPM/TPS/amps and averaged values
 *
 * Controls (gamepad1)
 *   A = Run F fit (power sweep → linear fit) and print FINAL kF
 *   X = Re-run fit (same as A)
 *   B = Abort / stop wheels
 *   LS (press) = Toggle EMA smoothing (OFF default)
 *   RS (press) = Adjust EMA alpha (+0.05; hold LB to −0.05)
 */
@TeleOp(name = "FlywheelPIDTuner", group = "Tuning")
public class wheel_pidtuner extends OpMode {

    // ---- Hardware ----
    private DcMotorEx wheel;   // "Wheel"  (required)
    private DcMotorEx wheel2;  // "Wheel2" (optional but recommended)
    private final Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    private List<LynxModule> hubs;

    // ---- Config ----
    private boolean battComp = true; // (reserved) scale target for voltage during fit
    private double tpr = 28.0;       // overwritten from motor type (from "Wheel")
    private double gearRatio = 1.0;  // motor revs per flywheel rev
    private double effectiveTPR = 28.0; // ticks per flywheel rev

    // ---- EMA (OFF by default) ----
    private boolean emaEnabled = false;
    private double emaAlpha = 0.25;
    private double velEma1 = 0.0;  // EMA for wheel
    private double velEma2 = 0.0;  // EMA for wheel2
    private double velEmaAvg = 0.0; // EMA for averaged TPS (telemetry convenience)

    // ---- Results ----
    private double kF_native = 0.0;  // PWM-per-TPS in FTC native units

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);

        wheel = hardwareMap.get(DcMotorEx.class, "Wheel");
        try {
            wheel2 = hardwareMap.get(DcMotorEx.class, "Wheel2");
        } catch (Throwable ignored) {
            wheel2 = null; // allow single-motor fallback if Wheel2 not present
        }

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        MotorConfigurationType type = wheel.getMotorType();
        tpr = type.getTicksPerRev();
        effectiveTPR = tpr * gearRatio;

        // Reset & prep both motors
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setPower(0);

        if (wheel2 != null) {
            wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel2.setPower(0);
        }

        panels.addData("FlywheelPIDTuner (Dual)", String.format(Locale.US,
                "TPR=%.1f (effective=%.1f) | motors=%s",
                tpr, effectiveTPR, (wheel2 != null ? "2" : "1")));
        panels.update();

        telemetry.addData("Init", "Wheel2 %s", (wheel2 != null ? "FOUND" : "not found (single-motor mode)"));
        telemetry.update();
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

        // Live readings
        double tps1 = safeTPS(wheel, 1);
        double tps2 = (wheel2 != null) ? safeTPS(wheel2, 2) : Double.NaN;
        double tpsAvg = averageOfPresent(tps1, tps2);
        if (emaEnabled) {
            velEmaAvg = velEmaAvg + emaAlpha * (tpsAvg - velEmaAvg);
        } else {
            velEmaAvg = tpsAvg;
        }

        double rpm1 = tpsToRpm(tps1);
        double rpm2 = (wheel2 != null) ? tpsToRpm(tps2) : Double.NaN;
        double rpmAvg = tpsToRpm(tpsAvg);

        double batt = getBatteryVoltage();

        double amps1 = 0, amps2 = 0;
        try { amps1 = wheel.getCurrent(CurrentUnit.AMPS); } catch (Throwable ignored) {}
        if (wheel2 != null) {
            try { amps2 = wheel2.getCurrent(CurrentUnit.AMPS); } catch (Throwable ignored) {}
        }

        // Driver station telemetry
        telemetry.addLine("=== Live ===");
        telemetry.addData("Batt(V)", "%.2f", batt);
        telemetry.addData("EMA", emaEnabled ? ("ON α=" + String.format(Locale.US, "%.2f", emaAlpha)) : "OFF");
        telemetry.addData("kF_native", String.format(Locale.US, "%.5f", kF_native));

        telemetry.addLine("--- Motor 1 ---");
        telemetry.addData("RPM1", "%.0f", rpm1);
        telemetry.addData("TPS1", "%.0f", tps1);
        telemetry.addData("I1(A)", "%.2f", amps1);

        if (wheel2 != null) {
            telemetry.addLine("--- Motor 2 ---");
            telemetry.addData("RPM2", "%.0f", rpm2);
            telemetry.addData("TPS2", "%.0f", tps2);
            telemetry.addData("I2(A)", "%.2f", amps2);
        }

        telemetry.addLine("--- Average ---");
        telemetry.addData("RPM(avg)", "%.0f", rpmAvg);
        telemetry.addData("TPS(avg)", "%.0f", tpsAvg);
        telemetry.addData("TPS(avg EMA)", "%.0f", velEmaAvg);
        telemetry.update();

        // Panels telemetry
        panels.addData("kF_native", kF_native);
        panels.addData("EMA", emaEnabled ? ("ON α=" + String.format(Locale.US, "%.2f", emaAlpha)) : "OFF");
        panels.addData("Batt(V)", batt);
        panels.addData("RPM1", rpm1);
        panels.addData("TPS1", tps1);
        if (wheel2 != null) {
            panels.addData("RPM2", rpm2);
            panels.addData("TPS2", tps2);
        }
        panels.addData("RPM(avg)", rpmAvg);
        panels.addData("TPS(avg)", tpsAvg);
        panels.update();
    }

    private void runFitAndPrint() {
        // Open-loop sweep and linear fit: TPS(avg) = a*power + b
        double[] powers = new double[]{0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85};
        List<Double> xs = new ArrayList<>();
        List<Double> ys = new ArrayList<>();

        for (double p : powers) {
            holdPowerAndSample(p, 0.9); // settle
            double meanTPS = sampleMeanTPS(0.35); // averaged across present motors
            xs.add(p);
            ys.add(meanTPS);
            telemetry.addData("fit", "p=%.2f tps(avg)=%.0f", p, meanTPS);
            telemetry.update();
        }
        double[] fit = linearFit(xs, ys);
        double slopeTPSperPower = fit[0];

        if (slopeTPSperPower > 100) kF_native = 32767.0 / slopeTPSperPower; else kF_native = 0.0;

        // Print the ONLY thing we care about.
        telemetry.clearAll();
        telemetry.addLine("=== Flywheel kF (native) ===");
        telemetry.addData("kF_native", String.format(Locale.US, "%.6f", kF_native));
        telemetry.addLine("Paste: Wheel.setVelocityPIDFCoefficients(P, I, D, kF_native);");
        telemetry.update();

        panels.addData("kF_native", String.format(Locale.US, "%.6f", kF_native));
        panels.addData("Hint", "Use setVelocityPIDFCoefficients(..., kF_native)");
        panels.update();

        abort();
    }

    // ---- Helpers ----

    private void holdPowerAndSample(double power, double settleSeconds) {
        timer.reset();
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setPower(power);
        if (wheel2 != null) {
            wheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel2.setPower(power);
        }
        while (opModeIsActive() && timer.seconds() < settleSeconds) idleOnce();
    }

    private double sampleMeanTPS(double windowSeconds) {
        double sum = 0; int n = 0; timer.reset();
        while (opModeIsActive() && timer.seconds() < windowSeconds) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            double t1 = safeTPS(wheel, 1);
            double t2 = (wheel2 != null) ? safeTPS(wheel2, 2) : Double.NaN;
            sum += averageOfPresent(t1, t2);
            n++;
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

    private double safeTPS(DcMotorEx m, int idx){
        try {
            double raw = m.getVelocity(); // native TPS
            if (!emaEnabled) return raw;
            if (idx == 1) { velEma1 = velEma1 + emaAlpha * (raw - velEma1); return velEma1; }
            else           { velEma2 = velEma2 + emaAlpha * (raw - velEma2); return velEma2; }
        } catch (Throwable t){
            // If sensor read fails, keep last EMA (if enabled) or 0
            if (!emaEnabled) return 0.0;
            return (idx == 1) ? velEma1 : velEma2;
        }
    }

    private double averageOfPresent(double a, double b){
        boolean aOk = !Double.isNaN(a);
        boolean bOk = !Double.isNaN(b);
        if (aOk && bOk) return 0.5*(a+b);
        if (aOk) return a;
        if (bOk) return b;
        return 0.0;
    }

    private void abort(){
        wheel.setPower(0);
        if (wheel2 != null) wheel2.setPower(0);
    }

    // OpMode scaffolding for this standalone tuner
    private boolean opModeIsActive(){ return true; }
    private void idleOnce(){ try { Thread.sleep(5); } catch (InterruptedException ignored) {} }

    private double getBatteryVoltage(){
        try { return hardwareMap.voltageSensor.iterator().next().getVoltage(); }
        catch (Throwable ignored){ return 12.0; }
    }
}
