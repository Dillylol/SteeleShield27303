package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

@TeleOp(name = "BjornTeleBLUE", group = "Bjorn")
public final class BlueTagShooterTele extends BaseTagShooterTele {

    @Override
    protected boolean isTargetGoal(TagObservation obs) {
        return obs != null && (obs.id == CameraConfig.BLUE_GOAL_TAG_ID
                || CameraConfig.CLASS_BLUE_GOAL.equals(obs.tagClass));
    }

    @Override
    protected String allianceName() {
        return "BLUE";
    }

    @Override
    protected Pose getAutoEndPose() {
        return BjornConstants.Auto.BLUE_AUTO_END_POSE;
    }
}
