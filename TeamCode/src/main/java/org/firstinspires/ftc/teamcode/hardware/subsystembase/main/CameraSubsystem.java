package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.paths.comp2.AutoValues;
import org.firstinspires.ftc.teamcode.auto.pipeline.comp2.一DefaultNewDetection;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class CameraSubsystem extends BaseSubsystem {
    OpenCvPipeline detector;
    LinearOpMode opMode;
    // Constructor
    public CameraSubsystem(LinearOpMode opMode, OpenCvPipeline detector) {
        this.opMode = opMode;
        this.detector = detector;
    }

    public Vector2d redWarehouseDetection (一DefaultNewDetection detector) {
        switch (detector.getAnalysis()) {
            case LEFT: return new Vector2d(20, -64);
            case CENTER: return new Vector2d(20, -64);
            case RIGHT: return new Vector2d(20, -64);
            default: return new Vector2d(20, -64);
        }
    }
    public String defaultDetectionCheck(一DefaultNewDetection detector) {
        switch (detector.getAnalysis()) {
            case LEFT: return "Low";
            case CENTER: return "Mid";
            default: return "High";
        }
    }
}