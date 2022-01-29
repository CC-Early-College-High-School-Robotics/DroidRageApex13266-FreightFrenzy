package org.firstinspires.ftc.teamcode.auto.paths.comp2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.comp2.一DefaultNewDetection;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BoxSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CameraSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.FlipperSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.TurretSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.Trajectories;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequenceimproved.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teleop.testing.TuningStart;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="forward thats it", group="Roadrunner Paths")
public class forward extends LinearOpMode {
    @Override
    public void runOpMode() {

        TuningStart.initializeTuning();
        Robot drive = new Robot(this);

        telemetry.addLine("Ready to Start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;


        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)

                .forward(25)
                .build();

        drive.followTrajectorySequence((Trajectory1));

    }

}

