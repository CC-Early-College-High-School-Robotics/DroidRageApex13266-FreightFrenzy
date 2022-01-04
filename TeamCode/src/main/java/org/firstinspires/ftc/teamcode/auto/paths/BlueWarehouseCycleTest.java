package org.firstinspires.ftc.teamcode.auto.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.一BlueCarouselDuckDetection;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.一AutoValues;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RoadrunnerTankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teleop.testing.TuningStart;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name="Blue warehouse cycle test Roadrunner Path", group="Roadrunner Paths")
public class BlueWarehouseCycleTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        TuningStart.initializeTuning();
        RoadrunnerTankDrive drive = new RoadrunnerTankDrive(hardwareMap);

        // On start

        waitForStart();
        if(isStopRequested()) return;










        Pose2d startPose = new Pose2d(-11, 62.5, Math.toRadians(90));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .back(19)
                .turn(Math.toRadians(-50))
                .forward(30)
                .turn(Math.toRadians(-40))
                .forward(50)
                .back(40)
                .build();

        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .splineTo(new Vector2d(-20, 48), Math.toRadians(200)) // reversed
                .build();

        TrajectorySequence Trajectory3 = drive.trajectorySequenceBuilder(Trajectory2.end())
                .forward(48)
                .turn(Math.toRadians(-20))
                .forward(30)
                .back(30)
                .build();

        TrajectorySequence Trajectory4 = drive.trajectorySequenceBuilder(Trajectory3.end())
                .splineTo(new Vector2d(-20, 48), Math.toRadians(200)) // reversed
                .build();

        /*
        TrajectorySequence Trajectory5 = drive.trajectorySequenceBuilder(Trajectory4.end())

                .build();

        TrajectorySequence Trajectory6 = drive.trajectorySequenceBuilder(Trajectory5.end())
                .forward(15)
                .build();

         */



        // Run trajectory 1
        drive.followTrajectorySequence((Trajectory1));
        drive.followTrajectorySequence((Trajectory2));
        drive.followTrajectorySequence((Trajectory3));
        drive.followTrajectorySequence((Trajectory4));
    }
}

