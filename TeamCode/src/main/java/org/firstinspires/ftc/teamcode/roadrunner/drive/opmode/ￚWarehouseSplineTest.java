package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.RoadrunnerTankDrive;
import org.firstinspires.ftc.teamcode.teleop.testing.TuningStart;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class ￚWarehouseSplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TuningStart.initializeTuning();
        RoadrunnerTankDrive drive = new RoadrunnerTankDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        Pose2d initialPose = new Pose2d(21, 63, Math.toRadians(0));

        Trajectory traj = drive.trajectoryBuilder(initialPose, true)
                .splineTo(new Vector2d(-20, 48), Math.toRadians(200))
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                        .build()
        );
    }
}
