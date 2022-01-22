package org.firstinspires.ftc.teamcode.auto.paths.comp2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerImprovedTankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequenceimproved.TrajectorySequence;

@Config
@Autonomous(name="Red warehouse cycle spline test Roadrunner Path", group="Roadrunner Paths")
public class RedWarehouseCycleSplineTest extends LinearOpMode {
    public static double MOTORVELOF = 1.0;
    public static double MOTORVELOFBACKFORTH = 0.5;
    public static double MOTORVELOFSPLINEIN = 0.5;
    public static double MOTORVELOF3 = 1.0;
    @Override
    public void runOpMode() {
        RoadRunnerImprovedTankDrive drive = new RoadRunnerImprovedTankDrive(this);

        // On start

        waitForStart();
        if(isStopRequested()) return;










        Pose2d startPose = new Pose2d(-11, -62.5, Math.toRadians(-90));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .back(15)
                .build();


        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .turn(Math.toRadians(50))
                .build();

        TrajectorySequence Trajectory3 = drive.trajectorySequenceBuilder(Trajectory2.end())
                .setReversed(false)
                .splineTo(new Vector2d(20, -64), Math.toRadians(0))
                .build();

        TrajectorySequence Forward = drive.trajectorySequenceBuilder(Trajectory3.end())
                .forward(24)
                .build();

        TrajectorySequence Backward = drive.trajectorySequenceBuilder(Forward.end())
                .back(24)
                .build();

        TrajectorySequence SplineOut = drive.trajectorySequenceBuilder(Backward.end())

                .setReversed(true)
                .splineTo(new Vector2d(-15, -44), Math.toRadians(-200)) // reversed
                .build();

        TrajectorySequence SplineIn = drive.trajectorySequenceBuilder(SplineOut.end())
                .setReversed(false)
                .splineTo(new Vector2d(20, -64), Math.toRadians(0))
                .build();

        /*
        TrajectorySequence Trajectory5 = drive.trajectorySequenceBuilder(Trajectory4.end())

                .build();

        TrajectorySequence Trajectory6 = drive.trajectorySequenceBuilder(Trajectory5.end())
                .forward(15)
                .build();

         */



        // Run trajectory 1
        drive.setMotorRPMFraction(MOTORVELOF);
        drive.followTrajectorySequence((Trajectory1));

        drive.setMotorRPMFraction(MOTORVELOFBACKFORTH);
        drive.followTrajectorySequence((Trajectory2));
        drive.setMotorRPMFraction(MOTORVELOFSPLINEIN);
        drive.followTrajectorySequence((Trajectory3));

        drive.setMotorRPMFraction(MOTORVELOFBACKFORTH);
        drive.followTrajectorySequence((Forward));
        drive.followTrajectorySequence((Backward));

        drive.setMotorRPMFraction(MOTORVELOF);
        drive.followTrajectorySequence((SplineOut));
        drive.setMotorRPMFraction(MOTORVELOFSPLINEIN);
        drive.followTrajectorySequence((SplineIn));

        drive.setMotorRPMFraction(MOTORVELOFBACKFORTH);
        drive.followTrajectorySequence((Forward));
        drive.followTrajectorySequence((Backward));
        drive.setMotorRPMFraction(MOTORVELOF);
        drive.followTrajectorySequence((SplineOut));
        drive.setMotorRPMFraction(MOTORVELOFSPLINEIN);
        drive.followTrajectorySequence((SplineIn));

        drive.setMotorRPMFraction(MOTORVELOFBACKFORTH);
        drive.followTrajectorySequence((Forward));
        drive.followTrajectorySequence((Backward));
        drive.setMotorRPMFraction(MOTORVELOF);
        drive.followTrajectorySequence((SplineOut));
        drive.setMotorRPMFraction(MOTORVELOFSPLINEIN);
        drive.followTrajectorySequence((SplineIn));

    }
}

