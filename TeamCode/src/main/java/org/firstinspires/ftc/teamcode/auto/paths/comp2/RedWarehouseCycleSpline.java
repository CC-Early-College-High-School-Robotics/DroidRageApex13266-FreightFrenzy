package org.firstinspires.ftc.teamcode.auto.paths.comp2;

import static org.firstinspires.ftc.teamcode.auto.paths.comp2.AutoValues.CYCLE_DROP_WAIT;
import static org.firstinspires.ftc.teamcode.auto.paths.comp2.AutoValues.CYCLE_DROP_WAIT2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Config
@Autonomous(name="Red warehouse cycle spline REAL Roadrunner Path", group="Roadrunner Paths")
public class RedWarehouseCycleSpline extends LinearOpMode {
    ArmSubsystem arm                            = new ArmSubsystem                          ();
    BoxSubsystem box                            = new BoxSubsystem                          ();
    CarouselSubsystem carousel                  = new CarouselSubsystem                     ();
    IntakeSubsystem intake                      = new IntakeSubsystem                       ();
    //    TelemetrySubsystem telemetrySubsystem       = new TelemetrySubsystem                    ();
    TurretSubsystem turret                      = new TurretSubsystem                       ();
    FlipperSubsystem flipper                    = new FlipperSubsystem                      ();
    DistanceSensorSubsystem distanceSensor      = new DistanceSensorSubsystem               ();

    AutoCommands command = new AutoCommands(this, arm, flipper, distanceSensor, intake, carousel);

    一DefaultNewDetection detector = new 一DefaultNewDetection();
    CameraSubsystem camera = new CameraSubsystem(this, detector);


    Vector2d preLoadDropPosition = new Vector2d(20, -64);

    @Override
    public void runOpMode() {
        // on start
        arm.init(hardwareMap, telemetry, box, turret, flipper, distanceSensor, intake);
        distanceSensor.init(hardwareMap, telemetry);
        box.init(hardwareMap, telemetry);
        carousel.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
        turret.init(hardwareMap, telemetry);
        flipper.init(hardwareMap,telemetry);
        flipper.flipperServo.setPosition(FlipperSubsystem.FLIPPER_CLOSED);

        TuningStart.initializeTuning();
        Robot drive = new Robot(this);
        /* Open CV */
//        camera.openCvRun();

        sleep(AutoValues.CAMERA_WAIT_TIME);

        preLoadDropPosition = camera.redWarehouseDetection(detector);

        telemetry.addData("Auto Position", detector.getAnalysis());
        telemetry.addLine("Ready to Start");
        telemetry.addLine("This auto better not fail pls pls pls");
        telemetry.update();

//        telemetrySubsystem.initMessage();

        waitForStart();

//        telemetrySubsystem.resetRuntime();

        if(isStopRequested()) return;

        Pose2d startPose = new Pose2d(7, -61, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                // go to hub
                .setReversed(true)
                .addDisplacementMarker(() -> command.armHigh.start())
                .splineTo(new Vector2d(-12, -45), Math.toRadians(90), Trajectories.splineStartConstraint, Trajectories.accelConstraint )
//                .splineTo(new Vector2d(0, -44), Math.toRadians(120))
                .build();


        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                //spline in
                .setReversed(false)
                .splineTo(preLoadDropPosition, Math.toRadians(0), Trajectories.splineInConstraint, Trajectories.accelConstraint)

                //back forth
                .addDisplacementMarker(() -> command.intakeIn.start())
                .addDisplacementMarker(() -> command.colorSensorRun.start())
                .forward(40, Trajectories.warehouseInConstraint, Trajectories.accelConstraint)

                // move out arm after 10 inches


                .back(40,  Trajectories.warehouseOutConstraint, Trajectories.accelConstraint)
                .addDisplacementMarker(() -> command.armCycle.start())

                .addDisplacementMarker(() -> command.intakeStop.start())


                //spline out
                .setReversed(true)
                .splineTo(new Vector2d(-16, -42), Math.toRadians(-200)) // reversed
//                .splineTo(new Vector2d(-15, -44), Math.toRadians(-200)) // reversed
                .build();

        TrajectorySequence Trajectory3 = drive.trajectorySequenceBuilder(Trajectory2.end())
                // spline in
                .setReversed(false)
                .splineTo(new Vector2d(20, -69), Math.toRadians(0), Trajectories.splineInConstraint, Trajectories.accelConstraint)

                //back forth
                .addDisplacementMarker(() -> command.intakeIn.start())
                .addDisplacementMarker(() -> command.colorSensorRun.start())
                .forward(40, Trajectories.warehouseInConstraint, Trajectories.accelConstraint)



                .back(40,  Trajectories.warehouseOutConstraint, Trajectories.accelConstraint)
                .addDisplacementMarker(() -> command.intakeStop.start())
                .addDisplacementMarker(() -> command.armCycle.start())



                // spline out
                .setReversed(true)
                .splineTo(new Vector2d(-16, -45), Math.toRadians(-200)) // reversed
                .build();

        TrajectorySequence Trajectory4 = drive.trajectorySequenceBuilder(Trajectory3.end())
                // spline in
                .setReversed(false)
                .splineTo(new Vector2d(20, -74), Math.toRadians(0), Trajectories.splineInConstraint, Trajectories.accelConstraint)

                //back forth
                .addDisplacementMarker(() -> command.intakeIn.start())
                .addDisplacementMarker(() -> command.colorSensorRun.start())
                .forward(40, Trajectories.warehouseInConstraint, Trajectories.accelConstraint)

//                // move out arm after 10 inches
//                .addDisplacementMarker(() -> armCycleCommand.start())

                .back(40,  Trajectories.warehouseOutConstraint, Trajectories.accelConstraint)
                .addDisplacementMarker(() -> command.armCycle.start())

                .addDisplacementMarker(() -> command.intakeStop.start())




                // spline out
                .setReversed(true)
                .splineTo(new Vector2d(-16, -47), Math.toRadians(-200)) // reversed
                .build();



        // Run trajectory 1
//        colorSensorCommand.start();
//        sleep(10000);

        drive.followTrajectorySequence((Trajectory1));
        dropSequence();


        drive.followTrajectorySequence((Trajectory2));
        dropSequence();

        drive.followTrajectorySequence((Trajectory3));
        dropSequence();

        while (opModeIsActive()) {
            drive.followTrajectorySequence((Trajectory4));
            dropSequence();
        }

    }
    public void dropSequence() {
        sleep((long)CYCLE_DROP_WAIT);
        command.drop.start();
        sleep((long)CYCLE_DROP_WAIT2);
        command.armIn.start();
    }
}

