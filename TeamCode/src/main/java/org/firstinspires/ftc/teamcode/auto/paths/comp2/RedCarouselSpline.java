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
@Autonomous(name="Red Carousel Spline", group="Roadrunner Paths")
public class RedCarouselSpline extends LinearOpMode {
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
    CameraSubsystem cameraSubsystem = new CameraSubsystem(this, detector);
    Thread preLoadArmCommand = command.armHigh;


    double preLoadDropBackAmount = 0;

    @Override
    public void runOpMode() {
        // on start
        arm.init(hardwareMap, telemetry, box, turret, flipper, distanceSensor, intake);
        distanceSensor.init(hardwareMap, telemetry);
        box.init(hardwareMap, telemetry);
        carousel.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
        turret.init(hardwareMap, telemetry);
        flipper.init(hardwareMap, telemetry);
        flipper.flipperServo.setPosition(FlipperSubsystem.FLIPPER_CLOSED);

        TuningStart.initializeTuning();
        Robot drive = new Robot(this);
        /* Open CV */
        // Obtain camera id to allow for camera preview
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Obtain webcam name
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Initialize OpenCvWebcam
        // With live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);
        // Open the Camera Device Asynchronously
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start Camera Streaming

                // NOTE: this must be called *before* you call startStreaming(...)
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

                // Start camera stream with 1280x720 resolution
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

                camera.setPipeline(detector);

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status", "Camera failed :(");
                telemetry.update();
            }
        });

        sleep(AutoValues.CAMERA_WAIT_TIME);





        telemetry.addData("Auto Position", detector.getAnalysis());
        telemetry.addLine("Ready to Start");
        telemetry.addLine("This auto better not fail pls pls pls");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

//        switch (detector.getAnalysis()) {
//            case CENTER:
//                preLoadDropBackAmount = AutoValues.BLUE_CAROUSEL_MID;
//                preLoadArmCommand = command.armMid;
//            case LEFT:
//                preLoadDropBackAmount = AutoValues.BLUE_CAROUSEL_LOW;
//                preLoadArmCommand = command.armLow;
//            default:
//                preLoadDropBackAmount = 0;
//                preLoadArmCommand = command.armHigh;
//        }
        if (detector.getAnalysis() == 一DefaultNewDetection.DuckPosition.RIGHT) {
            preLoadDropBackAmount = 0;
            preLoadArmCommand = command.armHigh;
        }

        if (detector.getAnalysis() == 一DefaultNewDetection.DuckPosition.CENTER) {

            preLoadDropBackAmount = AutoValues.BLUE_CAROUSEL_MID;
            preLoadArmCommand = command.armMid;
        }

        if (detector.getAnalysis() == 一DefaultNewDetection.DuckPosition.LEFT) {
            preLoadDropBackAmount = AutoValues.BLUE_CAROUSEL_LOW;
            preLoadArmCommand = command.armLow;
        }

        Pose2d startPose = new Pose2d(-40, -62.5, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)

                .back(15)
                .addDisplacementMarker(20, () -> command.carouselBlueRun.start())
                .turn(Math.toRadians(-60))
                .forward(25, Trajectories.warehouseConstraint, Trajectories.accelConstraint)
                .forward(5, Trajectories.imDyingItsSoSlowSpeedConstraint, Trajectories.accelConstraint)
//                .setReversed(false)
//                .splineTo(new Vector2d(-61, 55), Math.toRadians(180), Trajectories.WarehouseConstraint, Trajectories.accelConstraint)

//                .addCommand(() -> command.carouselBlueRun.start())
                .waitSeconds(0.5)
                .addCommand(() -> command.carouselStop.start())

                .turn(Math.toRadians(50))


                .setReversed(true)

                .addDisplacementMarker(58, () -> preLoadArmCommand.start())
                .splineTo(new Vector2d(-57, -24), Math.toRadians(0), Trajectories.warehouseConstraint, Trajectories.accelConstraint)

                .back(24 + preLoadDropBackAmount, Trajectories.warehouseSLowConstraint, Trajectories.accelConstraint)

//                .addCommand(() -> command.drop.start())
//                .waitSeconds(2)
//                .addCommand(() -> command.drop.start())

//                .setReversed(false)
//                .addCommand(() -> command.armIn.start())
//                .splineTo(new Vector2d(-60, 38), Math.toRadians(90), Trajectories.WarehouseConstraint, Trajectories.accelConstraint)

//                .waitSeconds(5)
                .build();
        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .setReversed(false)
                .addCommand(() -> command.armIn.start())
                .splineTo(new Vector2d(-60, -38), Math.toRadians(-90), Trajectories.warehouseConstraint, Trajectories.accelConstraint)
                .build();



        // Run trajectory 1

        drive.followTrajectorySequence((Trajectory1));
        command.drop.start();
        sleep(1000);
        drive.followTrajectorySequence((Trajectory2));
    }

}

