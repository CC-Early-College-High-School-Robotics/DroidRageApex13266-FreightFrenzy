package org.firstinspires.ftc.teamcode.auto.paths.comp2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.一DefaultDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequenceimproved.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teleop.testing.TuningStart;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="Red warehouse cycle spline test Roadrunner Path", group="Roadrunner Paths")
public class RedWarehouseCycleSplineTest extends LinearOpMode {
    public static long CYCLE_DROP_WAIT = 1000;


    // Declare subsystems
//    ArmSubsystem arm                            = new ArmSubsystem                          ();
//    BoxSubsystem box                            = new BoxSubsystem                          ();
//    CarouselSubsystem carousel                  = new CarouselSubsystem                     ();
//    IntakeSubsystem intake                      = new IntakeSubsystem                       ();
//    TelemetrySubsystem telemetrySubsystem       = new TelemetrySubsystem                    ();
//    TurretSubsystem turret                      = new TurretSubsystem                       ();
//    FlipperSubsystem flipper                    = new FlipperSubsystem                      ();
//    DistanceSensorSubsystem distanceSensor      = new DistanceSensorSubsystem               ();

    //    ArmForwardCommand armForwardCommand = new ArmForwardCommand(arm);
    AutoCommandThread autoThread = new AutoCommandThread(this);

    @Override
    public void runOpMode() {
        autoThread.start();
        TuningStart.initializeTuning();
        Robot drive = new Robot(this);
        // On start
//        arm.init(hardwareMap, telemetry, box, turret, flipper, distanceSensor, intake);
//        distanceSensor.init(hardwareMap, telemetry);
//        box.init(hardwareMap, telemetry);
//        carousel.init(hardwareMap, telemetry);
//        intake.init(hardwareMap, telemetry);
//        turret.init(hardwareMap, telemetry);
//        flipper.init(hardwareMap,telemetry);
//
//        telemetrySubsystem.init(telemetry, arm, box, carousel, drivetrain, intake, distanceSensor);

        /* Open CV */

        一DefaultDetection detector = new 一DefaultDetection();


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
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);

                camera.setPipeline(detector);

            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status", "Camera failed :(");
            }
        });
        sleep(AutoValues.CAMERA_WAIT_TIME);
        telemetry.addData("auto Position", detector.getAnalysis());

//        telemetrySubsystem.initMessage();

        waitForStart();

//        telemetrySubsystem.resetRuntime();

        if(isStopRequested()) {
            AutoBooleans.isActive = false;
            return;
        }

        Pose2d startPose = new Pose2d(7, -62.5, Math.toRadians(-90));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .back(15)
                .build();


        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .turn(Math.toRadians(50))
                .setReversed(false)
                .splineTo(new Vector2d(20, -65), Math.toRadians(0))

                //back forth
                .forward(24)
                .back(24)

                //spline out
                .setReversed(true)
                .splineTo(new Vector2d(-15, -44), Math.toRadians(-200)) // reversed
                .build();
        TrajectorySequence Trajectory3 = drive.trajectorySequenceBuilder(Trajectory2.end())
                .setReversed(false)
                .splineTo(new Vector2d(20, -65), Math.toRadians(0))
                .intake()
                .forward(24)
                .back(24)
                .setReversed(true)
                .splineTo(new Vector2d(-15, -44), Math.toRadians(-200)) // reversed
                .build();

        TrajectorySequence Trajectory4 = drive.trajectorySequenceBuilder(Trajectory3.end())
                .setReversed(false)
                .splineTo(new Vector2d(20, -65), Math.toRadians(0))
                .intake()
                .forward(24)
                .back(24)
                .setReversed(true)
                .splineTo(new Vector2d(-15, -44), Math.toRadians(-200)) // reversed
                .build();



        // Run trajectory 1
        AutoBooleans.armRedWarehouseStart = true;
        drive.followTrajectorySequence((Trajectory1));
        sleep(1000);
        AutoBooleans.openFlipper = true;
        sleep(1000);
        AutoBooleans.armIntake = true;

        drive.followTrajectorySequence((Trajectory2));
        AutoBooleans.armToAllianceHubRed = true;
        sleep(CYCLE_DROP_WAIT);
        AutoBooleans.armIntake = true;

        drive.followTrajectorySequence((Trajectory3));
        AutoBooleans.armToAllianceHubRed = true;
        sleep(CYCLE_DROP_WAIT);
        AutoBooleans.armIntake = true;
        drive.followTrajectorySequence((Trajectory4));
        AutoBooleans.armToAllianceHubRed = true;
        sleep(CYCLE_DROP_WAIT);
        AutoBooleans.armIntake = true;
        drive.followTrajectorySequence((Trajectory4));
        AutoBooleans.armToAllianceHubRed = true;
        sleep(CYCLE_DROP_WAIT);
        AutoBooleans.armIntake = true;
        drive.followTrajectorySequence((Trajectory4));
        AutoBooleans.armToAllianceHubRed = true;
        sleep(CYCLE_DROP_WAIT);
        AutoBooleans.armIntake = true;
    }
}

