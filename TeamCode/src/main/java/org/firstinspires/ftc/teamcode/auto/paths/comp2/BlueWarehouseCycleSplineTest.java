package org.firstinspires.ftc.teamcode.auto.paths.comp2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.一BlueCarouselDuckDetection;
import org.firstinspires.ftc.teamcode.auto.pipeline.一DefaultDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerImprovedTankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequenceimproved.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teleop.testing.TuningStart;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="Blue warehouse cycle spline test Roadrunner Path", group="Roadrunner Paths")
public class BlueWarehouseCycleSplineTest extends LinearOpMode {
    public static double MOTORVELOF = 1.0;
    public static double MOTORVELOFBACKFORTH = 0.5;
    public static double MOTORVELOFSPLINEIN = 0.5;
    public static double MOTORVELOF3 = 1.0;

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
        RoadRunnerImprovedTankDrive drive = new RoadRunnerImprovedTankDrive(this);
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
                telemetry.addData("you should see this", "ill be mad if you dont");

//                arm.loopCommand();
//                telemetrySubsystem.defaultCommand();

                telemetry.update();
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status", "Camera failed :(");
                telemetry.update();
            }
        });

//        telemetrySubsystem.initMessage();

        waitForStart();

//        telemetrySubsystem.resetRuntime();

        if(isStopRequested()) {
            AutoBooleans.isActive = false;
            return;
        }

        Pose2d startPose = new Pose2d(-11, 62.5, Math.toRadians(90));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .back(15)
                .build();


        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .turn(Math.toRadians(-50))
                .build();

        TrajectorySequence Trajectory3 = drive.trajectorySequenceBuilder(Trajectory2.end())
                .setReversed(false)
                .splineTo(new Vector2d(20, 63), Math.toRadians(0))
                .build();

        TrajectorySequence Forward = drive.trajectorySequenceBuilder(Trajectory3.end())
                .forward(24)
                .build();

        TrajectorySequence Backward = drive.trajectorySequenceBuilder(Forward.end())
                .back(24)
                .build();

        TrajectorySequence SplineOut = drive.trajectorySequenceBuilder(Backward.end())

                .setReversed(true)
                .splineTo(new Vector2d(-15, 44), Math.toRadians(200)) // reversed
                .build();

        TrajectorySequence SplineIn = drive.trajectorySequenceBuilder(SplineOut.end())
                .setReversed(false)
                .splineTo(new Vector2d(20, 63), Math.toRadians(0))
                .build();

        /*
        TrajectorySequence Trajectory5 = drive.trajectorySequenceBuilder(Trajectory4.end())

                .build();

        TrajectorySequence Trajectory6 = drive.trajectorySequenceBuilder(Trajectory5.end())
                .forward(15)
                .build();

         */



        // Run trajectory 1
        AutoBooleans.armForward = true;
        drive.setMotorRPMFraction(MOTORVELOF);
        drive.followTrajectorySequence((Trajectory1));
        AutoBooleans.openFlipper = true;

        drive.setMotorRPMFraction(MOTORVELOFBACKFORTH);
        drive.followTrajectorySequence((Trajectory2));
        AutoBooleans.armIntake = true;

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

