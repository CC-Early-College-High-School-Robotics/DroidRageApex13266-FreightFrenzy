package org.firstinspires.ftc.teamcode.auto.paths.comp2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.一DefaultDetection;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BoxSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.FlipperSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.TurretSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequenceimproved.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teleop.testing.TuningStart;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="Blue warehouse cycle spline test Roadrunner Path", group="Roadrunner Paths")
public class BlueWarehouseCycleSplineTest extends LinearOpMode {
    ElapsedTime runtime  = new ElapsedTime();
    ArmSubsystem arm                            = new ArmSubsystem                          ();
    BoxSubsystem box                            = new BoxSubsystem                          ();
    CarouselSubsystem carousel                  = new CarouselSubsystem                     ();
    IntakeSubsystem intake                      = new IntakeSubsystem                       ();
    //    TelemetrySubsystem telemetrySubsystem       = new TelemetrySubsystem                    ();
    TurretSubsystem turret                      = new TurretSubsystem                       ();
    FlipperSubsystem flipper                    = new FlipperSubsystem                      ();
    DistanceSensorSubsystem distanceSensor      = new DistanceSensorSubsystem               ();

    public static long CYCLE_DROP_WAIT = 1000;


    Thread armForwardStart = new Thread(() -> {
        double targetTime = runtime.seconds() + 3;
        arm.armUp(ArmSubsystem.ARM_HIGH_POS, BoxSubsystem.BOX_HIGH, AutoValues.AUTO_TURRET_BLUE_WAREHOUSE_STARt, false);
        while (targetTime > runtime.seconds()) {
            arm.loopCommand();
        }
    });

    Thread armCycleCommand = new Thread(() -> {
        double targetTime = runtime.seconds() + 3;
        arm.armUp(ArmSubsystem.ARM_HIGH_POS, BoxSubsystem.BOX_HIGH, AutoValues.AUTO_TURRET_POSITION_BLUE,false);
        while (targetTime > runtime.seconds()) {
            arm.loopCommand();
        }
    });

    Thread dropCommand = new Thread(() -> {
        flipper.flipperServo.setPosition(FlipperSubsystem.FLIPPER_OPEN);
    });

    Thread intakeInCommand = new Thread(() -> {
        intake.intakeMotor.setPower(-IntakeSubsystem.INTAKE_POWER);
    });

    Thread intakeStopCommand = new Thread(() -> {
        intake.intakeMotor.setPower(0);
    });



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

//        autoThread.start();
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

        if(isStopRequested()) return;

        Pose2d startPose = new Pose2d(7, 62.5, Math.toRadians(90));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    armForwardStart.start();
                })
                .back(15)
                .turn(Math.toRadians(-50))
                .build();


        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .setReversed(false)
                .splineTo(new Vector2d(20, 65), Math.toRadians(0))

                //back forth
                .forward(24)
                .back(24)
                .addDisplacementMarker(() -> {
                    intakeStopCommand.start();
                })

                //spline out
                .setReversed(true)
                .splineTo(new Vector2d(-15, 44), Math.toRadians(200)) // reversed
                .build();
        TrajectorySequence Trajectory3 = drive.trajectorySequenceBuilder(Trajectory2.end())
                .setReversed(false)
                .splineTo(new Vector2d(20, 65), Math.toRadians(0))
                .intake()
                .forward(24)
                .back(24)
                .setReversed(true)
                .splineTo(new Vector2d(-15, 44), Math.toRadians(200)) // reversed
        .build();

        TrajectorySequence Trajectory4 = drive.trajectorySequenceBuilder(Trajectory3.end())
                .setReversed(false)
                .splineTo(new Vector2d(20, 65), Math.toRadians(0))
                .intake()
                .forward(24)
                .back(24)
                .setReversed(true)
                .splineTo(new Vector2d(-15, 44), Math.toRadians(200)) // reversed
                .build();



        // Run trajectory 1
        drive.followTrajectorySequence((Trajectory1));
        sleep(1000);
        dropCommand.start();
        sleep(1000);
        intakeInCommand.start();

        drive.followTrajectorySequence((Trajectory2));
        armCycleCommand.start();
        sleep(CYCLE_DROP_WAIT);
        AutoBooleans.armIntake = true;

        drive.followTrajectorySequence((Trajectory3));
        AutoBooleans.armToAllianceHubBlue = true;
        sleep(CYCLE_DROP_WAIT);
        AutoBooleans.armIntake = true;
        drive.followTrajectorySequence((Trajectory4));
        AutoBooleans.armToAllianceHubBlue = true;
        sleep(CYCLE_DROP_WAIT);
        AutoBooleans.armIntake = true;
        drive.followTrajectorySequence((Trajectory4));
        AutoBooleans.armToAllianceHubBlue = true;
        sleep(CYCLE_DROP_WAIT);
        AutoBooleans.armIntake = true;
        drive.followTrajectorySequence((Trajectory4));
        AutoBooleans.armToAllianceHubBlue = true;
        sleep(CYCLE_DROP_WAIT);
        AutoBooleans.armIntake = true;
    }
}

