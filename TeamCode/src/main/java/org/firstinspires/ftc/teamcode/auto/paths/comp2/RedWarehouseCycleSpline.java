package org.firstinspires.ftc.teamcode.auto.paths.comp2;

import static org.firstinspires.ftc.teamcode.auto.paths.comp2.AutoValues.CYCLE_DROP_WAIT;
import static org.firstinspires.ftc.teamcode.auto.paths.comp2.AutoValues.CYCLE_DROP_WAIT2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.pipeline.一DefaultDetection;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BoxSubsystem;
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
@Autonomous(name="Red warehouse cycle spline REAL Roadrunner Path", group="Roadrunner Paths")
public class RedWarehouseCycleSpline extends LinearOpMode {
    ElapsedTime runtime  = new ElapsedTime();
    ArmSubsystem arm                            = new ArmSubsystem                          ();
    BoxSubsystem box                            = new BoxSubsystem                          ();
    CarouselSubsystem carousel                  = new CarouselSubsystem                     ();
    IntakeSubsystem intake                      = new IntakeSubsystem                       ();
    //    TelemetrySubsystem telemetrySubsystem       = new TelemetrySubsystem                    ();
    TurretSubsystem turret                      = new TurretSubsystem                       ();
    FlipperSubsystem flipper                    = new FlipperSubsystem                      ();
    DistanceSensorSubsystem distanceSensor      = new DistanceSensorSubsystem               ();
    public static Vector2d splineOutPos = new Vector2d(1, 1);




    //    Runnable armForwardStart = () -> {
//        double targetTime = runtime.seconds() + 3;
//        arm.armUp(ArmSubsystem.ARM_HIGH_POS, BoxSubsystem.BOX_HIGH, AutoValues.AUTO_TURRET_BLUE_WAREHOUSE_STARt, false);
//        while (targetTime > runtime.seconds()) {
//            arm.loopCommand();
//        }
//    };
    Thread armForwardStartCommand = new Thread(() -> {
        double targetTime = runtime.seconds() + 2;
        arm.armUp(ArmSubsystem.ARM_HIGH_POS, BoxSubsystem.BOX_HIGH, AutoValues.AUTO_TURRET_RED_WAREHOUSE_START, false);
        while (targetTime > runtime.seconds()) {
            arm.loopCommand();
        }
    });

    Thread armInCommand = new Thread(() -> {
        double targetTime = runtime.seconds() + 3;
        arm.sensorIsDisabled = false;
        arm.armReset();
        while (targetTime > runtime.seconds()) {
            arm.loopCommand();
        }
    });

    Thread armCycleCommand = new Thread(() -> {
        double targetTime = runtime.seconds() + 2;
        arm.armUp(ArmSubsystem.ARM_HIGH_POS, BoxSubsystem.BOX_HIGH, AutoValues.AUTO_TURRET_POSITION_RED,true);
        while (targetTime > runtime.seconds()) {
            arm.loopCommand();
        }
    });

    Thread dropCommand = new Thread(() -> flipper.flipperServo.setPosition(FlipperSubsystem.FLIPPER_OPEN));
//    Thread dropCommand = new Thread(drop);

    Thread intakeInCommand = new Thread(() -> intake.intakeMotor.setPower(-IntakeSubsystem.INTAKE_POWER));
//    Thread intakeInCommand = new Thread(intakeIn);

    Thread intakeStopCommand = new Thread(() -> intake.intakeMotor.setPower(0));
//    Thread intakeStopCommand = new Thread(intakeStop);

    Runnable colorSensor = () -> {
        arm.detected = false;
        while (!arm.detected) {
            if (distanceSensor.distanceSensor.getDistance(DistanceUnit.MM) < DistanceSensorSubsystem.DISTANCE_THRESHOLD && !arm.sensorIsDisabled) {
                flipper.flipperServo.setPosition(FlipperSubsystem.FLIPPER_CLOSED);
                intake.intakeMotor.setPower(IntakeSubsystem.INTAKE_POWER);
                arm.detected = true;
            }
            if(isStopRequested()) return;
        }
    };
    Thread colorSensorCommand = new Thread(colorSensor);
    Thread carouselBlue = new Thread(() -> carousel.carouselMotor.setPower(1));




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

        Pose2d startPose = new Pose2d(7, -61, Math.toRadians(-90));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                // go to hub
                .setReversed(true)
                .addDisplacementMarker(() -> armForwardStartCommand.start())
                .splineTo(new Vector2d(-12, -45), Math.toRadians(90), Trajectories.splineStartConstraint, Trajectories.accelConstraint )
//                .splineTo(new Vector2d(0, -44), Math.toRadians(120))
                .build();


        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                //spline in
                .setReversed(false)
                .splineTo(new Vector2d(20, -64), Math.toRadians(0), Trajectories.splineInConstraint, Trajectories.accelConstraint)

                //back forth
                .addDisplacementMarker(() -> intakeInCommand.start())
                .addDisplacementMarker(() -> colorSensorCommand.start())
                .forward(40, Trajectories.warehouseInConstraint, Trajectories.accelConstraint)

                // move out arm after 10 inches


                .back(40,  Trajectories.warehouseOutConstraint, Trajectories.accelConstraint)
                .addDisplacementMarker(() -> armCycleCommand.start())

                .addDisplacementMarker(() -> intakeStopCommand.start())


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
                .addDisplacementMarker(() -> intakeInCommand.start())
                .addDisplacementMarker(() -> colorSensorCommand.start())
                .forward(40, Trajectories.warehouseInConstraint, Trajectories.accelConstraint)



                .back(40,  Trajectories.warehouseOutConstraint, Trajectories.accelConstraint)
                .addDisplacementMarker(() -> intakeStopCommand.start())
                .addDisplacementMarker(() -> armCycleCommand.start())



                // spline out
                .setReversed(true)
                .splineTo(new Vector2d(-16, -45), Math.toRadians(-200)) // reversed
                .build();

        TrajectorySequence Trajectory4 = drive.trajectorySequenceBuilder(Trajectory3.end())
                // spline in
                .setReversed(false)
                .splineTo(new Vector2d(20, -74), Math.toRadians(0), Trajectories.splineInConstraint, Trajectories.accelConstraint)

                //back forth
                .addDisplacementMarker(() -> intakeInCommand.start())
                .addDisplacementMarker(() -> colorSensorCommand.start())
                .forward(40, Trajectories.warehouseInConstraint, Trajectories.accelConstraint)

//                // move out arm after 10 inches
//                .addDisplacementMarker(() -> armCycleCommand.start())

                .back(40,  Trajectories.warehouseOutConstraint, Trajectories.accelConstraint)
                .addDisplacementMarker(() -> armCycleCommand.start())

                .addDisplacementMarker(() -> intakeStopCommand.start())




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
        dropCommand.start();
        sleep((long)CYCLE_DROP_WAIT2);
        armInCommand.start();
    }
}

