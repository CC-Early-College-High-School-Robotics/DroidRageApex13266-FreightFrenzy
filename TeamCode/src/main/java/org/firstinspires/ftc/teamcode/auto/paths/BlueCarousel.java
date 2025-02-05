package org.firstinspires.ftc.teamcode.auto.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.一BlueCarouselDuckDetection;

import org.firstinspires.ftc.teamcode.hardware.一AutoValues;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RoadrunnerTankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teleop.testing.TuningStart;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Blue Carousel (Bottom) Roadrunner Path", group="Roadrunner Paths")
public class BlueCarousel extends LinearOpMode {
    @Override
    public void runOpMode() {
        TuningStart.initializeTuning();
        RoadrunnerTankDrive drive = new RoadrunnerTankDrive(hardwareMap);
        Devices robot = new Devices();
        robot.init(hardwareMap);

        double armHeight = 0;
        double hubDistance = 0;

        // move camera
        robot.cameraServo.setPosition(Devices.CAMERA_BLUE_CAROUSEL_POS);

        一BlueCarouselDuckDetection detector = new 一BlueCarouselDuckDetection();


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
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);

                camera.setPipeline(detector);
                telemetry.addData("you shoudl see this", "ill be mad if you dont");
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status", "Camera failed :(");
            }
        });
        sleep(robot.CAMERA_WAIT_TIME);
        // camera dection print
        telemetry.addData("Duck position", detector.getAnalysis());
        telemetry.addData("hi", "hi");


        // Before start

        // Lift box up
        robot.boxServo.setPosition(Devices.BOX_UP);

        // On start

        waitForStart();
        if(isStopRequested()) return;

        if (detector.getAnalysis() == 一BlueCarouselDuckDetection.DuckPosition.RIGHT) {
            armHeight = Devices.ARM_HIGH_POS;
            hubDistance = 一AutoValues.BLUE_CAROUSEL_HIGH;
        }

        if (detector.getAnalysis() == 一BlueCarouselDuckDetection.DuckPosition.CENTER) {
            armHeight = Devices.ARM_MID_POS;
            hubDistance = 一AutoValues.BLUE_CAROUSEL_MID;
        }

        if (detector.getAnalysis() == 一BlueCarouselDuckDetection.DuckPosition.LEFT) {
            armHeight = Devices.ARM_LOW_POS;
            hubDistance = 一AutoValues.BLUE_CAROUSEL_LOW;;

        }









        Pose2d startPose = new Pose2d(-47, 60, Math.toRadians(270));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .turn(Math.toRadians(30), 4, 4)
                .build();

        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .back(9)
                .build();

        TrajectorySequence Trajectory3 = drive.trajectorySequenceBuilder(Trajectory2.end())
                .forward(9)
                .turn(Math.toRadians(60), 4, 4)
                .back(6)
                .turn(Math.toRadians(90))
                .back(30)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence Trajectory4 = drive.trajectorySequenceBuilder(Trajectory3.end())
                .back(hubDistance)
                .build();

        TrajectorySequence Trajectory5 = drive.trajectorySequenceBuilder(Trajectory4.end())
                .forward(hubDistance + 4)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence Trajectory6 = drive.trajectorySequenceBuilder(Trajectory5.end())
                .forward(15)
                .build();

        camera.stopStreaming();


        // Run trajectory 1
        drive.followTrajectorySequence((Trajectory1));

        // Run carousel
        robot.carouselMotor.setPower(Devices.CAROUSEL_SLOW_POWER);


        // Run trajectory 2
        drive.followTrajectorySequence((Trajectory2));
        sleep(2000);

        robot.carouselMotor.setPower(0);

        // Run trajectory 3
        drive.followTrajectorySequence((Trajectory3));

        // Lift Arm
        robot.armMotor.setPower(Devices.ARM_SLOW_POWER);
        robot.setArmPosition(armHeight);

        // Lift box forward
        robot.boxServo.setPosition(Devices.BOX_AUTO_APPROACH_HUB);

        // Run Trajectory 4
        drive.followTrajectorySequence((Trajectory4));

        // Drop freight
        robot.boxServo.setPosition(Devices.BOX_DROP);
        sleep(1000);




        // Run Trajectory 5
        drive.followTrajectorySequence((Trajectory5));

        // Return box
        robot.boxServo.setPosition(Devices.BOX_INTAKE);


        // Run Trajectory 6
        drive.followTrajectorySequence((Trajectory6));

        //return arm
        robot.setArmPosition(Devices.ARM_NEUTRAL_POS);
        sleep(2000);
    }
}

