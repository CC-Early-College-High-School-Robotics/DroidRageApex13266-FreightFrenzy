package org.firstinspires.ftc.teamcode.auto.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.duckDetection;

import org.firstinspires.ftc.teamcode.hardware.Hardware13266;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Drive13266;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Blue Carosel Road Runner Path", group="Road Runner Paths")
public class BlueCarosel extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drive13266 drive = new Drive13266(hardwareMap);
        Hardware13266 robot = new Hardware13266();
        robot.init(hardwareMap);

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

                camera.setPipeline(new duckDetection());
            }
            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });









        Pose2d startPose = new Pose2d(-47, 60, Math.toRadians(270));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .turn(Math.toRadians(30)) // heading = 300
                .back(6)
                .build();

        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .forward(6)
                .turn(Math.toRadians(-210)) // heading = 90
                .back(27)
                .turn(Math.toRadians(90)) // heading = 180
                .build();

        TrajectorySequence Trajectory3 = drive.trajectorySequenceBuilder(Trajectory2.end())
                .back(23)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        // Run trajectory 1
        drive.followTrajectorySequence((Trajectory1));

        // Run carousel
        robot.carouselMotor.setPower(robot.CAROUSEL_POWER);
        sleep(2500);
        robot.carouselMotor.setPower(0);

        // Run trajectory 2
        drive.followTrajectorySequence((Trajectory2));

        // Lift Arm
        robot.armMotor.setPower(robot.ARM_POWER);
        robot.setArmPosition(robot.ARM_HIGH_POS);

        // Run Trajectory 3
        drive.followTrajectorySequence((Trajectory3));

        // Drop freight
        robot.boxServo.setPosition(robot.BOX_DROP);
        sleep(1000);

        // Put arm down
        robot.boxServo.setPosition(robot.BOX_INTAKE);
        robot.setArmPosition(robot.ARM_DOWN_POS);
        sleep(2000);
    }
}

