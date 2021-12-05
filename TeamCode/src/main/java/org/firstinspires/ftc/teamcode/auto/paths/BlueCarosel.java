package org.firstinspires.ftc.teamcode.auto.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.DuckDetection;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MechanumDriveRoadRunner;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

@Autonomous(name="Blue Carousel Roadrunner Path", group="Roadrunner Paths")
public class BlueCarosel extends LinearOpMode {
    @Override
    public void runOpMode() {
        MechanumDriveRoadRunner drive = new MechanumDriveRoadRunner(hardwareMap);
        Devices robot = new Devices();
        robot.init(hardwareMap);

        double armHeight = 0;
        String duckPos = "RIGHT";

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

                camera.setPipeline(new DuckDetection());
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status", "Camera failed :(");
            }
        });

        if (new DuckDetection().getAnalysis().equals(DuckDetection.DuckPosition.RIGHT)) {
            armHeight = robot.ARM_HIGH_POS;
            duckPos = "RIGHT";

        }

        if (new DuckDetection().getAnalysis().equals(DuckDetection.DuckPosition.CENTER)) {
            armHeight = robot.ARM_MID_POS;
            duckPos = "RIGHT";
        }

        if (new DuckDetection().getAnalysis().equals(DuckDetection.DuckPosition.LEFT)) {
            armHeight = robot.ARM_LOW_POS;
            duckPos = "RIGHT";


        }


        Pose2d startPose = new Pose2d(-47, 60, Math.toRadians(270));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .turn(Math.toRadians(45))
                .back(6.5)
                .build();

        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .forward(6.5)
                .turn(Math.toRadians(180))
                .build();


        Trajectory Trajectory3 = drive.trajectoryBuilder(Trajectory2.end(), true)
                .splineTo(new Vector2d(-25, 35), Math.toRadians(135))
                .build();

        Trajectory Trajectory4 = drive.trajectoryBuilder(Trajectory3.end())
                .splineTo(new Vector2d(-60, 35), Math.toRadians(180))
                .build();

        TrajectorySequence Trajectory5 = drive.trajectorySequenceBuilder(Trajectory4.end())
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence Trajectory6 = drive.trajectorySequenceBuilder(Trajectory5.end())
                .forward(10)
                .build();

        TrajectorySequence Trajectory7 = drive.trajectorySequenceBuilder(Trajectory6.end())
                .back(10)
                .turn(Math.toRadians(90))
                //.back(intakeAmount)
                .build();

        // Before start

        // Move camera
        robot.cameraServo.setPosition(0.25);

        // Lift box up
        robot.boxServo.setPosition(robot.BOX_UP);

        telemetry.addData("Duck position", new DuckDetection().getAnalysis());

        // On start
        waitForStart();

        if (isStopRequested()) return;


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
        robot.setArmPosition(armHeight);

        // Run Trajectory 3
        drive.followTrajectory((Trajectory3));

        // Drop freight
        robot.boxServo.setPosition(robot.BOX_DROP);
        sleep(1000);
        /*

        // Run Trajectory 4
        drive.followTrajectory((Trajectory4));

        // Put arm down
        robot.boxServo.setPosition(robot.BOX_INTAKE);
        robot.setArmPosition(robot.ARM_NEUTRAL_POS);



        // Run Trajectory 5
        drive.followTrajectorySequence((Trajectory5));

        // Turn on intake
        robot.intakeMotor.setVelocity(robot.INTAKE_VELOCITY);

        // Run Trajectory 6
        drive.followTrajectorySequence((Trajectory6));

        // Turn on intake
        robot.intakeMotor.setVelocity(0);

        // Lift Arm
        robot.armMotor.setPower(robot.ARM_POWER);
        robot.setArmPosition(armHeight);

        // Run Trajectory 7
        drive.followTrajectorySequence((Trajectory7));

        // Drop freight
        robot.boxServo.setPosition(robot.BOX_DROP);
        sleep(1000);
*/
        // Put arm down for TeleOp
        robot.boxServo.setPosition(robot.BOX_INTAKE);
        robot.setArmPosition(robot.ARM_NEUTRAL_POS);


    }
}

