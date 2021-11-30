package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Drive13266;
import org.firstinspires.ftc.teamcode.hardware.Hardware13266;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Blue Carosel Road Runner Path", group="Road Runner Paths")
public class BlueCarosel extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drive13266 drive = new Drive13266(hardwareMap);
        Hardware13266 robot = new Hardware13266();
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(-47, 60, Math.toRadians(270));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .turn(Math.toRadians(30)) // heading = 300
                .back(7)
                .build();

        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .forward(6)
                .turn(Math.toRadians(-210)) // heading = 90
                .back(30)
                .turn(Math.toRadians(90)) // heading = 180
                .back(26)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence((Trajectory1));
        robot.carouselMotor.setPower(robot.CAROUSEL_POWER);
        sleep(5000);
        robot.carouselMotor.setPower(0);
        drive.followTrajectorySequence((Trajectory2));
        robot.armMotor.setPower(robot.ARM_POWER);
        robot.setArmPosition(robot.ARM_HIGH_POS);
        sleep(2000);
        robot.boxServo.setPosition(robot.BOX_DROP);
        sleep(2000);
    }
}

