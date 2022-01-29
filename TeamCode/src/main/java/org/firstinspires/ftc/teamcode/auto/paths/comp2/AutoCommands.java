package org.firstinspires.ftc.teamcode.auto.paths.comp2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BoxSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.FlipperSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.IntakeSubsystem;

public class AutoCommands {
    ArmSubsystem arm;
    FlipperSubsystem flipper;
    DistanceSensorSubsystem distanceSensor;
    ElapsedTime runtime = new ElapsedTime();
    IntakeSubsystem intake;
    CarouselSubsystem carousel;
    LinearOpMode opMode;

    public AutoCommands(LinearOpMode opMode, ArmSubsystem arm, FlipperSubsystem flipper, DistanceSensorSubsystem distanceSensor, IntakeSubsystem intake, CarouselSubsystem carousel) {
        this.opMode = opMode;
        this.arm = arm;
        this.flipper = flipper;
        this.distanceSensor = distanceSensor;
        this.intake = intake;
        this.carousel = carousel;
    }
    Thread armHigh = new Thread(() -> {
        double targetTime = runtime.seconds() + 2;
        arm.armUp(ArmSubsystem.ARM_HIGH_POS, BoxSubsystem.BOX_HIGH, AutoValues.AUTO_TURRET_RED_WAREHOUSE_START, false);
        while (targetTime > runtime.seconds()) {
            arm.loopCommand();
        }
    });
    Thread armMid = new Thread(() -> {
        double targetTime = runtime.seconds() + 2;
        arm.armUp(ArmSubsystem.ARM_MID_POS, BoxSubsystem.BOX_MID, AutoValues.AUTO_TURRET_RED_WAREHOUSE_START, false);
        while (targetTime > runtime.seconds()) {
            arm.loopCommand();
        }
    });
    Thread armLow = new Thread(() -> {
        double targetTime = runtime.seconds() + 2;
        arm.armUp(ArmSubsystem.ARM_LOW_POS, BoxSubsystem.BOX_LOW, AutoValues.AUTO_TURRET_RED_WAREHOUSE_START, false);
        while (targetTime > runtime.seconds()) {
            arm.loopCommand();
        }
    });

    Thread armIn = new Thread(() -> {
        double targetTime = runtime.seconds() + 3;
        arm.sensorIsDisabled = false;
        arm.armReset();
        while (targetTime > runtime.seconds()) {
            arm.loopCommand();
        }
    });

    Thread armCycle = new Thread(() -> {
        double targetTime = runtime.seconds() + 2;
        arm.armUp(ArmSubsystem.ARM_HIGH_POS, BoxSubsystem.BOX_HIGH, AutoValues.AUTO_TURRET_POSITION_RED,true);
        while (targetTime > runtime.seconds()) {
            arm.loopCommand();
        }
    });

    Thread drop = new Thread(() -> flipper.flipperServo.setPosition(FlipperSubsystem.FLIPPER_OPEN));
//    Thread dropCommand = new Thread(drop);

    Thread intakeIn = new Thread(() -> intake.intakeMotor.setPower(-IntakeSubsystem.INTAKE_POWER));
//    Thread intakeInCommand = new Thread(intakeIn);

    Thread intakeStop = new Thread(() -> intake.intakeMotor.setPower(0));
//    Thread intakeStopCommand = new Thread(intakeStop);

    Thread colorSensorRun = new Thread(() -> {
        arm.detected = false;
        while (!arm.detected) {
            if (distanceSensor.distanceSensor.getDistance(DistanceUnit.MM) < DistanceSensorSubsystem.DISTANCE_THRESHOLD && !arm.sensorIsDisabled) {
                flipper.flipperServo.setPosition(FlipperSubsystem.FLIPPER_CLOSED);
                intake.intakeMotor.setPower(IntakeSubsystem.INTAKE_POWER);
                arm.detected = true;
            }
            if(opMode.isStopRequested()) return;
        }
    });

    Thread carouselBlueRun = new Thread(() -> carousel.carouselMotor.setPower(CarouselSubsystem.CAROUSEL_POWER));
    Thread carouselRedRun = new Thread(() -> carousel.carouselMotor.setPower(-CarouselSubsystem.CAROUSEL_POWER));
    Thread carouselStop = new Thread(() -> carousel.carouselMotor.setPower(0));

}
