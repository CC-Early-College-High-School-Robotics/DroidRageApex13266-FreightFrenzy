package org.firstinspires.ftc.teamcode.auto.paths.comp2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BoxSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.FlipperSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.TurretSubsystem;

public class AutoCommandThread extends Thread{
    ArmSubsystem arm                            = new ArmSubsystem                          ();
    BoxSubsystem box                            = new BoxSubsystem                          ();
    CarouselSubsystem carousel                  = new CarouselSubsystem                     ();
    IntakeSubsystem intake                      = new IntakeSubsystem                       ();
//    TelemetrySubsystem telemetrySubsystem       = new TelemetrySubsystem                    ();
    TurretSubsystem turret                      = new TurretSubsystem                       ();
    FlipperSubsystem flipper                    = new FlipperSubsystem                      ();
    DistanceSensorSubsystem distanceSensor      = new DistanceSensorSubsystem               ();

    LinearOpMode opmode;

    AutoCommandThread(LinearOpMode opmode) {
        this.opmode = opmode;
    }
    @Override
    public void run() {
        AutoBooleans.isActive = true;
        arm.init(opmode.hardwareMap, opmode.telemetry, box, turret, flipper, distanceSensor, intake);
        distanceSensor.init(opmode.hardwareMap, opmode.telemetry);
        box.init(opmode.hardwareMap, opmode.telemetry);
        carousel.init(opmode.hardwareMap, opmode.telemetry);
        intake.init(opmode.hardwareMap, opmode.telemetry);
        turret.init(opmode.hardwareMap, opmode.telemetry);
        flipper.init(opmode.hardwareMap,opmode.telemetry);
        flipper.flipperServo.setPosition(FlipperSubsystem.FLIPPER_CLOSED);
        while(AutoBooleans.isActive) {
            if (AutoBooleans.armBlueWarehouseStart) {
                arm.armUp(ArmSubsystem.ARM_HIGH_POS, BoxSubsystem.BOX_HIGH, AutoValues.AUTO_TURRET_BLUE_WAREHOUSE_STARt, false);
            }
            if (AutoBooleans.armRedWarehouseStart) {
                arm.armUp(ArmSubsystem.ARM_HIGH_POS, BoxSubsystem.BOX_HIGH, AutoValues.AUTO_TURRET_RED_WAREHOUSE_START, false);
            }
            if (AutoBooleans.armIntake) {
                arm.sensorIsDisabled = false;
                arm.armReset();
            }
            if (AutoBooleans.openFlipper) {
                flipper.flipperServo.setPosition(FlipperSubsystem.FLIPPER_OPEN);
            }
            if (AutoBooleans.armToAllianceHubBlue) {
                arm.armUp(ArmSubsystem.ARM_HIGH_POS, BoxSubsystem.BOX_HIGH, AutoValues.AUTO_TURRET_POSITION_BLUE, false);
            }
            if (AutoBooleans.armToAllianceHubRed) {
                arm.armUp(ArmSubsystem.ARM_HIGH_POS, BoxSubsystem.BOX_HIGH, AutoValues.AUTO_TURRET_POSITION_RED, false);
            }
            if (AutoBooleans.intake) {
//                try {
//                    sleep(AutoValues.AUTO_INTAKE_WAIT);
//                } catch (InterruptedException ignored) {
//                }
                intake.intakeMotor.setPower(-IntakeSubsystem.INTAKE_POWER);
            }
            arm.loopCommand();
            AutoBooleans.init = false;
            AutoBooleans.armBlueWarehouseStart = false;
            AutoBooleans.armIntake = false;
            AutoBooleans.openFlipper = false;
            AutoBooleans.intake = false;
            AutoBooleans.armToAllianceHubBlue = false;
            // dont ever do this bc screws up everything
//            opmode.telemetry.update();
        }
    }
}
