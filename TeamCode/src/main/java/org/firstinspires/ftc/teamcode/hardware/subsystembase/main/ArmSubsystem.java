package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class ArmSubsystem extends BaseSubsystem {
    // Values
    public static double ARM_INTAKE_POS = -0.03; // 0.04 //-0.09
//    public static double ARM_NEUTRAL_POS = -0.003;
    public static double ARM_LOW_POS = -.442; //0.132 //0.035
    public static double ARM_MID_POS = -.41; //0.25 //0.17
    public static double ARM_HIGH_POS = -.315; //0.36 //0.27
    public static double ARM_INTERMEDIATE_POS = -0.22; //0.36 //0.27

    public static double ARM_POWER = 0.35;
//    public static double ARM_SLOW_POWER = 0.3;
    public static double ARM_TICKS_PER_REV = 1425.06;
    public static double ARM_POS_CHANGE_SPEED = 0.0005;

//    public static double ARM_INTAKE_WAIT = 0.45;
    public static double ARM_UP_WAIT = 0.4;
    public static double ARM_FORWARD_WAIT = 0.7;
    public static double ARM_DOWN_WAIT = 0;
    public static double ARM_RESET_WAIT = 1.0;
    public static double ARM_RESET_WAIT_2 = 1.5;

    public static boolean useCustompidf = false;

    public static PIDFCoefficients armPIDF = new PIDFCoefficients(10, 0, 0.5, 0, MotorControlAlgorithm.LegacyPID);
    boolean setIntakePos = false;
    double targetTime1 = 0;
    double targetTime2 = 0;
    double targetTime3 = 0;
    boolean armUp = false;
    boolean armForward = false;
    boolean armReset = false;
    boolean armReset2 = false;
    boolean armDown = false;
    boolean boxUp = false;
    boolean boxDown = false;
    boolean armIsUp = false;
    boolean buttonPressed = false;

    BoxSubsystem box;
    TurretSubsystem turret;
    FlipperSubsystem flipper;

    double armTargetPos = 0;
    double armSelectedPos = 0;
    double servoSelectedPos = 0;


    // Create hardware variables
    public DcMotorEx armMotor = null;

    // Constructor
    public ArmSubsystem() {

    }

    // Initialize hardware variables
    public void init(HardwareMap hardwareMap, Telemetry telemetry, BoxSubsystem box, TurretSubsystem turret, FlipperSubsystem flipper) {
        super.init(hardwareMap, telemetry);
        this.box = box;
        this.turret = turret;
        this.flipper = flipper;

        armMotor = hardwareMap.get(DcMotorEx.class,"armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Arm Encoders
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (useCustompidf) {
            armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, armPIDF);
        }
    }

    // Arm Commands
    public void setArmPosition() {
        armMotor.setTargetPosition((int) (ARM_TICKS_PER_REV * armTargetPos));
    }

    public void findArmPosition(Gamepad gamepad1, Gamepad gamepad2) {
        super.gamepadInit(gamepad1, gamepad2);
        // High
//        if (gamepad2.dpad_up) {
//            armMotor.setPower(ARM_POWER);
//            armTargetPos = ARM_HIGH_POS;
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        // Mid
//        if (gamepad2.dpad_right) {
//            armMotor.setPower(ARM_POWER);
//            armTargetPos = ARM_MID_POS;
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        // Shared hub
//        if (gamepad2.dpad_down) {
//            armMotor.setPower(ARM_POWER);
//            armTargetPos = ARM_LOW_POS;
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        }
//
//        // intake/reset position
//        if (gamepad2.dpad_left) {
//
//            targetTime1 = runtime.seconds() + ARM_INTAKE_WAIT;
//            setIntakePos = true;
//        }
//        if (setIntakePos && runtime.seconds() >= targetTime1) {
//            armTargetPos = ARM_INTAKE_POS;
//            setIntakePos = false;
//        }
        if (gamepad2.dpad_up && !buttonPressed) {
            armUp(ARM_HIGH_POS, BoxSubsystem.BOX_HIGH);
            buttonPressed = true;
        } else if (gamepad2.dpad_right && !buttonPressed) {
            armUp(ARM_MID_POS, BoxSubsystem.BOX_HIGH);
            buttonPressed = true;
        } else if (gamepad2.dpad_down && !buttonPressed) {
            armUp(ARM_LOW_POS, BoxSubsystem.BOX_SHARED);
            buttonPressed = true;


        } else if (gamepad2.dpad_left && !buttonPressed) {
            armReset();
            buttonPressed = true;


        } else if (gamepad2.right_bumper && !buttonPressed) {
            armUp(TurretSubsystem.TURRET_SERVOS_RIGHT);
        } else if (gamepad2.left_bumper && !buttonPressed) {
            armUp(TurretSubsystem.TURRET_SERVOS_LEFT);
        }


        if (!gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_down && !gamepad2.dpad_right && !gamepad2.right_bumper) {
             buttonPressed = false;
        }






        if (-gamepad2.right_stick_y > ControllerSubsystem.TRIGGER_THRESHOLD) {
            armTargetPos += ARM_POS_CHANGE_SPEED;
            setArmPosition();
        }
        if (-gamepad2.right_stick_y < -ControllerSubsystem.TRIGGER_THRESHOLD) {
            armTargetPos -= ARM_POS_CHANGE_SPEED;
            setArmPosition();
        }



//        if (-gamepad2.right_stick_y > ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_left) {
//            ARM_INTAKE_POS += ARM_POS_CHANGE_SPEED;
//            setArmPosition();
//        }
//        if (-gamepad2.right_stick_y < -ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_left) {
//            ARM_INTAKE_POS -= ARM_POS_CHANGE_SPEED;
//            setArmPosition();
//        }



        if (-gamepad2.right_stick_y > ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_up) {
            ARM_HIGH_POS+= ARM_POS_CHANGE_SPEED;
            setArmPosition();
        }
        if (-gamepad2.right_stick_y < -ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_up) {
            ARM_HIGH_POS -= ARM_POS_CHANGE_SPEED;
            setArmPosition();
        }



        if (-gamepad2.right_stick_y > ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_right) {
            ARM_MID_POS+= ARM_POS_CHANGE_SPEED;
            setArmPosition();
        }
        if (-gamepad2.right_stick_y < -ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_right) {
            ARM_MID_POS -= ARM_POS_CHANGE_SPEED;
            setArmPosition();
        }

        if (-gamepad2.right_stick_y > ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_down) {
            ARM_LOW_POS+= ARM_POS_CHANGE_SPEED;
            setArmPosition();
        }
        if (-gamepad2.right_stick_y < -ControllerSubsystem.TRIGGER_THRESHOLD && gamepad2.dpad_down) {
            ARM_LOW_POS -= ARM_POS_CHANGE_SPEED;
            setArmPosition();
        }
    }

    public void loopCommand() {

        if (boxUp && runtime.seconds() >= targetTime1) {
            boxUp = false;

            box.boxServo.setPosition(servoSelectedPos);

            armForward = true;
        }
        if (armForward && runtime.seconds() >= targetTime2) {
            armForward = false;

            armTargetPos = armSelectedPos;
            armIsUp = true;
            turret.disableTurret = false;
            setArmPosition();
            turret.setTurretPosition();
        }






        if (boxDown && runtime.seconds() >= targetTime1) {
            boxDown = false;

            box.boxServo.setPosition(BoxSubsystem.BOX_DOWN_SLIGHTLY_FORWARD);

//            flipper.disableFlipper = true;
//            flipper.flipperServo.setPosition(FlipperSubsystem.FLIPPER_UP);

            armReset = true;
        }
        if (armReset && runtime.seconds() >= targetTime2) {
            armReset = false;

            armTargetPos = ARM_INTAKE_POS;
            armIsUp = false;
            setArmPosition();

            armReset2 = true;
        }
        if (armReset2 && runtime.seconds() >= targetTime3) {
            armReset2 = false;

           box.boxServo.setPosition(BoxSubsystem.BOX_DOWN);
           armMotor.setPower(0);
//           armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//           armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void armUp(double finalArmPosition, double finalServoPosition) {
        armSelectedPos = finalArmPosition;
        servoSelectedPos = finalServoPosition;
        armMotor.setPower(ARM_POWER);

        if (!armIsUp) {
            turret.targetPos = TurretSubsystem.TURRET_SERVOS_FRONT;
            turret.setTurretPosition();
            turret.disableTurret = true;

            armTargetPos = ARM_INTERMEDIATE_POS;
            setArmPosition();

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            flipper.disableFlipper = false;

            // action 1
            boxUp = true;

            // action 2
            targetTime1 = runtime.seconds() + ARM_UP_WAIT;

            // action 3
            targetTime2 = runtime.seconds() + ARM_FORWARD_WAIT;
        } else {
            armTargetPos = armSelectedPos;
            setArmPosition();
            box.boxServo.setPosition(servoSelectedPos);
        }
    }
    public void armUp(double turretPos) {
        armSelectedPos = ARM_INTERMEDIATE_POS;
        servoSelectedPos = BoxSubsystem.BOX_HIGH;
        turret.targetPos = turretPos;

        armMotor.setPower(ARM_POWER);

        if (!armIsUp) {
            turret.disableTurret = true;

            armTargetPos = ARM_INTERMEDIATE_POS;
            setArmPosition();

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            flipper.disableFlipper = false;

            // action 1
            boxUp = true;

            // action 2
            targetTime1 = runtime.seconds() + ARM_UP_WAIT;

            // action 3
            targetTime2 = runtime.seconds() + ARM_UP_WAIT;
        } else {
            armTargetPos = armSelectedPos;
            setArmPosition();
            box.boxServo.setPosition(servoSelectedPos);
        }
    }

    public void armReset() {
        if (armIsUp) {

            armMotor.setPower(ARM_POWER);

            turret.targetPos = TurretSubsystem.TURRET_SERVOS_FRONT;
            turret.setTurretPosition();
            turret.disableTurret = true;

            armTargetPos = ARM_INTERMEDIATE_POS;
            setArmPosition();

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            // action 1
            boxDown = true;

            // action 2
            targetTime1 = runtime.seconds() + ARM_DOWN_WAIT;

            // action 3
            targetTime2 = runtime.seconds() + ARM_RESET_WAIT;

            // action 4
            targetTime3 = runtime.seconds() + ARM_RESET_WAIT_2;
        }
    }


    public double getArmPosition() {
        return armMotor.getCurrentPosition()/ ARM_TICKS_PER_REV;

    }

    public void breakMode() {
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public void floatMode() {
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
}