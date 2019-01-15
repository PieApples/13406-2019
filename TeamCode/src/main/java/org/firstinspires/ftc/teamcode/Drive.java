package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive", group="13406")
public class Drive extends OpMode {

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor rightScoop = null;
    private DcMotor leftScoop = null;
    private DcMotor mainScoop = null;
    private CRServo yeetus = null;
    private DcMotor lifter = null;
    private Servo rightDoor = null;
    private Servo leftDoor = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightScoop = hardwareMap.get(DcMotor.class, "rightScoop");
        leftScoop = hardwareMap.get(DcMotor.class, "leftScoop");
        mainScoop = hardwareMap.get(DcMotor.class, "mainScoop");
        yeetus = hardwareMap.get(CRServo.class, "yeetus");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        rightDoor = hardwareMap.get(Servo.class, "rightDoor");
        leftDoor = hardwareMap.get(Servo.class, "leftDoor");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftScoop.setDirection(DcMotor.Direction.FORWARD);
        rightScoop.setDirection(DcMotor.Direction.REVERSE);
        mainScoop.setDirection(DcMotor.Direction.FORWARD);

    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {

       rightDoor.resetDeviceConfigurationForOpMode();
       leftDoor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void loop() {

        if (gamepad2.x == true) {

            leftDoor.setPosition(Servo.MAX_POSITION);
        }

        if (gamepad2.b == true){

            leftDoor.setPosition(Servo.MIN_POSITION);
        }

        if(gamepad2.dpad_left == true){

            rightDoor.setPosition(Servo.MAX_POSITION);
        }
        if (gamepad2.dpad_right == true){

            rightDoor.setPosition(Servo.MIN_POSITION);
        }

        if (gamepad1.dpad_down == true){

            lifter.setPower(.5);
        }
        if (gamepad1.dpad_up == true){

            lifter.setPower(-.5);
        }
        if (gamepad1.a == true){

            lifter.setPower(0);
        }


        double leftPower;
        double rightPower;


        double drive = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);


        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
        leftBack.setPower(leftPower);
        rightBack.setPower(rightPower);
        rightScoop.setPower(gamepad2.right_stick_y*2/3);
        leftScoop.setPower(gamepad2.left_stick_y*2/3);
        mainScoop.setPower(gamepad2.left_trigger*3/4);
        mainScoop.setPower(-gamepad2.right_trigger*3/4);
        yeetus.setPower(gamepad1.right_trigger*2/3);
        yeetus.setPower(-gamepad1.left_trigger*2/3);
    }

    @Override
    public void stop() {
    }

}
