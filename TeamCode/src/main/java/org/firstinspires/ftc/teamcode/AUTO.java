package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="RAuto", group="OOF")

public class AUTO extends LinearOpMode {

    // Declare OpMode members.


    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor rightScoop = null;
    private DcMotor leftScoop = null;
    private DcMotor mainScoop = null;
    private DcMotor lifter = null;
    ColorSensor sensorColor = null;
    DistanceSensor sensorDistance = null;
    private Servo colorArm = null;
    private Servo yeetus = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = .75;
    static final double     TURN_SPEED              = .75;


    private GoldAlignDetector detector;

    public void setAllMotors(double power){

        leftBack.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);

    }

    public void DriveForwardDistance(){

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setTargetPosition(-2120);
        leftFront.setTargetPosition(-2120);
        rightBack.setTargetPosition(-2120);
        rightFront.setTargetPosition(-2120);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBack.setPower(.5);
        leftFront.setPower(.5);
        rightFront.setPower(.5);
        rightBack.setPower(.5);

        while(rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy())
        {
            //wait until target position is reached
        }

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    private boolean motorsBusy(){
        return leftFront.isBusy() && leftFront.isBusy() && rightFront.isBusy() && rightBack.isBusy();
    }

    public void mainUp(){

        mainScoop.setPower(1);
        sleep(250);
    }

    public void Down(){

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lifter.setTargetPosition(11200);

        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lifter.setPower(1);

        while(lifter.isBusy())
        {
            //wait until target position is reached
        }

        lifter.setPower(0);


        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void encoderDrive(//double speed,
                             //double leftInches, double rightInches,double timeoutS
                             int ticksLeft, int ticksRight, double speed ) {
/*
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newLeftFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newRightBackTarget, newLeftBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition());
                leftBack.getCurrentPosition();
                rightBack.getCurrentPosition();

                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }*/

        int lfPose = leftFront.getCurrentPosition() + ticksLeft;
        int lrPose = leftBack.getCurrentPosition() + ticksLeft;
        int rfPos = rightFront.getCurrentPosition() + ticksRight;
        int rrPos = rightBack.getCurrentPosition() + ticksRight;

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition(lfPose);
        leftBack.setTargetPosition(lrPose);
        rightFront.setTargetPosition(rfPos);
        rightBack.setTargetPosition(rrPos);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.downscale = 0.3;
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 75;
        detector.alignPosOffset = 75;

        detector.enable();

        telemetry.addData("Status", "Initialized");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightScoop = hardwareMap.get(DcMotor.class, "rightScoop");
        leftScoop = hardwareMap.get(DcMotor.class, "leftScoop");
        mainScoop = hardwareMap.get(DcMotor.class, "mainScoop");
        sensorColor = hardwareMap.colorSensor.get("colorSensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorSensor");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        colorArm = hardwareMap.get(Servo.class, "colorArm");
        yeetus = hardwareMap.get(Servo.class, "yeetus");

        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        colorArm.setDirection(Servo.Direction.FORWARD);



        waitForStart();

        Down();


        yeetus.setPosition(Servo.MIN_POSITION);
        encoderDrive(535,  535,  .5);
        while( motorsBusy() && !isStopRequested()){

        }
        setAllMotors(0);
        mainUp();
        encoderDrive(-535, 535, .5);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);
        mainUp();
        /*encoderDrive(-178, -178, .5);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Moving Out");
            telemetry.update();
        }
        setAllMotors(0);*/
        mainUp();
        encoderDrive(-2426, -2426, 1);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);
        mainUp();
     /*   encoderDrive(-1248,-1248,.5);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        */
        mainUp();
        encoderDrive(500,-500,.5);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);
        mainUp();
      /*  encoderDrive(2000,  2000, .5);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }*/
        setAllMotors(0);
        mainUp();

        int tickRef = leftFront.getCurrentPosition();
        encoderDrive(3400, 3400, 0.3);

        while(detector.getAligned() == false && motorsBusy() && !isStopRequested()){
            telemetry.addData("Aligned", detector.getXPosition());
            telemetry.update();
        }
        int ticksLeft = 3400 -( leftFront.getCurrentPosition() - tickRef);
        setAllMotors(0);

        encoderDrive(1400, -1400, 1 );
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);

        encoderDrive(1200 , 1200, 1);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Scoring");
            telemetry.update();
        }
        setAllMotors(0);

        encoderDrive(-1200 , -1200, 1);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Scoring");
            telemetry.update();
        }
        setAllMotors(0);

        encoderDrive(-1300, 1300, 0.5);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Turning");
            telemetry.update();
        }
        setAllMotors(0);
       /* yeetus.setPosition(Servo.MIN_POSITION);
        encoderDrive(ticksLeft, ticksLeft, 0.5);
        while( motorsBusy() && !isStopRequested()){
            telemetry.addData("Status","Scoring");
            telemetry.update();
        }
        setAllMotors(0);
        mainUp();
        encoderDrive(900,-900,1);
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Turning");
        }
        setAllMotors(0);
        mainUp();
        encoderDrive(1200,1200,1);//hello human
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Moving Forwards");
        }
        setAllMotors(0);
        mainUp();
        encoderDrive(1000,-1000,1);
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Turning");//do you want to play a game
        }
        setAllMotors(0);
        mainUp();/*
        encoderDrive(700,700,1);
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Moving Forwards");
        }
        setAllMotors(0);
        mainUp();
        encoderDrive(700,-700,1);
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Turning");
        }
        setAllMotors(0);*/
        mainUp();
        encoderDrive(3300,3300,1);
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Going to YEET");
        }
        setAllMotors(0);
        yeetus.setPosition(Servo.MAX_POSITION);
        sleep(400);
        mainUp();
        encoderDrive(-1000,-1000,1);
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Finishing");
        }
        mainUp();
        encoderDrive(200,-200,1);
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Finishing");
        }
        setAllMotors(0);
        mainUp();
        encoderDrive(-5900,-5900,1);
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Turning");
        }
        setAllMotors(0);
       /* encoderDrive(-3000,-3000,1);
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Finishing");
        }
        /*encoderDrive(300,-300,1);
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Finishing");
        }*/
        setAllMotors(0);
        mainUp();
        encoderDrive(-4000,-4000,1);
        while(motorsBusy() && !isStopRequested()){
            telemetry.addData("Status:", " Finishing");
        }
        setAllMotors(0);


        detector.disable();

    }

}
