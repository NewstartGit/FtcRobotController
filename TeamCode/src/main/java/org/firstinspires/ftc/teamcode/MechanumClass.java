package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MechanumClass {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor sliderRight;
    DcMotor sliderLeft;
    Servo pivotServo;
    Servo clawServo;

    public double getEncoderVal(String encoder) {
        //0 = front left motor
        //1 = front right motor
        //3 =
        switch (encoder) {
            case "x1":
                return -frontRight.getCurrentPosition();
            case "x2":
                return frontLeft.getCurrentPosition();
            case "y":
                return backLeft.getCurrentPosition();
            default:
                return 0;
        }
    }

    public void init(HardwareMap hwMap, boolean autoMode) {
        frontLeft = hwMap.get(DcMotor.class, "FL_Motor");
        frontRight = hwMap.get(DcMotor.class, "FR_Motor");
        backLeft = hwMap.get(DcMotor.class, "BL_Motor");
        backRight = hwMap.get(DcMotor.class, "BR_Motor");

        sliderRight = hwMap.get(DcMotor.class, "R_Slider");
        sliderLeft = hwMap.get(DcMotor.class, "L_Slider");

        pivotServo = hwMap.get(Servo.class, "PIVOT_Servo");
        clawServo = hwMap.get(Servo.class, "CLAW_Servo");

        //handServo = hwMap.get(Servo.class, "Hand_Servo");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(!autoMode)
        {
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        //frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void teleOP(double power, double pivot, double vertical, double horizontal, double slider, boolean intakeClose, boolean intakeOpen, boolean pivotUp, boolean pivotDown)
    {
        //, double arm, boolean open, boolean close, CameraClass aTag, boolean bumperPressed) {

        double pivotPosition = pivotServo.getPosition();
        boolean servoTrigger = !intakeOpen;
        double sliderPower = 0.75;
        int sliderLimit = 3000;
        /*
        if(intakeOpen)
        {
            servoTrigger = true;
        }
        if(intakeClose)
        {
            servoTrigger = false;
        }
        */
        //Placeholder because it gets grumpy
        sliderRight.setTargetPosition(300);
        sliderLeft.setTargetPosition(300);

        frontLeft.setPower(power * pivot + (power * (-vertical - horizontal)));
        frontRight.setPower(-power * pivot + (power * (-vertical + horizontal)));
        backLeft.setPower(power * pivot + (power * (-vertical + horizontal)));
        backRight.setPower(-power * pivot + (power * (-vertical - horizontal)));

        /*
        if(pivotPosition <= -.25 && pivotUp)
        {
            pivotServo.setPosition(pivotPosition - .01);
        }

        if(pivotPosition >= 0 && pivotDown)
        {
            pivotServo.setPosition(pivotPosition + .01);
        }
        */
        if(pivotUp)
        {
            pivotServo.setPosition(.75);
        }
        else if(pivotDown)
        {
            pivotServo.setPosition(.9);
        }
        else
        {
            pivotServo.setPosition(1);
        }


        if(sliderRight.getCurrentPosition() <= sliderLimit && sliderLeft.getCurrentPosition() <= sliderLimit)
        {
            sliderRight.setPower(-slider * sliderPower);
            sliderLeft.setPower(slider * sliderPower);
        }
        else
        {
            sliderRight.setPower(0.05);
            sliderLeft.setPower(0.05);
        }

        if(servoTrigger)
        {
            //servo close
            clawServo.setPosition(0);
        }
        else
        {
            //servo open
            clawServo.setPosition(1);
        }
    }

    public double returnTelemetry(String hardware)
    {
        if(hardware.equalsIgnoreCase("PivotServo"))
        {
            return pivotServo.getPosition();
        }
        else
        {
            return 3;
        }
    }

    public void drive(double angle, double power, long delay, int position, boolean run) throws InterruptedException {
        if (run) {
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            // converts the degrees that is inputted to radians, adjusted to equal unit circle
            double radAngle = Math.toRadians(-angle + 90);
            // calculate motor power
            double ADPower = power * Math.sqrt(2) * 0.5 * (Math.sin(radAngle) + Math.cos(radAngle));
            double BCPower = power * Math.sqrt(2) * 0.5 * (Math.sin(radAngle) - Math.cos(radAngle));

            // tells the motors to run using the encoder

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // stops and resets the encoders so that the position isnt repeated
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // multiplies the position parameter by the value that was outputted by the equation
            double ADPositionPower = position * ADPower;
            double BCPositionPower = position * BCPower;
            // AD & BC move the same power
            // sets the target position to the multiplied position power

            frontLeft.setTargetPosition((int) ADPositionPower);
            frontRight.setTargetPosition((int) BCPositionPower);
            backLeft.setTargetPosition((int) BCPositionPower);
            backRight.setTargetPosition((int) ADPositionPower);
            // tells the motors to run based on the encoders
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // set the motors
            // powers the motors using the original power values
            frontLeft.setPower(ADPower);
            frontRight.setPower(BCPower);
            backLeft.setPower(BCPower);
            backRight.setPower(ADPower);
            // delay
            Thread.sleep(delay);
        } else {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    //This is going to rotate x degrees, NOT TO x DEGREES
    public void rotate(double degrees, double power, long delay, IMUClass imu) throws InterruptedException {
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        imu.resetDegree();
        double imuDegrees = imu.runIMU();
        double threshold = 1;

        while (imuDegrees < degrees + threshold) {
            if (-degrees > 0) {
                frontLeft.setPower(power);
                frontRight.setPower(-power);
                backLeft.setPower(power);
                backRight.setPower(-power);
                imuDegrees = imu.runIMU();
            } else {
                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(-power);
                backRight.setPower(power);
                imuDegrees = imu.runIMU();
            }
        }
        while (imuDegrees > degrees - threshold) {
            if (-degrees > 0) {
                frontLeft.setPower(-power * .3);
                frontRight.setPower(power * .3);
                backLeft.setPower(-power * .3);
                backRight.setPower(power * .3);
                imuDegrees = imu.runIMU();
            } else {
                frontLeft.setPower(power * .3);
                frontRight.setPower(-power * .3);
                backLeft.setPower(power * .3);
                backRight.setPower(-power * .3);
                imuDegrees = imu.runIMU();
            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
    }

    public boolean alignWithAprilTag(double power, int distance, CameraClass aTag, int tagID) throws InterruptedException {
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double tagDistance;// = aTag.returnAprilTagValues("Distance");
        double angle;// = aTag.returnAprilTagValues("Angle");
        double xDistance;// = aTag.returnAprilTagValues("Heading");
        double tolerance = 2;
        double powerMultiplier = .75;
        double rotateMultiplier = 20;
        boolean facingTowards = false;

        if(aTag.returnAprilTagValues("Detected",tagID) == -1) // && aTag.returnAprilTagValues("1") == 1;
        {
            tagDistance = aTag.returnAprilTagValues("Distance",tagID);
            angle = aTag.returnAprilTagValues("Angle",tagID);
            xDistance = aTag.returnAprilTagValues("Heading",tagID);

            if(xDistance > 0 + tolerance) //if over tolerance
            {
                frontLeft.setPower(-power * powerMultiplier);
                frontRight.setPower(power * powerMultiplier);
                backLeft.setPower(power * powerMultiplier);
                backRight.setPower(-power * powerMultiplier);
            }
            else if(xDistance < 0 - tolerance) // if under tolerance
            {
                frontLeft.setPower(power * powerMultiplier);
                frontRight.setPower(-power * powerMultiplier);
                backLeft.setPower(-power * powerMultiplier);
                backRight.setPower(power * powerMultiplier);
            }
            else
            {
                if(tagDistance > distance) {
                    frontLeft.setPower(power);
                    frontRight.setPower(power);
                    backLeft.setPower(power);
                    backRight.setPower(power);
                }
                else
                {
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    return false;
                }
            }

            return true;

        }
        else
        {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            return true;
        }

    }

    public int returnPixelRegion(CameraClass tensorflow)
    {
        //Loops through, need to give it time before going to else statement
        if(tensorflow.runTfod() > 0)
        {
            return tensorflow.runTfod();
        }
        else
        {
            return 0;
        }
    }
}
