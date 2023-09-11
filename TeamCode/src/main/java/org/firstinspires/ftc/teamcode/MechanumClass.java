package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MechanumClass {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

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

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void teleOP(double power, double pivot, double vertical, double horizontal) {

        frontLeft.setPower(-power * pivot + (power * (-vertical - horizontal)));
        frontRight.setPower(-power * pivot + (power * (-vertical + horizontal)));
        backLeft.setPower(power * pivot + (power * (-vertical + horizontal)));
        backRight.setPower(power * pivot + (power * (-vertical - horizontal)));
    }

    public void drive(double angle, double power, long delay, int position, boolean run) throws InterruptedException {
        if (run) {
            // converts the degrees that is inputted to radians, adjusted to equal unit circle
            double radAngle = Math.toRadians(angle);
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

        while (imuDegrees < degrees) {
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
        while (imuDegrees > degrees) {
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

    public boolean alignWithAprilTag(double power, int distance, AprilTagClass aTag) {
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        double tagDistance = aTag.returnAprilTagValues("Distance");
        double angle = aTag.returnAprilTagValues("Angle");
        double xDistance = aTag.returnAprilTagValues("X Value");

        if(tagDistance > distance) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }
        return false;
    }


    public boolean strafeAroundAprilTag(double speed, int distance, AprilTagClass aTag)
    {
        double tagDistance = aTag.returnAprilTagValues("Distance");
        double angle = aTag.returnAprilTagValues("Angle");
        double xDistance = aTag.returnAprilTagValues("X Value");

        //Get angle and strafing down first, then distance
        if(xDistance < 0) // Strafe right
        {
            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);
            return true;
        }
        else if(xDistance > 0) // Strafe left
        {
            frontLeft.setPower(-speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(-speed);
            return true;
        }
        else
        {
            return false;
        }

        /*
        if(tagDistance > distance)
        {
            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);
            if(xDistance > 0) // Strafe right
            {
                frontLeft.setPower(speed);
                frontRight.setPower(-speed);
                backLeft.setPower(-speed);
                backRight.setPower(speed);
            }
            else if(xDistance < 0) // Strafe left
            {
                frontLeft.setPower(-speed);
                frontRight.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(-speed);
            }
            else
            {
                return false;
            }
            if(angle > 0)
            {
                frontLeft.setPower(0);
                frontRight.setPower(-speed*2);
                backLeft.setPower(0);
                backRight.setPower(speed*2);
                return true;
            }
            else if(angle < 0)
            {
                frontLeft.setPower(speed*2);
                frontRight.setPower(0);
                backLeft.setPower(-speed*2);
                backRight.setPower(0);
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return false;
        }

         */
    }

    public boolean driveForwardUntil(double speed,int position,AprilTagClass aTag) throws InterruptedException
    {
        double distance = aTag.returnAprilTagValues("Distance");
        double angle = aTag.returnAprilTagValues("Angle");
        if(distance > position)
        {
            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);

        }
        else if(distance <= position)
        {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }
        if(angle > 0)
        {
            frontLeft.setPower(0);
            frontRight.setPower(-speed*2);
            backLeft.setPower(0);
            backRight.setPower(speed*2);
            return true;
        }
        else if(angle < 0)
        {
            frontLeft.setPower(speed*2);
            frontRight.setPower(0);
            backLeft.setPower(-speed*2);
            backRight.setPower(0);
            return true;
        }
        else
        {
            return false;
        }


        //frontRight.setPower(-speed);

        //Thread.sleep(10000);
    }


}
