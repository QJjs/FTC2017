package org.firstinspires.ftc.teamcode;
/**
 * Created by Peder on 7/9/2017.
 */

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Mecanum Drive", group="Final")

public class MecanumDrive extends OpMode
{
    //Controls:
    //  Left Stick: forward/backward/left/right movement
    //  Right Stick (x): turn left/right
    //  Start: switch direction front <-> back
    //  Left Trigger (hold down): use localized movement
    //  Left Button: toggle localized movement
    //  A: toggle glyph claw
    //  D-pad: lift/lower glyph claw
    //  Back: emergency color sensor arm retract

    DefaultHWMap robot = new DefaultHWMap();
    public DcMotor LeftF, LeftB, RightF, RightB, lift;
    public BNO055IMU imu;
    public Servo sensorArm;
    public NormalizedColorSensor sensor;
    public CRServo claw;
    public double goal = 0;
    public int autonMode = -1;

    boolean[] toggles = { false, false, false, false, false };
    boolean localMove = false, switchDir = true;

    @Override
    public void init()
    {
        robot.init(hardwareMap);

        LeftF = robot.LeftF;
        LeftB = robot.LeftB;
        RightF = robot.RightF;
        RightB = robot.RightB;

        imu = robot.imu;

        sensorArm = robot.sensorArm;
        sensor = robot.sensor;

        lift = robot.lift;
        claw = robot.claw;
    }

    public void setMode()
    {
        if(gamepad1.start)
        {
            stage = 1;
            return;
        }
        if(gamepad1.back)
        {
            stage = -1;
            return;
        }

        if(!toggles[0] && gamepad1.a)
        {
            toggles[0] = true;
            autonMode++;
        }
        if(toggles[0] && !gamepad1.a) toggles[0] = false;

        switch(autonMode % 4)
        {
            case 4:
                autonMode -= 4;
            case 0:
                telemetry.addLine("Auton Position Set: Blue 1");
                break;
            case 1:
                telemetry.addLine("Auton Position Set: Blue 2");
                break;
            case 2:
                telemetry.addLine("Auton Position Set: Red 2");
                break;
            case 3:
                telemetry.addLine("Auton Position Set: Red 1");
                break;
            default:
                telemetry.addLine("Error with autonMode");
                return;
        }

        telemetry.addLine("Press START to begin");
        telemetry.addLine("Press BACK to skip autonomous");
    }

    int stage = 0;
    boolean detectColor; //false: red; true: blue
    @Override
    public void init_loop()
    {
        telemetry.addData("stage:", stage);
        float[] hsv = new float[3];
        Color.colorToHSV(sensor.getNormalizedColors().toColor(), hsv);
        telemetry.addData("h:", hsv[0]);
        telemetry.addData("s:", hsv[1]);
        telemetry.addData("v:", hsv[2]);

        if(stage == -1) return;
        else if(stage == 0) setMode();

        else if(stage == 1)
        {
            sensorArm.setPosition(1);
            stage = 2;
        }

        else if(stage == 2)
        {
            if(sensorArm.getPosition() < 1) return;

            if(hsv[1] < 0.5 || hsv[2] > 0.1)
                return;
            if(hsv[0] >= 340 && hsv[0] < 360 || hsv[0] >= 0 && hsv[0] <= 20)
            {
                detectColor = false;
                stage = 3;
            }
            else if(hsv[0] >= 170 && hsv[0] < 220)
            {
                detectColor = true;
                stage = 3;
            }
        }

        else if(stage == 3)
        {
            double movePow = 0.3;
            switch(autonMode)
            {
                case 0:
                case 1:
                    //if(detectColor) moveStraight(movePow);
                    //else moveStraight(-movePow);
                    turn(detectColor ? -0.3 : 0.5);
                    waitMs(100);
                    turn(detectColor ? 0.3 : -0.5);
                    break;
                case 2:
                case 3:
                    turn(detectColor ? 0.5 : -0.3);
                    waitMs(100);
                    turn(detectColor ? -0.5 : 0.3);
                    break;
                default:
                    telemetry.addLine("error");
                    return;
            }
            moveStraight(0);
            stage = 4;
        }

        else if(stage == 4)
        {
            switch(autonMode)
            {
                case 0:
                    rotateTo(90);
                    break;
                case 1:

                    break;
                case 2:

                    break;
                case 3:

                    break;
                default:
                    telemetry.addLine("error");
                    return;
            }
        }
    }

    public void waitMs(long ms)
    {
        try { Thread.sleep(ms); } catch(Exception e) { }
    }

    public void rotateTo(double angle)
    {
        double dir = wrap(imu.getAngularOrientation().firstAngle + angle);
        LeftF.setPower(dir); RightF.setPower(-dir);
        LeftB.setPower(dir); RightB.setPower(-dir);
        while(Math.abs(wrap(imu.getAngularOrientation().firstAngle + angle)) > 5)
        {
            waitMs(10);
        }
        LeftF.setPower(0); RightF.setPower(0);
        LeftB.setPower(0); RightB.setPower(0);
    }

    public void turn(double pow)
    {
        LeftF.setPower(-pow); RightF.setPower(pow);
        LeftB.setPower(-pow); RightB.setPower(pow);
    }

    public void moveStraight(double pow)
    {
        LeftF.setPower(pow); RightF.setPower(pow);
        LeftB.setPower(pow); RightB.setPower(pow);
    }

    public void setTargetPos(int target)
    {
        LeftF.setTargetPosition(-target); RightF.setTargetPosition(target);
        LeftB.setTargetPosition(target); RightB.setTargetPosition(-target);
    }

    @Override
    public void start()
    {
        for(boolean t : toggles)
            t = false;
        //toggles[0] = false;
    }

    public int liftEncoder = 0;

    @Override
    public void loop()
    {
        double x = gamepad1.left_stick_x,
                y = gamepad1.left_stick_y;

        if(gamepad1.back) sensorArm.setPosition(0);

        double currRot = imu.getAngularOrientation().firstAngle;

        if(!toggles[0] && gamepad1.left_trigger >= 0.1)
        {
            toggles[0] = true;
            if(!localMove) goal = currRot;
            localMove = !localMove;
        }
        if(toggles[0] && gamepad1.left_trigger < 0.1)
        {
            toggles[0] = false;
            localMove = !localMove;
        }

        if(!toggles[1] && gamepad1.left_bumper)
        {
            toggles[1] = true;
            localMove = !localMove;
        }
        if(toggles[1] && !gamepad1.left_bumper) toggles[1] = false;

        if(!toggles[2] && gamepad1.start)
        {
            switchDir = !switchDir;
            toggles[2] = true;
        }
        if(toggles[2] && !gamepad1.start) toggles[2] = false;

        double angle = -Math.toRadians(wrap(goal + currRot));

        double turn = -gamepad1.right_stick_x;

        if(switchDir)
        {
            x = -x;
            y = -y;
        }
        if(x*x+y*y > 0.01 || Math.abs(turn) > 0.1)
        {
            if(localMove)
            {
                double nx = x * Math.cos(angle) - y * Math.sin(angle);
                double ny = x * Math.sin(angle) + y * Math.cos(angle);
                x = nx; y = ny;
            }
            LeftF.setPower(clamp(y - x + turn, 1)); RightF.setPower(clamp(y + x - turn, 1));
            LeftB.setPower(clamp(y + x + turn, 1)); RightB.setPower(clamp(y - x - turn, 1));
        }
        else
        {
            LeftF.setPower(0); RightF.setPower(0);
            LeftB.setPower(0); RightB.setPower(0);
        }

        if(gamepad1.a) claw.setPower(1);
        else if(gamepad1.b) claw.setPower(-1);
        else claw.setPower(0);

        if(gamepad1.dpad_up) lift.setPower(0.5);
        else if(gamepad1.dpad_down) lift.setPower(-0.5);
        if(gamepad1.dpad_up || gamepad1.dpad_down) toggles[3] = true;
        if(toggles[3] && !gamepad1.dpad_up && !gamepad1.dpad_down)
        {
            toggles[3] = false;
            liftEncoder = lift.getCurrentPosition();
        }


        sensorArm.setPosition(gamepad1.back ? 1 : 0);

        telemetry.addData("mode:", x);
        telemetry.addData("rotation:", currRot);
        telemetry.addLine("Encoders:");
        telemetry.addData("  LF:", LeftF.getCurrentPosition());
        telemetry.addData("  LB:", LeftB.getCurrentPosition());
        telemetry.addData("  RF:", RightF.getCurrentPosition());
        telemetry.addData("  RB:", RightB.getCurrentPosition());
        telemetry.addData("color:", sensor.getNormalizedColors().toString());
    }

    public double clamp(double num, double min, double max)
    {
        return Math.min(Math.max(num, min), max);
    }

    public double clamp(double num, double absLimit)
    {
        return clamp(num, -absLimit, absLimit);
    }

    public double wrap(double num)
    {
        return wrap(num, -180, 180, 360);
    }

    public double wrap(double num, double min, double max, double correction)
    {
        while(num <= min) num += correction;
        while(num > max) num -= correction;
        return num;
    }

    public double getAngularDistance(double theta1, double theta2)
    {
        theta1 = Math.toRadians(theta1);
        theta2 = Math.toRadians(theta2);
        return Math.toDegrees(Math.acos(Math.cos(theta1) * Math.cos(theta2) + Math.sin(theta1) * Math.sin(theta2)));
    }

    @Override
    public void stop()
    {

    }
}