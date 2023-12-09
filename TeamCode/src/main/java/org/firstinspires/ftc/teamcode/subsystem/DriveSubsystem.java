package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.teamcode.common.Algorithms.returnMecanumValues;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.DcMotorWrapper;

import java.util.function.DoubleSupplier;


public class DriveSubsystem extends SubsystemBase {
    private DcMotorEx fL, fR, bL, bR;
    private IMU imu;
    private DoubleSupplier leftX, leftY, rightX;

    public DriveSubsystem(HardwareMap hwMap, GamepadEx gamepad){
        fL = new DcMotorWrapper("Front_left", hwMap, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR = new DcMotorWrapper("Front_right", hwMap, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL = new DcMotorWrapper("Back_left", hwMap, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR = new DcMotorWrapper("Back_right", hwMap, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);




        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        leftX = gamepad::getLeftX;
        leftY = gamepad::getLeftY;
        rightX = gamepad::getRightX;
    }

    public void setPowers(double[] powers){
        fL.setPower(powers[0]);
        fR.setPower(powers[1]);
        bL.setPower(powers[2]);
        bR.setPower(powers[3]);
    }

    public void drive(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double[] powers = returnMecanumValues(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble(), orientation.getYaw(AngleUnit.DEGREES), Constants.Drive.DRIVE_POWER_MODIFIER);
        setPowers(powers);
    }
    /*
    public class MotorWrapper{
        public DcMotorEx motor;
        double lastPower;

        public MotorWrapper(DcMotorEx motor){
            this.motor = motor;
            lastPower = 0;
        }

        public void setPower(double power){
            if (power != lastPower){
                motor.setPower(power);
                lastPower = power;
            }
        }
    }
    */
}
