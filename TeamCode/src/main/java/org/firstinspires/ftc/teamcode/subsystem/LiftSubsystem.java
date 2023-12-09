package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.common.PIDController;

@Config
public class LiftSubsystem extends SubsystemBase {
    private DcMotorEx liftMotor;
    private PIDController controller;
    private double target;

    public static double kP, kI, kD, kF;

    public LiftSubsystem(HardwareMap hardwareMap){
        liftMotor = new DcMotorWrapper("Lift_Motor", hardwareMap, DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controller = new PIDController(kP, kI, kD, 0);
    }

    public void update(){
        controller.setPID(kP, kI, kD);
        double power = controller.calculate(liftMotor.getCurrentPosition(), target);
        liftMotor.setPower(power + kF);
    }

    public void setTarget(double target){
        if (target > Constants.Lift.LINEAR_SLIDE_MINIMUM && target < Constants.Lift.LINEAR_SLIDE_MAXIMUM){
            this.target = target;
        }
    }

    public double getTarget(){
        return target;
    }

    public double getPosition(){
        return liftMotor.getCurrentPosition();
    }

    public void moveSlide(double dTarget){
        if (this.target + dTarget <= Constants.Lift.LINEAR_SLIDE_MAXIMUM && this.target + dTarget >= Constants.Lift.LINEAR_SLIDE_MINIMUM){
            this.target += dTarget;
        }
    }

    public void override(double dTarget){
        this.target += dTarget;
    }

    public void reset(){
        double pos = liftMotor.getCurrentPosition();
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDController(kP, kI, kD, 0, pos);
    }

}
