package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    DriveSubsystem driveSubsystem;
    GamepadEx gamepad;


    @Override
    public void initialize(){
        gamepad = new GamepadEx(gamepad1);
        driveSubsystem = new DriveSubsystem(hardwareMap, gamepad);

        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem));
    }
}
