package org.firstinspires.ftc.teamcode.opmode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;


import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.command.LiftHighCommand;
import org.firstinspires.ftc.teamcode.command.LiftOverrideMoveCommand;
import org.firstinspires.ftc.teamcode.command.LiftRedZoneCommand;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.RotateSubsystem;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    DriveSubsystem driveSubsystem;
    LiftSubsystem lift;
    RotateSubsystem passthrough;
    GamepadEx gamepad;
    List<LynxModule> allHubs;


    @Override
    public void initialize(){
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule model : allHubs){
            model.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        gamepad = new GamepadEx(gamepad1);
        driveSubsystem = new DriveSubsystem(hardwareMap, gamepad);
        lift = new LiftSubsystem(hardwareMap);
        passthrough = new RotateSubsystem(hardwareMap);

        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenHeld(new LiftOverrideMoveCommand(lift))
                .whenReleased(new InstantCommand(lift::reset, lift));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new LiftHighCommand(lift));

        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SequentialCommandGroup(
                        new LiftRedZoneCommand(lift),
                        new InstantCommand(passthrough::rotate, passthrough)));


        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem));
    }

    @Override
    public void run(){
        super.run();
        for (LynxModule module: allHubs){
            module.clearBulkCache();
        }
    }
}
