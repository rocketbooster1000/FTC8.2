package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class LiftHighCommand extends CommandBase {
    LiftSubsystem m_subsystem;

    public LiftHighCommand(LiftSubsystem subsystem){
        this.m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        m_subsystem.setTarget(Constants.Lift.HIGH_POSITION);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
