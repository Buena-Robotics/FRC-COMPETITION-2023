package frc.robot.nerds.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TrajectoryCommandCalculator {
    static SequentialCommandGroup trajectoryCommand(){

        List<Command> commandList = new ArrayList<Command>();

        return new SequentialCommandGroup((Command[]) commandList.toArray());
    }
}
