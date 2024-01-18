// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class FollowTrajectory {

    public Command getAuto(){
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        return AutoBuilder.followPath(path);
        
    }

    }
