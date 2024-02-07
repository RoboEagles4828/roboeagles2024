from robot.constants import Constants
from robot.subsystems.Swerve import Swerve

from wpimath.controller import PIDController;
from wpimath.controller import ProfiledPIDController;
from wpimath.geometry import Pose2d;
from wpimath.geometry import Rotation2d;
from wpimath.geometry import Translation2d;
from wpimath.trajectory import Trajectory;
from wpimath.trajectory import TrajectoryConfig;
from wpimath.trajectory import TrajectoryGenerator;
from commands2 import InstantCommand
from commands2 import SequentialCommandGroup
from commands2 import SwerveControllerCommand

import math as Math

class exampleAuto(SequentialCommandGroup):
    def __init__(self, s_Swerve: Swerve):
        config = \
            TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
            )
        
        config.setKinematics(Constants.Swerve.swerveKinematics)

        # An example trajectory to follow.  All units in meters.
        exampleTrajectory = \
            TrajectoryGenerator.generateTrajectory(
                # Start at the origin facing the +X direction
                Pose2d(0, 0, Rotation2d(0)),
                # Pass through these two interior waypoints, making an 's' curve path
                [Translation2d(1, 1), Translation2d(2, -1)],
                # End 3 meters straight ahead of where we started, facing forward
                Pose2d(3, 0, Rotation2d(0)),
                config
            )

        thetaController = \
            ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.pi, Math.pi);

        swerveControllerCommand = \
            SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve.getPose,
                Constants.Swerve.swerveKinematics,
                PIDController(Constants.AutoConstants.kPXController, 0, 0),
                PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve.setModuleStates,
                s_Swerve
            )


        self.addCommands(
            InstantCommand(lambda: s_Swerve.setPose(exampleTrajectory.initialPose())),
            swerveControllerCommand
        )