from commands2 import CommandBase, SequentialCommandGroup, InstantCommand, PrintCommand, Swerve4ControllerCommand
from wpilib import Timer
import wpimath
from wpimath import angleModulus
from wpimath.units import *
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.trajectory import TrapezoidProfileRadians, TrajectoryConfig, Trajectory, TrajectoryGenerator
from hardware_interface.subsystems.drive_subsystem import DriveSubsystem
import logging
import math
import typing

class Units:
    METERS = 0
    FEET = 1
    INCHES = 2

class DriveTimeAutoCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, seconds: float, velocity: tuple[float, float, float]):
        super().__init__()
        self.drive = drive
        self.seconds = seconds
        self.x = velocity[0]
        self.y = velocity[1]
        self.z = velocity[2]
        self.timer = Timer()
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.timer.reset()
        self.timer.start()        

    def execute(self):
        self.drive.swerve_drive(self.x, self.y, self.z, True)
        print(f"Time Elapsed: {self.timer.get()} s")
        print(f"Distance Traveled: {self.timer.get()*self.x} m")
        
    def end(self, interrupted):
        print("TIME END")
        self.drive.swerve_drive(0, 0, 0, False)
        self.drive.stop()
        
    def isFinished(self):
        return self.timer.get() >= self.seconds
        
class TurnToAngleCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, angle: float):
        super().__init__()
        self.drive = drive
        self.angle = angle
        self.pid_constraints = TrapezoidProfileRadians.Constraints(2*math.pi, 2*math.pi)
        self.turnPID = ProfiledPIDControllerRadians(1, 0, 0, self.pid_constraints)
        self.turnPID.setTolerance(math.radians(1))
        self.addRequirements(self.drive)

    def initialize(self):
        self.start_angle = self.drive.drivetrain.navx.getRotation2d().radians()
        self.turnPID.reset(self.drive.drivetrain.navx.getRotation2d().radians())
        self.turnPID.setGoal(math.radians(self.angle))

    def execute(self):
        self.angularVelMRadiansPerSecond = self.turnPID.calculate(self.drive.drivetrain.navx.getRotation2d().radians())
        self.drive.swerve_drive(0, 0, self.angularVelMRadiansPerSecond, False)

    def end(self, interrupted):
        self.drive.swerve_drive(0, 0, 0, False)
        self.drive.stop()

    def isFinished(self):
        return self.turnPID.atSetpoint()


        
class BalanceOnChargeStationCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, level_threshold: float):
        super().__init__()
        self.drive = drive
        self.power = 0.1
        self.level_threshold = level_threshold
        self.zero_count = 0
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.drive.unlockDrive()
        
    def execute(self):
        print("Balance Pitch: ", self.drive.getGyroRoll180())
        if self.drive.getGyroRoll180() < 1:
            self.drive.swerve_drive(-self.power, 0, 0, False)
        elif self.drive.getGyroRoll180() > 1:
            self.drive.swerve_drive(self.power, 0, 0, False)
        if self.drive.getGyroRoll180() == 0:
            self.power /= 1.1
        
    def end(self, interrupted):
        self.drive.swerve_drive(0, 0, 0, False)
        print("LOCKING")
        self.drive.lockDrive()
        
    def isFinished(self):
        curr_angle = self.drive.getGyroRoll180()
        return abs(curr_angle) < self.level_threshold
    
class SwerveTrajectoryCommand(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem, waypoints: list[Pose2d], trajectory=None):
        super().__init__()
        self.drive = drive
        self.waypoints = waypoints

        if len(self.waypoints) > 0:
            self.trajectory_config = TrajectoryConfig(
                1.0,
                0.5
            )
            self.trajectory_config.setKinematics(self.drive.getKinematics())
            
            self.trajectory: Trajectory = TrajectoryGenerator.generateTrajectory(
                self.waypoints,
                self.trajectory_config
            )
        else:
            self.trajectory = trajectory
        
        self.xController = PIDController(0.5, 0, 0)
        self.yController = PIDController(0.5, 0, 0)
        self.thetaController = ProfiledPIDControllerRadians(
            4, 0.0, 0.0, 
            constraints=TrapezoidProfileRadians.Constraints(1.0, 1.0)
        )
        self.thetaController.enableContinuousInput(-math.pi, math.pi)
        self.addRequirements(self.drive)
        print(f"Callable {type(self.drive.getPose)}")
        self.addCommands(
            InstantCommand(
                lambda: self.drive.resetOdometry(self.trajectory.sample(0).pose),
                self.drive
            ),
            Swerve4ControllerCommand(
                self.trajectory,
                self.drive.getPose,
                self.drive.getKinematics(),
                self.xController,
                self.yController,
                self.thetaController,
                self.drive.setModuleStates,
                [self.drive]
            ),
            InstantCommand(
                lambda: self.drive.swerve_drive(0, 0, 0, False),
                self.drive
            )
        )
        
        

        
        
class DriveToChargeStationCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, tilt_threshold: float):
        super().__init__()
        self.drive = drive
        self.tilt_threshold = tilt_threshold
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.drive.unlockDrive()
        
    def execute(self):
        print("To Charge Pitch: ", self.drive.getGyroRoll180())
        self.drive.swerve_drive(-3.5, 0, 0, False)
        
    def end(self, interrupted):
        self.drive.swerve_drive(0, 0, 0, False)
        self.drive.stop()
        
    def isFinished(self):
        return abs(self.drive.getGyroRoll180()) >= self.tilt_threshold
    
class TaxiAutoCommand(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem, side: str):
        super().__init__()
        self.drive = drive
        self.side = side
        self.addRequirements(self.drive)
        if self.side == "bump":
            self.addCommands(
                PrintCommand("Starting Taxi Auto"),
                DriveTimeAutoCommand(self.drive, 1.75, (-1.5, 0, 0))
            )
        else:
            self.addCommands(
                PrintCommand("Starting Taxi Auto"),
                DriveTimeAutoCommand(self.drive, 1.3, (-1.5, 0, 0))
            )
        
class UnlockDriveCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive = drive
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.drive.unlockDrive()
    
    def isFinished(self):
        return not self.drive.drivetrain.locked
    
class FieldOrientCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive = drive
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.drive.unlockDrive()
        self.drive.hardResetGyro()
        self.drive.recalibrateGyro()



class ConeMoveAuton():
    def __init__(self, drive: DriveSubsystem, object_pos: tuple[float, float, float]):
        self.drive = drive
        self.object_pos = object_pos    
        self.x = self.object_pos[0]
        self.y = self.object_pos[1]
        self.z = self.object_pos[2]
        self.max_power = 0.05
        self.angle_power = 0.5
        self.power = 0.2
        self.thresh = 0.1

    def execute(self) -> None:
        self.x = self.object_pos[0]
        self.y = self.object_pos[1]
        self.z = self.object_pos[2]
        
        distance = math.sqrt(self.x**2 + self.y**2)
        angle = math.atan2(self.y, self.x)
        
        if distance <= 0.5:
            distance = 0.0
        
        if distance > 2:
            distance = 0.0
        
        angular_velocity = (angle/(math.pi/2))*self.angle_power
        linear_velocity = distance*self.max_power
        
        x_vel = linear_velocity*math.cos(angle)
        y_vel = linear_velocity*math.sin(angle)
        
        if abs(x_vel) > self.max_power:
            x_vel = self.max_power*math.copysign(1, x_vel)

        self.drive.swerve_drive(-x_vel, -y_vel, angular_velocity, True)
        print("INPUT POSE", [self.x, self.y, self.z])
        print("CONE MOVE VELOCITY", [-x_vel, -y_vel, angular_velocity])
        
        
    
    