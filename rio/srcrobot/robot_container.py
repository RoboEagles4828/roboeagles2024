from wpilib.interfaces import GenericHID
from wpilib import Joystick
from wpilib import XboxController
from commands2 import Command, Subsystem
from commands2 import InstantCommand, ConditionalCommand
from commands2.button import JoystickButton
from CTREConfigs import CTREConfigs
from commands2 import CommandScheduler

from wpimath.geometry import Rotation2d
from constants import Constants

from autos.exampleAuto import exampleAuto
from commands.TeleopSwerve import TeleopSwerve
from subsystems.Swerve import Swerve
from subsystems.Arm import Arm
from commands.TurnInPlace import TurnInPlace

from commands.SysId import DriveSysId

from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets, BuiltInLayouts
from wpilib import SendableChooser

from autos.PathPlannerAutoRunner import PathPlannerAutoRunner
from pathplannerlib.auto import NamedCommands

class RobotContainer:
    ctreConfigs = CTREConfigs()
    # Drive Controls
    translationAxis = XboxController.Axis.kLeftY
    strafeAxis = XboxController.Axis.kLeftX
    rotationAxis = XboxController.Axis.kRightX

    driver = XboxController(0)
    operator = XboxController(1)

    sysId = JoystickButton(driver, XboxController.Button.kY)

    robotCentric_value = True

    # Subsystems
    s_Swerve : Swerve = Swerve()
    s_Arm : Arm = Arm()

    #SysId
    driveSysId = DriveSysId(s_Swerve)


    # The container for the robot. Contains subsystems, OI devices, and commands.
    def __init__(self):
        translation = lambda: 0.0
        strafe = lambda: 0.0
        rotation = lambda: 0.0
        robotcentric = lambda: False

        NamedCommands.registerCommand("FaceForward", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(180)), translation, strafe, rotation, robotcentric))
        NamedCommands.registerCommand("FaceBackward", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(0)), translation, strafe, rotation, robotcentric))
        NamedCommands.registerCommand("FaceLeft", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(90)), translation, strafe, rotation, robotcentric))
        NamedCommands.registerCommand("FaceRight", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(-90)), translation, strafe, rotation, robotcentric))

        self.auton = Shuffleboard.getTab("Auton")
        self.teleop = Shuffleboard.getTab("Teleop")
        self.zeroGyro = JoystickButton(self.driver, XboxController.Button.kBack)
        self.robotCentric = JoystickButton(self.driver, XboxController.Button.kStart)
        self.faceForward = JoystickButton(self.driver, XboxController.Button.kY)
        self.faceBack = JoystickButton(self.driver, XboxController.Button.kA)
        self.faceRight = JoystickButton(self.driver, XboxController.Button.kB)
        self.faceLeft = JoystickButton(self.driver, XboxController.Button.kX)

        self.resetToAbsoluteButton = JoystickButton(self.driver, XboxController.Button.kRightBumper)

        self.manualArm = JoystickButton(self.driver, XboxController.Button.kLeftBumper)
        self.armAmp = JoystickButton(self.driver, XboxController.Button.kY)
        self.armPodium = JoystickButton(self.driver, XboxController.Button.kB)
        self.armSubwoofer = JoystickButton(self.driver, XboxController.Button.kA)
        self.intakeOn = self.driver.POVRight(CommandScheduler.getInstance().getDefaultButtonLoop())
        self.intakeOff = self.driver.POVLeft(CommandScheduler.getInstance().getDefaultButtonLoop())

        self.armHome = JoystickButton(self.operator, XboxController.Axis.kRightTrigger)
        self.shooterOff = JoystickButton(self.operator, XboxController.Button.kRightBumper)
        self.reverse = JoystickButton(self.operator, XboxController.Axis.kLeftTrigger)
        self.queSubFront = JoystickButton(self.operator, XboxController.Button.kA)
        self.quePodium = JoystickButton(self.operator, XboxController.Button.kY)
        self.queSubRight = JoystickButton(self.operator, XboxController.Button.kB)
        self.queSubLeft = JoystickButton(self.operator, XboxController.Button.kX)
        self.queAmp = self.operator.POVUp(CommandScheduler.getInstance().getDefaultButtonLoop())
        self.queClimbFront = self.operator.POVDown(CommandScheduler.getInstance().getDefaultButtonLoop())
        self.queClimbRight = self.operator.POVRight(CommandScheduler.getInstance().getDefaultButtonLoop())
        self.queClimbLeft = self.operator.POVLeft(CommandScheduler.getInstance().getDefaultButtonLoop())

        self.configureButtonBindings()

        self.auton_selector = SendableChooser()
        self.auton_selector.setDefaultOption("Test Auto", PathPlannerAutoRunner("TestAuto", self.s_Swerve).getCommand())
        self.auton_selector.addOption("Example Auto", exampleAuto(self.s_Swerve).getCommand())

        self.auton.add("Auton Selector", self.auton_selector)\
            .withWidget(BuiltInWidgets.kComboBoxChooser)\
            .withSize(2, 1)\
            .withPosition(0, 0)
        
        self.teleop.addBoolean("Field Centric", lambda: not self.robotCentric_value)\
            .withPosition(9, 0)\
            .withSize(1, 1)\
            .withWidget(BuiltInWidgets.kBooleanBox)
        self.teleop.addBoolean("Zero Gyro", lambda: self.zeroGyro.getAsBoolean())\
            .withPosition(10, 0)\
            .withSize(1, 1)\
            .withWidget(BuiltInWidgets.kBooleanBox)
        self.teleop.addBoolean("SysId", lambda: self.sysId.getAsBoolean())\
            .withPosition(11, 0)\
            .withSize(1, 1)\
            .withWidget(BuiltInWidgets.kBooleanBox)
        self.teleop.add("Gyro", self.s_Swerve.gyro)\
            .withPosition(0, 0)\
            .withSize(2, 2)\
            .withWidget(BuiltInWidgets.kGyro)
        self.teleop.add("Swerve Subsystem", self.s_Swerve)\
            .withPosition(0, 2)\
            .withSize(3, 3)
        
        Shuffleboard.update()

    """
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    """
    def configureButtonBindings(self):
        translation = lambda: -self.driver.getRawAxis(self.translationAxis) 
        strafe = lambda: -self.driver.getRawAxis(self.strafeAxis)
        rotation = lambda: self.driver.getRawAxis(self.rotationAxis)
        robotcentric = lambda: self.robotCentric_value

        self.s_Swerve.setDefaultCommand(
            TeleopSwerve(
                self.s_Swerve, 
                translation,
                strafe,
                rotation,
                robotcentric
            )
        )

        # Arm Buttons
        self.s_Arm.setDefaultCommand(self.s_Arm.seekArmZero())
        self.manualArm.whileTrue(self.s_Arm.moveArm(lambda: self.operator.getLeftY()))
        self.armAmp.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kAmpPivotAngle))
        self.armPodium.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kPodiumPivotAngle))
        self.armSubwoofer.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kSubwooferPivotAngle))
        # Driver Buttons
        self.zeroGyro.onTrue(InstantCommand(lambda: self.s_Swerve.zeroYaw()))
        self.robotCentric.onFalse(InstantCommand(lambda: self.toggleFieldOriented()))

        self.faceForward.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(180)), translation, strafe, rotation, robotcentric))
        self.faceBack.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(0)), translation, strafe, rotation, robotcentric))
        self.faceLeft.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(90)), translation, strafe, rotation, robotcentric))
        self.faceRight.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(-90)), translation, strafe, rotation, robotcentric))



    def toggleFieldOriented(self):
        self.robotCentric_value = not self.robotCentric_value


    """
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
    """
    def getAutonomousCommand(self) -> Command:
        auto = self.auton_selector.getSelected()
        return auto