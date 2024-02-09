from wpilib.interfaces import GenericHID
from wpilib import Joystick
from wpilib import XboxController
from commands2 import Command, Subsystem
from commands2 import InstantCommand, ConditionalCommand
from commands2.button import JoystickButton
from CTREConfigs import CTREConfigs


from autos.exampleAuto import exampleAuto
from commands.TeleopSwerve import TeleopSwerve
from subsystems.Swerve import Swerve

from commands.SysId import DriveSysId

from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets, BuiltInLayouts
from wpilib import SendableChooser

from pathplannerlib.auto import PathPlannerAuto

class RobotContainer:
    ctreConfigs = CTREConfigs()
    # Drive Controls
    translationAxis = XboxController.Axis.kLeftY
    strafeAxis = XboxController.Axis.kLeftX
    rotationAxis = XboxController.Axis.kRightX

    driver = XboxController(0)

    # Driver Buttons
    zeroGyro = JoystickButton(driver, XboxController.Button.kBack)
    robotCentric = JoystickButton(driver, XboxController.Button.kStart)

    sysId = JoystickButton(driver, XboxController.Button.kY)

    robotCentric_value = True

    # Subsystems
    s_Swerve : Swerve = Swerve()

    #SysId
    driveSysId = DriveSysId(s_Swerve)


    # The container for the robot. Contains subsystems, OI devices, and commands.
    def __init__(self):

        self.s_Swerve.setDefaultCommand(
            TeleopSwerve(
                self.s_Swerve, 
                lambda: -self.driver.getRawAxis(self.translationAxis), 
                lambda: -self.driver.getRawAxis(self.strafeAxis), 
                lambda: -self.driver.getRawAxis(self.rotationAxis), 
                lambda: self.robotCentric_value
            )
        )

        self.auton = Shuffleboard.getTab("Auton")
        self.teleop = Shuffleboard.getTab("Teleop")

        self.auton_selector = SendableChooser()
        self.auton_selector.setDefaultOption("Test Auto", PathPlannerAuto("TestAuto"))

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

        # Configure the button bindings
        self.configureButtonBindings()

    """
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    """
    def configureButtonBindings(self):
        # Driver Buttons
        self.zeroGyro.onTrue(InstantCommand(lambda: self.s_Swerve.zeroHeading()))
        self.robotCentric.onTrue(InstantCommand(lambda: self.toggleFieldOriented()))

        #TODO: Uncomment for sysid
        # self.sysId.whileTrue(self.driveSysId.getCommand())


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