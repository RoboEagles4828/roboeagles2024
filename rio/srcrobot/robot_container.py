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

    robotCentric_value = True

    # Subsystems
    s_Swerve : Swerve = Swerve()


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

    def toggleFieldOriented(self):
        self.robotCentric_value = not self.robotCentric_value


    """
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
    """
    def getAutonomousCommand(self) -> Command:
        # An ExampleCommand will run in autonomous
        return exampleAuto(self.s_Swerve)