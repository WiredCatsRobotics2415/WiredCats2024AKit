# WiredCats2024 as an AdvantageKit project

[AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) is a logging system for WPILib projects. Mechanical Advantage (team 6328) maintains it along with thier log viewer [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope). They have the best robot code framework that I have seen out there, so I've replicated it using our competition code for the 2024 season in this repo.

Significant changes made to the 2024 code:
 - There is now a constants folder with better names for all classes/variables
 - OI keybind strings are now enums
 - Extensive simulation support for arm and simulated note shots
 - All subsystems are seperated into logic and IO layers per Mechanical Advantage's recommendation
 - util is reorganized and any logic that should have been its own class is moved here

Future plans:
 - rework all commands
 - add the robot CAD files so others can use them in Ascope
