@echo off
set /p file="File Name: "
scp admin@roboRIO-447-frc.local/home/lvuser/deploy/%file% .\CSV