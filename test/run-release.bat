@echo off & cd /d %~dp0
..\out\build\x64-release\exo-skeleton-pose-ebimu.exe COM5 || (pause & exit /b)