@echo off
REM Check if user passed an argument
IF "%~1"=="" (
    echo Usage: build_test.bat test\testAll.cpp
    exit /b 1
)

REM Get argument (test file)
set TESTFILE=%~1

REM Configure CMake with the test file
cmake -B build -DBUILD_TEST=ON -DTEST_FILE=%TESTFILE%

REM Get just the filename (without path and extension) for target
for %%f in (%TESTFILE%) do set TESTNAME=%%~nf

REM Build that test target
cmake --build build --target %TESTNAME%

pause