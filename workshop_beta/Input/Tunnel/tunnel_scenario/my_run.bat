echo off
cls
start "Server" server configuration.txt
start mrmp
echo Please instruct GUI to connect to server...
pause
start run_client_a
start run_client_b
