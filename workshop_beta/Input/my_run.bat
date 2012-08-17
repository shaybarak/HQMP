echo off
cls
start "Server" server configuration.txt
start mrmp
echo Please instruct GUI to connect to server...
pause
start "Player A" client configuration_a.txt
start "Player B" client configuration_b.txt
