set MAVPROXY_PROCESS_NAME=mavproxy.exe

REM Beende laufende MAVProxy-Instanz
taskkill /IM %MAVPROXY_PROCESS_NAME% /F

REM Warte kurz, um sicherzustellen, dass der Prozess beendet wird
timeout /t 2 /nobreak >nul

mavproxy.exe ^
  --master=COM4 ^
  --baudrate 57600 ^
  --out=udp:127.0.0.1:14555 ^
  --out=udp:127.0.0.1:14666 ^
  --out=udp:127.0.0.1:14777 ^
  --default-modules log,signing,wp,param,relay,tuneopt,arm,mode,calibration,rc,auxopt,misc,cmdlong,battery,terrain,output,graph,message ^
  --cmd "graph SERVO_OUTPUT_RAW.servo3_raw " ^
  --streamrate 25
pause
