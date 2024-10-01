set AUTOTESTDIR="%~dp0\aircraft"

FOR /F "delims=" %%D in ('dir /b "\Program Files"\FlightGear*') DO set FGDIR=%%D
echo "Using FlightGear 2020.3"
cd "\Program Files\FlightGear 2020.3\bin"
fgfs ^
    --native-fdm=socket,in,10,,5503,udp ^
    --fdm=external ^
    --aircraft=Rascal110-JSBSim ^
    --fg-aircraft=%AUTOTESTDIR% ^
    --airport=SIGQ ^
    --geometry=650x550 ^
    --bpp=32 ^
    --disable-hud-3d ^
    --disable-horizon-effect ^
    --timeofday=noon ^
    --disable-sound ^
    --disable-fullscreen ^
    --disable-random-objects ^
    --disable-ai-models ^
    --fog-disable ^
    --disable-specular-highlight ^
    --disable-anti-alias-hud ^
    --wind=0@0
pause
