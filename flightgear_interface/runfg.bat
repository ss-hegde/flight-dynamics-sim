C:

cd C:\Program Files\FlightGear 2024.1

SET FG_ROOT=C:\Program Files\FlightGear 2024.1\data

START .\\bin\fgfs.exe --fdm=null --native-fdm=socket,in,30,localhost,5502,udp --native-fdm=socket,out,30,localhost,5501,udp --enable-terrasync --prop:/sim/rendering/shaders/quality-level=0 --aircraft=c172p --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 --disable-sound --in-air --airport=KSFO --runway=10L --altitude=842.4 --heading=113 --offset-distance=4.72 --offset-azimuth=0  
