#!/bin/sh
cat <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Simulator
GenericName=OctoMapping Simulator
Comment=OctoMapping Simulator
TryExec=$1/simulator
Exec=$1/simulator &
Categories=Application;Network;X-MandrivaLinux-Internet-Other;
Icon=/usr/share/icons/oxygen/64x64/status/weather-clouds.png
EOF