#!/bin/bash
appimage=arduino-ide_2.3.6_Linux_64bit.AppImage
scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
targetdir=/usr/local/bin/Arduino

if [ -f "$scriptdir/$appimage" ]; then
    echo "Datei $scriptdir/$appimage existiert bereits."
else
    echo "Datei $scriptdir/$appimage existiert nicht, wird heruntergeladen..."
    wget -O "$scriptdir/$appimage" "https://downloads.arduino.cc/arduino-ide/$appimage"
    chmod +x "$scriptdir/$appimage"
    echo "copy $appimage to $targetdir"
    sudo rm -rf $targetdir/*.AppImage
    sudo cp -v $appimage $targetdir
    sudo cp -v logo.png $targetdir
    echo "add permissions"
    sudo chmod 777 $targetdir/$appimage
    sudo chmod 666 $targetdir/logo.png


    rules=99-arduino.rules
    rulesdir=/etc/udev/rules.d
    echo "copy $rules"
    sudo cp -v $rules $rulesdir
    echo "add permissions"
    sudo chmod 644 $rulesdir/$rules

    desktop=arduino.desktop
    desktopdir=/usr/share/applications
    echo "copy $desktop"
    sudo cp -v $desktop $desktopdir
    echo "add permissions"
    sudo chmod 755 $desktopdir/$desktop

    boards=boards.txt
    boardsdir=/home/robi/.arduino15/packages/arduino/hardware/avr/1.8.6
    echo "copy $boards"
    sudo cp -v $boards $boardsdir
    echo "add permissions"
    sudo chmod 644 $boardsdir/$boards
fi