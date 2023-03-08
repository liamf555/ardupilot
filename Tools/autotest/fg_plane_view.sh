#!/bin/sh

AUTOTESTDIR=$(dirname $0)

d=$(echo "/home/tu18537/flightgear/install")
export LD_LIBRARY_PATH=$d/simgear/lib:$d/openscenegraph/lib:$d/openrti/lib:$d/plib/lib${LD_LIBRARY_PATH:+:}${LD_LIBRARY_PATH}
# cd "$d/flightgear/bin"

# nice fgfs
exec /home/tu18537/flightgear/install/flightgear/bin/fgfs --fg-root="$d/flightgear/fgdata" "$@" \
    --native-fdm=socket,in,10,,5503,udp \
    --fdm=external \
    --aircraft=Rascal110-JSBSim \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    --airport=YKRY \
    --geometry=650x550 \
    --bpp=32 \
    --disable-hud-3d \
    --disable-horizon-effect \
    --timeofday=noon \
    --disable-sound \
    --disable-fullscreen \
    --disable-random-objects \
    --disable-ai-models \
    --fog-disable \
    --disable-specular-highlight \
    --disable-anti-alias-hud \
    --wind=0@0 \
    $*
