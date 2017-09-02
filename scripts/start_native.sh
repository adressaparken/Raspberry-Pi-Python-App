#!/usr/bin/env bash


############################################################
##  Arguments  #############################################

if [ $# -eq 0 ]; then
    LOOP_TIME=10
else
    LOOP_TIME=$1
fi


############################################################
##  Globals  ###############################################

SHELL=bash
NATIVE_APP='main'
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && cd .. && pwd )"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


############################################################
##  Update native app  #####################################

function update_native_app {
    $SHELL $SCRIPT_DIR/pull_git_native.sh
}


############################################################
##  Start native app  ######################################

function start_native_app {
    python3 $ROOT_DIR/$NATIVE_APP.py
}


############################################################
##  Main loop  #############################################

while true; do

    ########################################################
    ##  Update native app  #################################

    update_native_app


    ########################################################
    ##  Start native app  ##################################

    echo ""
    echo "############################################################"
    echo "## Starting native app.                                   ##"
    echo ""

    start_native_app
    echo ""


    ########################################################
    ##  Native app ended - looping #########################

    echo ""
    echo "############################################################"
    echo "## Native app ended.                                      ##"
    echo "## Restarting native app in $LOOP_TIME seconds.                   ##"

    read -t $LOOP_TIME -p "## Press <CTRL>+c to stop.                                ##"
    echo ""
    echo ""

done
