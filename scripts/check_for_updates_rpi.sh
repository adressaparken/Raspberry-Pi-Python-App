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

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && cd .. && pwd )"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
IP=$(ip route get 8.8.8.8 | awk '/8.8.8.8/ {print $NF}')
IP_ARR=(${IP//./ })
ID=${IP_ARR[3]}


############################################################
##  Stop native app  #######################################

function stop_native_app {
    mosquitto_pub -t "parken/rpi/$ID/quit" -m 0
}


############################################################
##  Main loop  #############################################

while true; do

    ########################################################
    ##  Fetching git  ######################################

    echo ""
    echo "############################################################"
    echo "## Updating tracking branches.                            ##"
    echo ""

    git -C $ROOT_DIR fetch
    echo "Done."
    echo ""


    ########################################################
    ##  Get repo heads  ####################################

    echo ""
    echo "############################################################"
    echo "## Fetching local and remote repo heads.                  ##"
    echo ""

    LOCAL=$(git -C $ROOT_DIR rev-parse HEAD)
    REMOTE=$(git -C $ROOT_DIR rev-parse origin/master)
    echo "Local head:  " $LOCAL
    echo "Remote head: " $REMOTE
    echo ""


    ########################################################
    ##  Check if update is required  #######################

    echo ""
    echo "############################################################"
    echo "## Checking if update is required.                        ##"
    echo ""

    if [ $LOCAL = $REMOTE ]; then
        UPDATE=false
        echo "Repo is up-to-date."
    else
        UPDATE=true
        echo "Repo needs to be pulled."
    fi
    echo ""


    ########################################################
    ##  Stop native app if update is required  #############

    if [ $UPDATE = true ]; then
        echo ""
        echo "############################################################"
        echo "## Stopping native app.                                   ##"
        echo ""

        stop_native_app
        echo "Done."
        echo ""
    fi


    ########################################################
    ##  Native app ended - looping #########################

    echo ""
    echo "############################################################"
    echo "## Re-checking for updates in $LOOP_TIME seconds.                 ##"

    read -t $LOOP_TIME -p "## Press <CTRL>+c to stop.                                ##"
    echo ""
    echo ""
done
