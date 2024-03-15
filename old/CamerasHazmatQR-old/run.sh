#!/bin/bash

# Prints my output in blue so it can be seen easier and different from the outputs from the commands
BLUE='\033[0;34m'
NC='\033[0m' # no color
printInColor() {
    echo -e "${BLUE} $1 ${NC}"
}


printInColor "STARTING!"
printInColor "(note: this script doesn't play super nice with control-c)\n"


python3 webcam_live_hazmat_qr_multiprocessing_with_server.py &
sleep 1

python3 server.py &
sleep 1


# start: wait for quit requested ---------------------------------------------#
printInColor "Finished startup Press K to exit"
while true; do 
read -rsn1 input
if [ "$input" = "k" ]; then
    printInColor "K key pressed"
    break
fi
done 
# end: wait for quit requested -----------------------------------------------#


# start: cleanup -------------------------------------------------------------#
printInColor "Starting closing background processes"
printInColor "Currently running backgrounds processes:"

echo -e "${BLUE}"
jobs -r
echo -e "${NC}"

sleep 1

printInColor "Killing background processes"

# Take output of running background jobs and pass them to kill
jobs -p | xargs kill

sleep 5 # give time for the procceses to end

echo -e "${BLUE}"
jobs
echo -e "${NC}"

printInColor "Finished cleanup"
# end: cleanup ---------------------------------------------------------------#


printInColor "Done"
