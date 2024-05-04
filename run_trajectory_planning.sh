#!/bin/bash

#This script will set up the correct python evironment and will run the trajectory planning 

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"
echo "The pythonpath has been set to: $PYTHONPATH"

if [[ ":$PYTHONPATH:" != *"/Deterministic_Annealing:"* ]]; then
    echo "Error: No entry in PYTHONPATH ending with /Deterministic_Annealing. Aborting script."
    exit 1
else 
    echo "Success: PYTHONPATH has been correctly set"
fi

python3 ./src/main.py
exit 0 