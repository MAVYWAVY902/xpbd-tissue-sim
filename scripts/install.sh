#!/bin/bash

# make sure the script has root
# if (( $EUID != 0 )); then
#     echo "This script requires root access."
#     exit 1
# fi

# Default to prompting
auto_yes=false

# Check all arguments for -y
for arg in "$@"; do
    if [[ "$arg" == "-y" ]]; then
        auto_yes=true
        break
    fi
done

# Function to ask for confirmation
ask_confirmation() {
    local prompt="$1"
    
    if [[ "$auto_yes" == true ]]; then
        echo "$prompt (y/n): y"
        return 0  # true
    fi
    
    while true; do
        read -p "$prompt (y/n): " answer
        case "${answer,,}" in
            y|yes) return 0 ;;
            n|no) return 1 ;;
            *) echo "Please answer y or n" ;;
        esac
    done
}

# Usage
THIRDPARTY_DIR=$(realpath $1)
echo "Third-party drivers and Github repositories will be installed in ${THIRDPARTY_DIR}"
if ask_confirmation "Do you want to continue?"; then
    echo "Continuing..."
else
    echo "Exiting..."
    exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# install dependencies
bash $SCRIPT_DIR/install_dependencies.sh $THIRDPARTY_DIR

# set up environment
source $SCRIPT_DIR/set_env.sh $THIRDPARTY_DIR