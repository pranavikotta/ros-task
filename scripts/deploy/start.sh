# #!/bin/bash

# # Get base.sh funcs
# source "$(dirname "$0")/base.sh"

# # Stop any existing Docker containers
# stop_docker

# # declare mode, use gpu by default
# # mode="gpu"

# # declare sim, use sim by default
# sim="True"

# while getopts 'ch' opt; do
#     case "$opt" in
#         c)
#             mode="cpu"
#             ;;
#         ?|h)
#             echo "Usage: $(basename $0) [-c]"
#             exit 1
#             ;;
#     esac
# done
# shift "$(($OPTIND -1))"

# if [ "$mode" == "gpu" ]; then
#     run_docker --runtime=nvidia \
#     -v $(dirname "$0")/../../workspace/:/root/workspace/src \
#     limo_bot:sim "/root/app.sh"
# else
#     run_docker \
#     -v $(dirname "$0")/../../workspace/:/root/workspace/src \
#     limo_bot:sim "/root/app.sh"
# fi

#!/bin/bash

# Get base.sh funcs
source "$(dirname "$0")/base.sh"

stop_docker

# declare mode, use gpu by default
mode="cpu"

# declare sim, use sim by default
sim="True"

# Absolute paths for volumes
WORKSPACE_PATH=$(realpath "$(dirname "$0")/../../workspace")
APP_SCRIPT_PATH=$(realpath "$(dirname "$0")/app.sh")

while getopts 'ch' opt; do
    case "$opt" in
        c)
            mode="cpu"
            ;;
        ?|h)
            echo "Usage: $(basename $0) [-c]"
            exit 1
            ;;
    esac
done
shift "$(($OPTIND -1))"

if [ "$mode" == "gpu" ]; then
    run_docker --runtime=nvidia \
    -v $WORKSPACE_PATH:/root/workspace/src \
    limo_bot:sim bash $APP_SCRIPT_PATH
else
    run_docker \
    -v $WORKSPACE_PATH:/root/workspace/src \
    limo_bot:sim bash $APP_SCRIPT_PATH
fi