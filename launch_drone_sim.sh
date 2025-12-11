#!/usr/bin/env bash

################ CONFIG ################
CATKIN_WS="$HOME/courseRoot/catkin_ws"
ARDUCOPTER_DIR="$HOME/courseRoot/apm/ardupilot/ArduCopter"

# ROS setup
ROS_SETUP="/opt/ros/noetic/setup.bash"

# Paths for help menu
WORLD_DIR="$HOME/courseRoot/ardupilot_gazebo/worlds"
MODEL_DIR="$HOME/courseRoot/ardupilot_gazebo/models"
DRONE_SCRIPT_DIR="$CATKIN_WS/src/gazebo_drone/scripts"
REPO_LINK="https://github.com/McMaster-Drone-Club/VM_Scripts_NEW"
########################################

# Colors
RED="\e[31m"
GREEN="\e[32m"
YELLOW="\e[33m"
BLUE="\e[34m"
BOLD="\e[1m"
RESET="\e[0m"

# Track shell PIDs inside GNOME Terminal tabs
declare -a TERMINAL_PIDS=()

########################################
# Check GNOME Terminal
########################################
if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo -e "${RED}Error:${RESET} gnome-terminal not found. Use GNOME Terminal only."
  exit 1
fi

########################################
# TAB OPEN HELPERS (tracks per-tab shell PID)
########################################

open_tab() {
  local title="$1"
  shift

  local pidfile
  pidfile=$(mktemp)

  gnome-terminal --tab --title="$title" -- bash -c "
    echo \$BASHPID > '$pidfile'
    source \"$ROS_SETUP\" 2>/dev/null || true
    source \"$CATKIN_WS/devel/setup.bash\" 2>/dev/null || true
    $*
  " &

  while [[ ! -s $pidfile ]]; do sleep 0.1; done

  local shell_pid
  shell_pid=$(cat "$pidfile")

  rm -f "$pidfile"
  TERMINAL_PIDS+=("$shell_pid")
}

open_sitl_tab() {
  local pidfile
  pidfile=$(mktemp)

  gnome-terminal --tab --title="SITL" -- bash -c "
    echo \$BASHPID > '$pidfile'
    source \"$ROS_SETUP\" 2>/dev/null || true
    cd \"$ARDUCOPTER_DIR\" || exit 1
    ../Tools/autotest/sim_vehicle.py -f gazebo-iris
  " &

  while [[ ! -s $pidfile ]]; do sleep 0.1; done

  local shell_pid
  shell_pid=$(cat "$pidfile")
  rm -f "$pidfile"

  TERMINAL_PIDS+=("$shell_pid")
}

########################################
# CLEANUP
########################################

cleanup_all() {
  echo -e "${YELLOW}[CLEANUP] Killing all simulation processes...${RESET}"

  pkill -f gzserver 2>/dev/null || true
  pkill -f gzclient 2>/dev/null || true
  pkill -f sim_vehicle.py 2>/dev/null || true
  pkill -f rqt 2>/dev/null || true
  pkill -f "rosrun gazebo_drone" 2>/dev/null || true

  sleep 5
  pkill -2 -f "roslaunch gazebo_ros iris_world.launch" 2>/dev/null || true

  if ((${#TERMINAL_PIDS[@]})); then
    kill "${TERMINAL_PIDS[@]}" 2>/dev/null || true
  fi

  echo -e "${GREEN}[DONE] All tabs + processes closed.${RESET}"
}

########################################
# WAIT FOR SCRIPTS
########################################

wait_for_script_finish() {
  local script="$1"
  echo -e "${YELLOW}[INFO] Waiting for ${script} to finish...${RESET}"
  while pgrep -f "$script" >/dev/null 2>&1; do sleep 5; done
  echo -e "${GREEN}[INFO] ${script} finished.${RESET}"
}

########################################
# EXTRA SCRIPT PROMPT
########################################

prompt_extra_script() {
  echo
  read -p "Run another script? Enter name or type CLOSE: " NEXT
  if [[ "$NEXT" == "CLOSE" ]]; then cleanup_all; exit 0; fi
  if [[ -n "$NEXT" ]]; then
    open_tab "DRONE:$NEXT" "
      chmod +x \"$DRONE_SCRIPT_DIR\"/*
      rosrun gazebo_drone \"$NEXT\"
    "
  fi
}

########################################
# LAUNCH FUNCTIONS
########################################

launch_gazebo_with_world() {
  local WORLD="$1"
  open_tab "GAZEBO:$WORLD" "
    cd \"$CATKIN_WS\" || exit 1
    catkin_make
    roslaunch gazebo_ros iris_world.launch world_name:=$WORLD
  "
}

launch_gazebo_interactive() {
  echo
  read -p "World file [default: new_multi.world]: " WORLD
  WORLD=${WORLD:-new_multi.world}
  launch_gazebo_with_world "$WORLD"
}

launch_sitl() {
  open_sitl_tab
}

launch_mission_script() {
  echo
  read -p "Mission script [default: final_report.py]: " SCRIPT
  SCRIPT=${SCRIPT:-final_report.py}

  open_tab "MISSION:$SCRIPT" "
    chmod +x \"$DRONE_SCRIPT_DIR\"/*
    rosrun gazebo_drone \"$SCRIPT\"
  "
}

launch_rqt() {
  open_tab "RQT" "rqt"
}

launch_emergency() {
  open_tab "EMERGENCY" "
    chmod +x \"$DRONE_SCRIPT_DIR\"/*
    rosrun gazebo_drone emerg.py
  "
}

########################################
# FULL PIPELINE
########################################

run_full_sequence() {
  echo
  read -p "World file [default: new_multi.world]: " WORLD
  WORLD=${WORLD:-new_multi.world}

  read -p "Mission script [default: final_report.py]: " SCRIPT
  SCRIPT=${SCRIPT:-final_report.py}

  launch_gazebo_with_world "$WORLD"
  echo "Waiting 60s for Gazebo..."
  sleep 60

  launch_sitl
  echo "Waiting 240s for SITL..."
  sleep 240

  open_tab "MISSION:$SCRIPT" "
    chmod +x \"$DRONE_SCRIPT_DIR\"/*
    rosrun gazebo_drone \"$SCRIPT\"
  "

  sleep 10
  launch_rqt

  wait_for_script_finish "$SCRIPT"
  prompt_extra_script
}

########################################
# RANDOM WORLD GENERATION (Option 7)
########################################

generate_random_world() {
  echo
  echo -e "${BLUE}[WORLD GENERATOR] Using world_script.py in:${RESET}"
  echo "  $WORLD_DIR"
  echo
  echo "Default: randomly_generated.world"
  read -p "Enter custom filename (.world) or ENTER for default: " TARGET

  if [[ -z "$TARGET" ]]; then
    TARGET="randomly_generated.world"
  fi

  if [[ "$TARGET" != *.world ]]; then
    echo -e "${RED}Invalid filename. Must end in .world.${RESET}"
    read -p "Press ENTER to return to menu..." _
    return
  fi

  if [[ ! -f "$WORLD_DIR/world_script.py" ]]; then
    echo -e "${RED}world_script.py not found.${RESET}"
    read -p "Press ENTER to return to menu..." _
    return
  fi

  echo -e "${YELLOW}Generating: $TARGET${RESET}"
  (
    cd "$WORLD_DIR" || exit 1
    python world_script.py > "$TARGET"
  )

  if [[ $? -eq 0 ]]; then
    echo -e "${GREEN}World generated: $WORLD_DIR/$TARGET${RESET}"
  else
    echo -e "${RED}Generation failed.${RESET}"
  fi

  read -p "Press ENTER to return to menu..." _
}

########################################
# HELP MENU (Option 0)
########################################

show_help() {
  clear
  echo -e "${BOLD}${BLUE}=========== HELP INFORMATION ===========${RESET}"

  echo -e "${YELLOW}World Files:${RESET}"
  echo -e "  $WORLD_DIR"

  echo
  echo -e "${YELLOW}Models:${RESET}"
  echo -e "  $MODEL_DIR"

  echo
  echo -e "${YELLOW}Drone Python Scripts:${RESET}"
  echo -e "  $DRONE_SCRIPT_DIR"

  echo
  echo -e "${YELLOW}Repository:${RESET}"
  echo -e "  $REPO_LINK"

  echo
  read -p "Press ENTER to return to menu..." _
}

########################################
# MAIN MENU
########################################

show_menu() {
  clear
  echo -e "${BOLD}${BLUE}"
  echo "==========================================="
  echo "         DRONE CONTROL CENTER CLI"
  echo "==========================================="
  echo -e "${RESET}"

  echo "  0) HELP"
  echo "  1) Launch Gazebo world"
  echo "  2) Launch SITL"
  echo "  3) Launch mission script"
  echo "  4) Launch rqt"
  echo "  5) Emergency retrieval"
  echo "  6) FULL pipeline"
  echo "  7) Generate random world"
  echo "  8) CLOSE ALL"
}

main_loop() {
  while true; do
    show_menu
    read -p "Choice [0-8]: " CHOICE
    case "$CHOICE" in
      0)
        show_help
        ;;
      1)
        launch_gazebo_interactive
        read -p "Press ENTER to return to menu..." _
        ;;
      2)
        launch_sitl
        read -p "Press ENTER to return to menu..." _
        ;;
      3)
        launch_mission_script
        read -p "Press ENTER to return to menu..." _
        ;;
      4)
        launch_rqt
        read -p "Press ENTER to return to menu..." _
        ;;
      5)
        launch_emergency
        read -p "Press ENTER to return to menu..." _
        ;;
      6)
        run_full_sequence
        read -p "Press ENTER to return to menu..." _
        ;;
      7)
        generate_random_world
        # generate_random_world already has its own ENTER pause,
        # but this extra one doesn't hurt; you can remove this line if redundant.
        ;;
      8)
        cleanup_all
        exit 0
        ;;
      *)
        echo -e "${RED}Invalid choice.${RESET}"
        sleep 1
        ;;
    esac
  done
}

main_loop

