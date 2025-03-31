# ROS 2 Vicon Workspace

Workspace to be used on the UAVs for communication with the Vicon system.
This workspace contains the following packages:
* `vrpn_mocap`
* `px4_msgs`
* `Micro-XRCE-DDS-Agent`

## Installation

1. Clone the repository
2. Install the submodules

    ```bash
    git submodule update --init --recursive
    ```

## Devcontainer installation

- Ensure the VSCode devcontainer extension is installed
- Ensure that Docker is installed and running
- Open the folder from the workspace path
- Open the command palette (Ctrl+Shift+P) and select "Remote-Containers: Open Folder in Container..."


