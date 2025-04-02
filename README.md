# Workspace ROS 2 Vicon

Workspace da utilizzare sugli UAV per la comunicazione con il sistema Vicon.
Questo workspace contiene i seguenti pacchetti:
* `vrpn_mocap`
* `px4_msgs`
* `Micro-XRCE-DDS-Agent`

## Installazione

1. Clonare la repository
2. Installare i sottomoduli

    ```bash
    git submodule update --init --recursive
    ```

## Installazione Devcontainer

- Assicurarsi che l'estensione devcontainer di VSCode sia installata
- Assicurarsi che Docker sia installato e in esecuzione
- Aprire la cartella dal percorso del workspace
- Aprire la command palette (Ctrl+Shift+P) e selezionare "Remote-Containers: Open Folder in Container..."

# Parametri autopilota

| **Parameter**  | **Changed value**       | **Default value**       |
| :------------- | :---------------------- | :---------------------- |
| EKF2_BARO_CTRL | 0 (Disabled)            | 1 (Enabled)             |
| EKF2_EV_CTRL   | 3 (Horizontal+Vertical) | 15 (All flags selected) |
| EKF2_EV_DELAY  | 10.0 ms                 | 0.0 ms                  |
| EKF2_HGT_REF   | 3 (Vision)              | 1 (GPS)                 |
