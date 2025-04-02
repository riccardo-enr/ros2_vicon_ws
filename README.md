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
