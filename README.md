# F1TENTH gym environment ROS2 communication bridge
This is a containerized ROS communication bridge for the F1TENTH gym environment that turns it into a simulation in ROS2.

The installed ros2 in the container is a foxy version but as soon as car1 and car2 will be able to use humble, I will update the container as well.

**Supported System:**

- Windows 10, macOS, and Ubuntu

## Prerequisites (Before Starting)
- Install Docker Desktop (required to run containers)
- Install Visual Studio Code (for attaching to the running simulation container)
- (Optional) Install Git if you prefer cloning instead of ZIP download

## Windows Quick Start (English / Magyar)
| English | Magyar |
|--------|--------|
| 1. Download the repository: click the Code button (green) on GitHub, choose Download ZIP. | 1. Töltsd le a repót: kattints a Code (zöld) gombra GitHub-on, majd Download ZIP. |
| 2. Extract the ZIP to your Desktop (you should get: Desktop/Simulator-main). | 2. Csomagold ki a ZIP-et az Asztalra (eredmény: Asztal/Simulator-main mappa). |
| 3. Install Docker Desktop and start it (must be running, no errors). | 3. Telepítsd a Docker Desktopot és indítsd el (futnia kell hiba nélkül). |
| 4. Install Visual Studio Code. | 4. Telepítsd a Visual Studio Code-ot. |
| 5. Open Windows PowerShell or Command Prompt. | 5. Nyisd meg a Windows PowerShellt vagy a Parancssort (CMD). |
| 6. Navigate into the project folder: cd .\\Desktop\\Simulator-main | 6. Lépj be a mappába: cd .\\Desktop\\Simulator-main |
| 7. Start the containers: docker compose up (or docker-compose up). | 7. Indítsd a konténereket: docker compose up (vagy docker-compose up). |
| 8. Wait until services finish starting (no new error spam). | 8. Várj, míg minden elindul (nincsenek új hibák). |
| 9. Open browser: http://localhost:8080/vnc.html then click Connect (Ubuntu desktop should appear). | 9. Nyiss böngészőt: http://localhost:8080/vnc.html majd Connect (Ubuntu felület látszik). |
| 10. In VS Code install the "Dev Containers" extension (formerly Remote - Containers) and optionally "Docker" extension. | 10. VS Code-ban telepítsd a "Dev Containers" (régen Remote - Containers) és opcionálisan a "Docker" kiegészítőt. |
| 11. Press Ctrl+Shift+P → "Dev Containers: Attach to Running Container..." | 11. Ctrl+Shift+P → "Dev Containers: Attach to Running Container..." |
| 12. Select the simulation container (the one WITHOUT 'novnc' in its name). | 12. Válaszd ki a szimulációs konténert (amelyik nevében NINCS 'novnc'). |
| 13. In the attached VS Code window open a terminal (Terminal > New Terminal). | 13. A csatolt VS Code ablakban nyiss egy terminált (Terminal > New Terminal). |
| 14. Source the workspace: source sim_ws/install/setup.bash | 14. Forrásold a workspace-et: source sim_ws/install/setup.bash |
| 15. Launch simulation: ros2 launch f1tenth_gym_ros bringup_launch.py | 15. Indítsd a szimulációt: ros2 launch f1tenth_gym_ros bringup_launch.py |
| 16. Open/refresh http://localhost:8080/vnc.html — you should now see the track and cars. | 16. Frissítsd / nyisd meg újra a http://localhost:8080/vnc.html oldalt — látnod kell a pályát és az autókat. |
| 17. (Optional) Use RViz inside the container if needed. | 17. (Opcionális) Használd az RViz-et a konténerben ha szükséges. |
| 18. To stop: press Ctrl+C in the terminal, then docker compose down (in host shell). | 18. Leállítás: terminálban Ctrl+C, majd hoston docker compose down. |

> If the ros2 launch command succeeds and no errors appear, the simulation is running.  
> Ha a ros2 launch parancs sikeresen fut és nincs hiba, a szimuláció fut.

# Launching the Simulation

```bash
$ cd sim_ws
$ colcon build
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

## Quick Start (Necessary Steps to Use the Simulator)

1. Open a terminal and navigate to the project folder:
   ```bash
   cd /path/to/Simulator-main
   ```
2. Start the containers:
   ```bash
   docker compose up
   ```
3. Open Visual Studio Code and attach to the running simulation container (use "Dev Containers: Attach to Running Container...").
4. In the attached VS Code window, open a terminal and run:
   ```bash
   $ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```

A rviz window should pop up showing the simulation either on your host system or in the browser window depending on the display forwarding you chose.


