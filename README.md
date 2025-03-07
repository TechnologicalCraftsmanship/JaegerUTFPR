# Jaeger UTFPR Project Repository

This repository organizes all project files related to the **Jaeger** system, developed as part of ongoing research at **UTFPR**. It includes mechanical parts, electronic fabrication files, embedded system workspaces, and a VR application.

---

## 📂 Folder Structure

.

├── 3d_Models

│   └── models

├── Fabrication

│   ├── Jaeger_5_Rev1

│   │   └── gerber

│   └── Jaeger_Driver_3_Rev1

│       └── gerber

├── JaegerUTFPR_Workspace

│   └── MainProject

└── VR_Application



---

## 📁 Folder Descriptions

### `3d_Models/`
Contains all mechanical 3D models designed for the project that must be printed.

- `models/`: Subfolder holding specific 3D models

---

### `Fabrication/`
Houses the PCB designs and manufacturing files for the Jaeger hardware components.

- `Jaeger_5_Rev1/`: Files related to **Jaeger 5 Revision 1**, which include schematic diagrams and layout files.
  - `gerber/`: Exported Gerber files for PCB fabrication of **Jaeger 5 Rev1**.
  
- `Jaeger_Driver_3_Rev1/`: Files related to **Jaeger Driver 3 Revision 1**, responsible for driving specific modules or actuators.
  - `gerber/`: Exported Gerber files for PCB fabrication of **Jaeger Driver 3 Rev1**.

---

### `JaegerUTFPR_Workspace/`
This folder is the workspace for embedded development, using ESP-IDF.

- `MainProject/`: The main embedded firmware source code, configuration files, and dependencies for the Jaeger system.

---

### `VR_Application/`

Contains the virtual reality application developed for project visualization, simulation, or remote monitoring. This may include **Unity** project files, assets, and scripts for interacting with the Jaeger system in a 3D environment.

---

## 🚀 Purpose
This repository aims to centralize all essential components of the Jaeger project, ensuring easy access and version control across mechanical design, hardware development, firmware programming, and VR visualization.

---

## 📝 License

This project is licensed under the [MIT License](LICENSE).

