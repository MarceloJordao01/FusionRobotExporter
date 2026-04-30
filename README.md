# FusionRobotExporter

Script para Fusion 360 que exporta modelos CAD para formatos de simulação robótica.

## Formatos Suportados

| Formato | Destino | Meshes | Launch Files |
|---------|---------|--------|--------------|
| URDF (ROS1) | RViz + Gazebo (catkin) | STL | XML |
| URDF (ROS2) | RViz2 + Gazebo (colcon) | STL | Python |
| SDF | Gazebo | OBJ | - |

## Instalação

### Windows

```powershell
cd FusionRobotExporter
.\deploy.ps1
```

### macOS/Linux

```bash
cd FusionRobotExporter
./deploy.sh
```

O script copia os arquivos para `%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\FusionRobotExporter`.

## Uso

1. Abrir modelo no Fusion 360
2. Pressionar **Shift+S** (Scripts and Add-Ins)
3. Selecionar **FusionRobotExporter** → **Run**
4. Configurar opções na UI:
   - **Format**: URDF ou SDF
   - **ROS Version**: ROS1 ou ROS2 (apenas para URDF)
   - **Base Link**: componente raiz do robô
   - **Export Options**: meshes, collision, inertia, launch files
   - **Mesh Options**: formato (STL/OBJ), tipo de colisão

5. Selecionar pasta de destino

## Saída Gerada

### URDF ROS1

```
robot_description/
├── urdf/
│   ├── robot.xacro
│   ├── materials.xacro
│   ├── transmissions.xacro
│   └── gazebo.xacro
├── launch/
│   ├── display.launch
│   ├── gazebo.launch
│   └── controller.launch
├── config/
│   └── controller.yaml
└── meshes/
    └── *.stl
```

### URDF ROS2

```
robot_description/
├── urdf/
│   └── *.xacro
├── launch/
│   ├── display.launch.py
│   └── gazebo.launch.py
├── config/
├── meshes/
│   └── *.stl
├── resource/
├── robot_description/
│   └── __init__.py
├── setup.py
└── package.xml
```

### SDF

```
output_dir/
├── model.sdf
└── meshes/
    └── *.obj
```

## Requisitos do Modelo Fusion 360

- Cada **link** deve ser um **componente separado**
- **Joints** conectam componentes (Revolute, Prismatic, Fixed)
- Componentes não devem ter subcomponentes aninhados
- Convenção de joints: Parent = Component2, Child = Component1

## Joints Suportados

| Fusion 360 | URDF/SDF |
|------------|----------|
| Revolute | revolute |
| Slider | prismatic |
| Rigid | fixed |

## Estrutura do Projeto

```
FusionRobotExporter/
├── FusionRobotExporter.py    # Entry point + UI
├── FusionRobotExporter.manifest
├── core/
│   └── mesh.py               # Exportação STL/OBJ compartilhada
└── exporters/
    ├── urdf_ros1/            # Exportador ROS1
    ├── urdf_ros2/            # Exportador ROS2
    └── sdf/                  # Exportador SDF
```

## Referências

Baseado em:
- [fusion2urdf](https://github.com/syuntoku14/fusion2urdf) - URDF ROS1
- [fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2) - URDF ROS2
- [FusionSDF](https://github.com/andreasBihlmaier/FusionSDF) - SDF