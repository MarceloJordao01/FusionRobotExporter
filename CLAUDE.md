# FusionRobotExporter

Script Fusion 360 para exportar modelos CAD para URDF (ROS1/ROS2) e SDF (Gazebo).

## Arquitetura

```
FusionRobotExporter/
├── FusionRobotExporter.py        # Entry point + UI (CommandInputs)
├── exporters/
│   ├── urdf_ros1/
│   │   ├── exporter.py           # export(design, save_dir, options)
│   │   ├── joint.py              # Joint class + make_joints_dict()
│   │   ├── link.py               # Link class + make_inertial_dict()
│   │   ├── write.py              # write_urdf(), write_*_launch(), etc.
│   │   └── utils.py              # copy_occs(), export_stl(), prettify()
│   ├── urdf_ros2/
│   │   ├── exporter.py
│   │   ├── joint.py
│   │   ├── link.py
│   │   ├── write.py
│   │   ├── launch_templates.py   # Templates Python para .launch.py
│   │   └── utils.py
│   └── sdf/
│       ├── exporter.py
│       ├── sdf.py                # Classe SDF - parsing e geração
│       ├── joint.py              # Joint + JointType enum
│       ├── link.py               # Link + LinkInertial + LinkGeometry
│       ├── pose.py               # Pose com Transform
│       ├── transform.py          # Matriz 4x4 homogênea
│       ├── log.py                # Logging para Text Commands
│       └── util.py               # normalize_name(), cm_to_m(), etc.
└── core/                         # Compartilhado (a implementar)
```

## Fluxo de Exportação

1. **UI** (`FusionRobotExporter.py`)
   - Cria CommandInputs (dropdowns, checkboxes)
   - Coleta opções do usuário
   - Chama exportador apropriado

2. **Exporter** (`exporters/*/exporter.py`)
   - `export(design, save_dir, options) -> (success, message)`
   - Extrai joints e links do design
   - Gera arquivos de saída

3. **Joint/Link** (`joint.py`, `link.py`)
   - `make_joints_dict(root)` - extrai joints da API Fusion
   - `make_inertial_dict(root)` - calcula massa/inércia
   - Classes para gerar XML

4. **Write** (`write.py`)
   - Funções para escrever cada tipo de arquivo
   - URDF, XACRO, launch files, YAML, etc.

## API Fusion 360 Usada

```python
# Design
app.activeProduct  # -> adsk.fusion.Design
design.rootComponent
design.allComponents

# Componentes
component.occurrences
occurrence.bRepBodies
occurrence.transform / transform2
occurrence.getPhysicalProperties()

# Joints
root.joints
joint.jointMotion.jointType
joint.jointMotion.rotationAxisVector
joint.occurrenceOne / occurrenceTwo
joint.geometryOrOriginOne / geometryOrOriginTwo

# Export
design.exportManager
exportManager.createSTLExportOptions()
exportManager.createOBJExportOptions()

# UI
ui.commandDefinitions.addButtonDefinition()
cmd.commandInputs.addDropDownCommandInput()
cmd.commandInputs.addBoolValueInput()
cmd.commandInputs.addGroupCommandInput()
```

## Conversões de Unidades

- Fusion usa **cm**, URDF/SDF usam **metros**
- `xyz / 100.0` para posições
- Inércia: `kg/cm² / 10000.0` para `kg/m²`
- Parallel axis theorem para converter inércia world → center of mass

## Deploy

```powershell
.\deploy.ps1  # Copia para %APPDATA%\Autodesk\...\API\Scripts\
```

## TODO

- [ ] Unificar código duplicado entre urdf_ros1 e urdf_ros2
- [ ] Adicionar suporte a mais tipos de joint (Ball, Cylindrical)
- [ ] Opção de simplificação de mesh
- [ ] Progress bar durante exportação
- [ ] Validação do modelo antes de exportar
