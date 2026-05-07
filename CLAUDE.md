# FusionRobotExporter

Script Fusion 360 para exportar modelos CAD para URDF (ROS1/ROS2) e SDF (Gazebo).

## Arquitetura

```
FusionRobotExporter/
├── FusionRobotExporter.py        # Entry point + UI (CommandInputs)
├── sensors.example.json          # Exemplo de configuração de sensores
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
└── core/
    ├── mesh.py                   # Exportação de meshes STL/OBJ
    └── sensors.py                # Carregamento e geração de sensores
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

## Sensores

Sensores são definidos em um arquivo `sensors.json` no diretório de exportação.
O script busca automaticamente por `sensors.json` no diretório de saída.

### Estrutura do JSON

```json
{
  "sensors": [
    {
      "name": "camera_front",
      "type": "camera",
      "parent_link": "base_link",
      "pose": {
        "xyz": [0.1, 0, 0.05],
        "rpy": [0, 0, 0]
      },
      "params": {
        "update_rate": 30.0,
        "width": 640,
        "height": 480,
        "fov": 1.047,
        "clip": {"near": 0.1, "far": 100.0}
      }
    }
  ]
}
```

### Tipos de Sensores Suportados

| Tipo | Descrição | Parâmetros Principais |
|------|-----------|----------------------|
| `camera` | Câmera RGB | width, height, fov, clip |
| `depth_camera` | Câmera de profundidade | width, height, fov, clip |
| `lidar` | Sensor laser 2D | samples, range, angle |
| `imu` | Unidade inercial | update_rate, noise |

### Parâmetros de Câmera

| Parâmetro | Descrição | Valor Padrão |
|-----------|-----------|--------------|
| `width` | Largura da imagem em pixels | 640 |
| `height` | Altura da imagem em pixels | 480 |
| `fov` | Campo de visão horizontal (radianos) | 1.047 (~60°) |
| `clip.near` | Distância mínima de detecção (metros) | 0.1 |
| `clip.far` | Distância máxima de detecção (metros) | 100.0 |
| `update_rate` | Taxa de atualização (Hz) | 30.0 |

### Módulo core/sensors.py

```python
# Carregar sensores
sensors = load_sensors('sensors.json')

# Gerar URDF
xml = generate_sensors_urdf(sensors)           # Links + Joints
xml = generate_sensors_gazebo_urdf(sensors)    # Plugins Gazebo

# Gerar SDF
xml = generate_sensors_for_link_sdf(sensors, 'base_link')
```

## TODO

- [ ] Unificar código duplicado entre urdf_ros1 e urdf_ros2
- [ ] Adicionar suporte a mais tipos de joint (Ball, Cylindrical)
- [ ] Opção de simplificação de mesh
- [ ] Progress bar durante exportação
- [ ] Validação do modelo antes de exportar
- [x] Suporte a sensores via JSON
