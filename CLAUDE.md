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
└── core/
    ├── mesh.py                   # Exportação de meshes STL/OBJ
    └── rigid_groups.py           # Modo "link por Rigid Group" (compartilhado)
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

## Base Link

O link raiz do robô é escolhido pelo usuário no dropdown **Base Link** da UI
(populado com `root.allOccurrences`). O nome da ocorrência selecionada é passado
como `options['base_link']` e usado em `make_joints_dict(root, msg, base_link_name)`
e `make_inertial_dict(root, msg, base_link_name)`, onde a ocorrência correspondente
é registrada com a chave `'base_link'`.

Fallback: se nenhum base link for informado, o exportador procura um componente
cujo nome seja `base_link` (comportamento legado).

## Modos de Link (`options['link_mode']`)

A UI tem um checkbox **"Define links by Rigid Group"** que define como os links
são montados. O valor vai em `options['link_mode']`:

- `'components'` (padrão): **um componente = um link** (comportamento clássico).
- `'rigid_groups'`: **cada Rigid Group do nível principal da montagem = um link
  fundido** (todos os bodies das ocorrências do grupo juntos). Apenas os rigid
  groups do `rootComponent` são considerados — rigid groups definidos **dentro de
  submontagens são ignorados**. Componentes fora de qualquer rigid group também
  são **ignorados**. O dropdown Base Link passa a listar os rigid groups.

No modo `rigid_groups` o dropdown Base Link é repopulado com
`core.rigid_groups.list_rigid_group_names(root)` (ver `CommandInputChangedHandler`).

### `core/rigid_groups.py`
- `resolve_group_links(root, base_group_name)` → `[{group, link_name, group_name}]`
  (grupo base vira `'base_link'`).
- `occurrence_to_group_map(group_links)` → `{occurrence.name: link_name}`.
- `build_group_link_data(design, save_dir, group_links, link_frames, export_meshes)`
  → exporta a STL fundida **em coords locais do link** (ocorrência temporária
  transladada por `P`, bodies copiados, STL, `deleteMe()`) e calcula a inércia;
  retorna `inertial_dict` no mesmo formato de `link.py:make_inertial_dict`.

### Convenção de frames no modo rigid_groups (URDF)
Cada link de grupo usa frame **alinhado ao mundo**, transladado para o ponto da
junta que o liga ao pai (`P`). Assim o `<axis>` fica em coords de mundo (sem
conversão p/ frame do filho), o `<origin>` da junta é `P_child − P_parent`, e a
mesh é exportada já em coords locais (visual origin `0`). Implementado em
`exporters/urdf_*/joint.py:make_rigid_group_joints_dict`.

### SDF
O SDF **já** fundia rigid groups; agora é **condicional** ao `link_mode`. Em
`components` cada ocorrência vira link (não funde); em `rigid_groups` funde e
ignora soltos (`add_link` pula ocorrências fora de grupo). O grupo base
selecionado vira o `root_link` (`sdf.py:parse_root_component`).

## Sensores (`core/sensors.py`)

Sensores são definidos por **convenção de nome de componente** na montagem:

```
sensor__<tipo>__<link>[__<nome>]
```

- Separador é **duplo underscore** `__` (parseado do nome BRUTO do componente,
  antes de `normalize_name`, para preservar underscores simples de `base_link`).
- `<tipo>` ∈ `camera`, `depth`, `lidar`, `imu`, `gps`, `contact`.
- `<link>` = link alvo (normalizado); `<nome>` opcional (default = tipo).
- Ex.: `sensor__camera__base_link__frontal`, `sensor__lidar__base_link__topo`.

A **pose/orientação** vem do transform do componente relativo ao frame do link
(`_relative_xyz_rpy`). Componentes `sensor__*` são **excluídos** de links/juntas/
meshes (filtro em `urdf_*/link.py:make_inertial_dict`, `core/mesh.py:export_stl`
e `sdf.py:add_link`).

Geração:
- **URDF ROS1/ROS2** (`make_urdf_sensor_xml`): blocos
  `<gazebo reference="link"><sensor>…</sensor></gazebo>` no `.gazebo` xacro.
  ROS1 = plugins Gazebo classic (`libgazebo_ros_*`); ROS2 = Gz novo (sistema de
  sensores do Gz, bridge via ros_gz). Ligado em `write.py:write_gazebo_xacro`.
- **SDF** (`make_sdf_sensor_element`): `<sensor>` anexado ao `<link>` pai
  (`Link.sensors` em `sdf/link.py`; coletado em `sdf.py:attach_sensors`).

Defaults por tipo em `SENSOR_DEFAULTS` (FOV, resolução, alcance, update_rate).
Mapa link→transform: `build_component_link_transforms` (modo componentes) e
`build_rigid_link_transforms` (modo rigid groups). No SDF, o frame de mundo de
cada link é registrado em `self.link_world` durante `add_link`.

## TODO

- [ ] Unificar código duplicado entre urdf_ros1 e urdf_ros2
- [ ] Adicionar suporte a mais tipos de joint (Ball, Cylindrical)
- [ ] Opção de simplificação de mesh
- [ ] Progress bar durante exportação
- [ ] Validação do modelo antes de exportar
