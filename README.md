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
   - **Base Link**: componente/grupo raiz do robô
   - **Define links by Rigid Group**: alterna o modo de definição de links
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

## Modos de Link

O checkbox **Define links by Rigid Group** define como os links são montados:

| Modo | Comportamento |
|------|---------------|
| **Components** (padrão) | Cada componente vira um link. |
| **Rigid Groups** | Cada Rigid Group **do nível principal da montagem** vira **um link fundido** (todos os bodies das ocorrências do grupo juntos). Rigid groups definidos dentro de submontagens são **ignorados**; componentes fora de qualquer grupo também são **ignorados**. |

Use o modo **Rigid Groups** quando quiser tratar uma submontagem inteira como um
único link — por exemplo, agrupar várias peças do chassi em um rigid group e
exportá-lo como `base_link`. Nesse modo o dropdown **Base Link** lista os rigid
groups; as juntas do Fusion entre peças de grupos diferentes viram as juntas
entre os links.

## Sensores

Sensores são adicionados criando um **componente** na montagem com nome no
formato (separador é **duplo underscore** `__`):

```
sensor__<tipo>__<link>__<nome>
```

- `<tipo>`: `camera`, `depth`, `lidar`, `imu`, `gps`, `contact`
- `<link>`: link onde o sensor monta (ex.: `base_link`)
- `<nome>`: opcional (default = tipo)

Exemplos: `sensor__camera__base_link__frontal`, `sensor__lidar__base_link__topo`,
`sensor__imu__base_link`.

A pose e a orientação do sensor são tiradas do transform do componente relativo
ao link (oriente o frame do componente para onde a câmera/LiDAR aponta). O
componente em si **não** vira link nem gera mesh. Gera blocos `<sensor>` para
SDF/Gazebo e `<gazebo><sensor>` para URDF (ROS1 = gazebo_ros classic, ROS2 = Gz).

Cada tipo tem defaults razoáveis (FOV, resolução, alcance, `update_rate`) — ver
`SENSOR_DEFAULTS` em `core/sensors.py`.

### Sensores × "Define links by Rigid Group"

O `<link>` no nome do sensor **sempre** se refere a um nome de link do URDF/SDF
gerado — e quais nomes existem depende do modo de link. O componente do sensor é
sempre localizado pelo **nome** (varredura recursiva da montagem); ele **não
precisa** estar dentro de nenhum rigid group. O que muda entre os modos é (1)
**quais nomes de `<link>` são válidos** e (2) **em relação a qual frame a pose do
sensor é calculada**:

**Checkbox DESMARCADO — modo Components (um componente = um link):**

- `<link>` deve ser o nome de um **componente** que virou link, ou seja, o nome
  da ocorrência normalizado (`normalize_name`), ou `base_link` para o componente
  base escolhido no dropdown.
- A pose do sensor é calculada **relativa ao frame do componente** daquele link
  (`T_link⁻¹ · T_sensor`, onde `T_link` é o transform de mundo da ocorrência).
- Mapa link→frame montado por `build_component_link_transforms`.
- Ex.: se a perna do robô é o componente `perna_dir`, use
  `sensor__contact__perna_dir__pe`.

**Checkbox MARCADO — modo Rigid Groups (um rigid group = um link fundido):**

- `<link>` deve ser o nome de um **rigid group** (do nível principal da montagem)
  normalizado, ou `base_link` para o rigid group base selecionado no dropdown.
  Componentes individuais **não** são links neste modo, então nomes de componente
  não casam — use o nome do grupo.
- A pose do sensor é calculada relativa ao **frame do link de grupo**, que é
  alinhado ao mundo e transladado para o ponto da junta `P` daquele link (a mesma
  convenção da mesh/junta), **não** ao frame de um componente específico. Mapa
  link→frame montado por `build_rigid_link_transforms` a partir dos `link_frames`.
- O componente do sensor pode estar solto (fora de qualquer rigid group) — mesmo
  sendo ignorado como link, ele continua sendo detectado como sensor pelo nome.
- Ex.: se o rigid group `rotor_1` virou um link, use
  `sensor__imu__rotor_1__medidor`.

Em ambos os modos, se o `<link>` informado não existir entre os links gerados, o
sensor é pulado e um aviso aparece no painel **Text Commands**
(`Sensor warning: ... link '<x>' not found`). Confira lá após exportar.

## Requisitos do Modelo Fusion 360

- Cada **link** deve ser um **componente separado** (modo Components) ou um
  **Rigid Group** (modo Rigid Groups)
- **Joints** conectam componentes (Revolute, Prismatic, Fixed)
- Componentes não devem ter subcomponentes aninhados
- Convenção de joints: Parent = Component2, Child = Component1

## Visualizador URDF/SDF (Docker)

Um container Docker com app Flask renderiza no navegador o URDF/SDF de um pacote
gerado (e, numa etapa futura, fará simplificação de malha). Os scripts ficam em
`meshSimplification/` e o `Dockerfile` em `docker/`.

```bat
REM 1) buildar a imagem (uma vez)
build.bat

REM 2) subir o Flask (sem precisar apontar a pasta na linha de comando)
run.bat
REM   base opcional (default = seu perfil de usuário) e porta:
REM     run.bat C:\Users\Administrador\Desktop
REM     run.bat C:\Users\Administrador\Desktop 8080
```

Abra `http://localhost:5000`. A pasta base é montada em `/data`. Clique em
**“Open URDF / SDF…”** para abrir uma **janela de browser** (modal) e navegar até
o `.urdf`/`.xacro`/`.sdf`; ao clicar no arquivo ele é renderizado (STL/OBJ/DAE,
eixo Z para cima, com OrbitControls). O default monta `%USERPROFILE%`, então
qualquer pacote gerado sob o seu usuário fica acessível; passe um caminho para
restringir.

As **joints** também são mostradas no estilo do RViz: em cada junta um **triedro
RGB** (X=vermelho, Y=verde, Z=azul) na orientação do frame da junta, com um rótulo
com o **nome da junta**. Controles na lateral:
- **Show joints** / **Show joint names** — liga/desliga.
- **Joint name size** — tamanho da fonte do nome (multiplicador).
- **Frame size** — tamanho do triedro (multiplicador).

Os tamanhos-base escalam pelo bounding box do modelo; os sliders aplicam um
multiplicador por cima e atualizam ao vivo.

> Ao alterar os scripts em `meshSimplification/`, **rebuilde** a imagem
> (`build.bat`) antes do `run.bat` — o código é copiado para dentro da imagem no
> build.

Como funciona: o parsing é **server-side** (`meshSimplification/scene.py`) e
gera uma "cena" plana (lista de visuais já posicionados no mundo) que o viewer
Three.js (`static/viewer.js`) só carrega e desenha:
- **URDF/xacro**: `xacro:include` é inlinado, `$(find pkg)` e `package://pkg/...`
  resolvem para a raiz do pacote; os frames dos links vêm de FK na configuração
  zero das juntas; cores vêm de `materials.xacro`.
- **SDF**: usa os `<pose>` de cada `<link>`/`<visual>` (relativos ao modelo).

Mesh simplification ainda **não** está implementada — o foco atual é só
renderizar. As deps extras já estão anotadas em `meshSimplification/requirements.txt`.

> Requer Docker Desktop rodando. O `deploy.ps1` **não** copia `docker/`,
> `meshSimplification/`, `build.bat` nem `run.bat` para o Fusion (não são parte
> do script).

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
│   ├── mesh.py               # Exportação STL/OBJ compartilhada
│   ├── rigid_groups.py       # Modo "link por Rigid Group"
│   └── sensors.py            # Sensores por convenção de nome
├── exporters/
│   ├── urdf_ros1/            # Exportador ROS1
│   ├── urdf_ros2/            # Exportador ROS2
│   └── sdf/                  # Exportador SDF
├── docker/
│   └── Dockerfile            # Imagem do visualizador/simplificador
├── meshSimplification/       # App Flask (render URDF/SDF; simplificação depois)
│   ├── app.py
│   ├── scene.py              # URDF/SDF -> cena JSON (server-side)
│   ├── templates/index.html
│   └── static/viewer.js      # Viewer Three.js
├── build.bat                 # Builda a imagem Docker
└── run.bat                   # Roda o container montando a pasta do pacote
```

## Referências

Baseado em:
- [fusion2urdf](https://github.com/syuntoku14/fusion2urdf) - URDF ROS1
- [fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2) - URDF ROS2
- [FusionSDF](https://github.com/andreasBihlmaier/FusionSDF) - SDF