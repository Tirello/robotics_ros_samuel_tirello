# robotics_ros_samuel_tirello

Atividades de ROS 2 e Docker do Curso de Programação e Robótica do LabRoM (projeto SONHO).

As demais atividades do curso (Linux, C++, Python, Git e Teoria de Robótica) ficam no repositório
[robotics_course_samuel_tirello](https://github.com/sonho-usp/robotics_course_samuel_tirello).

## Estrutura

```
robotics_ros_samuel_tirello/
├── docker/                         ambiente de desenvolvimento
│   ├── config/
│   │   ├── bashrc                  carrega o ambiente ROS 2 ao abrir o terminal
│   │   └── tools.sh                variáveis usadas pelos scripts
│   ├── scripts/
│   │   ├── build.sh                constrói a imagem
│   │   └── run.sh                  inicia o container
│   └── robot.dockerfile            imagem de desenvolvimento (base ros:jazzy)
├── ros_ws/                         workspace ROS 2
│   └── src/
│       └── go_to_goal/             controlador go-to-goal (Projeto 1)
├── .gitignore
└── README.md
```

As pastas `ros_ws/build/`, `ros_ws/install/` e `ros_ws/log/` são geradas pelo `colcon build` e
estão no `.gitignore` — quem clonar o repositório as recria rodando `colcon build`.

> **Nomenclatura:** `docker/` e `ros_ws/` correspondem ao `Docker` e ao `ROS (ros_ws)` da estrutura
> pedida no Discord. Os nomes em minúsculo seguem a Aula 2 (Docker e ROS), porque os scripts
> `tools.sh` e `run.sh` referenciam o diretório pelo nome `ros_ws`.

## Requisitos

- Docker instalado e com o serviço ativo
- Host Linux com X11, para as ferramentas gráficas (`rviz2`, `rqt_graph`)

## Início rápido

Todos os comandos do host rodam a partir da raiz do repositório.

```bash
bash docker/scripts/build.sh    # constrói a imagem (só na primeira vez)
bash docker/scripts/run.sh      # inicia o container
```

Já **dentro do container**:

```bash
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 run go_to_goal go_to_goal_node
```

> O `run.sh` inicia o container com `--rm`: ao sair da shell, ele é removido. O que estiver em
> `ros_ws/` sobrevive, porque é volume montado do host. Para abrir outra aba no mesmo container:
>
> ```bash
> docker exec -it ros_container bash
> ```

## Pacotes

### `go_to_goal`

Controlador go-to-goal para base móvel de tração diferencial. Recebe um objetivo e a pose atual do
robô e publica o comando de velocidade que leva um até o outro.

| Interface | Tópico | Tipo |
|---|---|---|
| Entrada | `/goal` | `geometry_msgs/msg/Vector3` |
| Entrada | `/robot_position` | `geometry_msgs/msg/Pose` |
| Saída | `/cmd_vel` | `geometry_msgs/msg/TwistStamped` |

A publicação acontece a **10 Hz**, independentemente da chegada de novas mensagens. Enquanto nenhum
objetivo tiver sido recebido, o nó publica velocidade nula — mas continua publicando: silêncio não
equivale a comando de parada, porque a maioria das bases mantém a última velocidade até um timeout.

#### Arquitetura

```
go_to_goal/
├── include/go_to_goal/
│   ├── geometry.hpp        wrap_to_pi, saturate, yaw_from_quaternion
│   └── controller.hpp      ControllerGains, compute_velocity_command
├── src/
│   └── go_to_goal_node.cpp camada ROS: assinaturas, parâmetros, temporizador
├── test/
│   └── test_controller.cpp 7 testes do controlador
├── CMakeLists.txt
├── package.xml
├── README.md
└── LICENSE
```

A lei de controle vive fora do nó, em cabeçalhos que não dependem do `rclcpp`:

```cpp
geometry_msgs::msg::Twist compute_velocity_command(
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Vector3 & goal,
  const ControllerGains & gains);
```

Entram dados, sai um comando — sem estado interno e sem efeito colateral. O nó é a camada fina que
traduz ROS para essa função e de volta. A consequência prática é que o algoritmo pode ser testado
sem inicializar o ROS, e reutilizado por outro nó sem copiar código.

#### Parâmetros

| Nome | Padrão | Faixa | Descrição |
|---|---|---|---|
| `kp_linear` | 0.5 | 0 – 10 | ganho proporcional da velocidade linear |
| `kp_angular` | 1.5 | 0 – 10 | ganho proporcional da velocidade angular |
| `max_linear_velocity` | 1.0 | 0 – 5 | velocidade linear máxima [m/s] |
| `max_angular_velocity` | 2.0 | 0 – 10 | velocidade angular máxima [rad/s] |
| `position_tolerance` | 0.05 | 0 – 1 | raio de chegada [m] |
| `frame_id` | `base_link` | — | frame do comando publicado |

Cada parâmetro é declarado com descrição e faixa de valores válidos, então um valor fora do
intervalo é recusado pelo próprio ROS:

```bash
ros2 param describe /go_to_goal_node kp_linear
ros2 param set /go_to_goal_node kp_linear 0.2      # aplicado na publicação seguinte
ros2 param set /go_to_goal_node kp_linear 999.0    # recusado: fora da faixa
```

A taxa de 10 Hz **não** é parâmetro: é requisito da interface com quem consome `/cmd_vel`, não
ajuste de sintonia.

#### Testes

```bash
cd ~/ros_ws
colcon test --packages-select go_to_goal
colcon test-result --all
```

São 32 checagens: 7 testes unitários do controlador — que rodam em cerca de um segundo, sem subir o
ROS — e os seis linters do ROS 2 (`cpplint`, `uncrustify`, `cppcheck`, `copyright`, `lint_cmake`,
`xmllint`), todos habilitados.

> Nesta imagem, as 4 checagens do `cppcheck` (uma por arquivo-fonte) aparecem como `skipped`: o
> `ament_cppcheck` se desabilita sozinho na versão 2.13.0 do `cppcheck`, por uma limitação de
> desempenho conhecida dessa versão. O que reprova a entrega é `errors` e `failures`, e os dois
> ficam em zero.

#### Verificar em execução

Com o nó rodando, em outra aba do container:

```bash
ros2 topic hz /cmd_vel                          # ~10 Hz
ros2 topic echo /cmd_vel --once --field twist   # nulo enquanto não houver objetivo

ros2 topic pub --once /robot_position geometry_msgs/msg/Pose \
  "{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}"
ros2 topic pub --once /goal geometry_msgs/msg/Vector3 "{x: 3.0, y: 0.0, z: 0.0}"
ros2 topic echo /cmd_vel --once --field twist   # linear.x saturado, angular.z nulo

rqt_graph                                       # os dois tópicos de entrada e o de saída
```

## Fluxo de trabalho

1. **Host:** `bash docker/scripts/run.sh` para subir o ambiente.
2. **Host ou container:** editar os arquivos em `ros_ws/src/`. A pasta é volume montado, então é a
   mesma dos dois lados.
3. **Container:** `colcon build --symlink-install` na raiz do workspace (`~/ros_ws`), nunca de
   dentro de `src/`.
4. **Container:** `colcon test` antes de considerar pronto.
5. **Host:** `git` — commits e push. O `.git` fica fora do volume, então **não** é visível de dentro
   do container.

## Convenção de branches

Cada atividade é desenvolvida em uma branch própria, no formato `area/item`, e integrada à `main`
por Pull Request. É a mesma convenção do repositório
[robotics_course_samuel_tirello](https://github.com/sonho-usp/robotics_course_samuel_tirello),
onde as listas usam `cpp/lista-2`, `python/lista-4`, `linux/lista_6`.

| Formato | Exemplo |
|---|---|
| `ros/lista-N` | listas de ROS 2 |
| `ros/projeto-N` | projetos de ROS 2 — `ros/projeto-1` |
| `docker/lista-N` | atividades de ambiente |
| `docs/<assunto>` · `chore/<assunto>` | revisão de documentação ou manutenção, quando não é atividade nova |

Commits seguem o padrão [Conventional Commits](https://www.conventionalcommits.org/): `feat:` para
funcionalidade, `fix:` para correção, `docs:` para documentação.

## Atividades

| Atividade | Onde |
|---|---|
| Aula 2 — Docker e ROS: montagem do ambiente | `docker/` |
| Projeto 1 — Publisher e Subscriber (`go_to_goal_node`) | `ros_ws/src/go_to_goal/` |

## Licença

MIT. Ver `ros_ws/src/go_to_goal/LICENSE`.
