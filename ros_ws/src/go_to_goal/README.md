# `go_to_goal`

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

## Arquitetura

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

## Parâmetros

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

## Testes

```bash
cd ~/ros_ws
colcon test --packages-select go_to_goal
colcon test-result --all
```

São 32 checagens: 7 testes unitários do controlador — que rodam em cerca de um segundo, sem subir o
ROS — e os seis linters do ROS 2 (`cpplint`, `uncrustify`, `cppcheck`, `copyright`, `lint_cmake`,
`xmllint`), todos habilitados.

## Verificar em execução

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

## Contexto

Projeto 1 (Publisher e Subscriber) do Curso de Programação e Robótica do LabRoM — projeto SONHO.
Repositório: [robotics_ros_samuel_tirello](https://github.com/Tirello/robotics_ros_samuel_tirello).

A lei de controle é a mesma do projeto de Movimento 2D, escrito em Python no semestre anterior,
sem o estágio final de alinhamento: como o alvo chega em um `Vector3`, é um ponto sem orientação,
e não existe yaw de destino a ajustar.

## Licença

MIT — ver [`LICENSE`](LICENSE).
