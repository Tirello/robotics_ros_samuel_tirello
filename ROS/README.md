# ROS

Workspace ROS 2 (`ros_ws`) com os pacotes das atividades da disciplina.

## Estrutura

```
ROS/
`-- src/
    `-- projeto/
        `-- pacote_entrega/
```

Os pacotes ficam sob `src/`, agrupados por projeto. Cada pacote tem um README proprio
descrevendo os nos, topicos e como executar.

## Listas

Nenhum pacote entregue ate o momento.

## Como compilar e rodar

A partir desta pasta, que e a raiz do workspace:

```bash
colcon build
source install/setup.bash
ros2 run <pacote> <no>
```
