# robotics_ros_samuel_tirello

Atividades de ROS 2 e Docker do Curso de Programacao e Robotica do LabRoM (projeto SONHO).

As demais atividades do curso (Linux, C++, Python, Git e Teoria de Robotica) ficam no repositorio
[robotics_course_samuel_tirello](https://github.com/sonho-usp/robotics_course_samuel_tirello).

## Temas

| Tema | Conteudo |
|---|---|
| [Docker](./Docker) | Conteinerizacao do ambiente de desenvolvimento em robotica |
| [ROS](./ROS) | Workspace ROS 2 (`ros_ws`) com os pacotes das atividades |

## Organizacao

```
robotics_ros_samuel_tirello/
|-- Docker/
|   `-- Lista N - Assunto/        uma pasta por atividade de Docker
|-- ROS/                          workspace ROS 2 (ros_ws)
|   `-- src/
|       `-- projeto/              uma pasta por projeto
|           `-- pacote_entrega/   pacotes ROS 2 da entrega
`-- README.md
```

## Fluxo de trabalho

Cada atividade nova e desenvolvida em uma branch propria, no padrao `tema/lista-N`
(por exemplo `ros/lista-1`, `docker/lista-1`), e so depois de concluida e integrada a `main`.
E o mesmo fluxo usado no repositorio principal do curso.
