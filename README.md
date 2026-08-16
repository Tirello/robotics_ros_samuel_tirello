# robotics_ros_samuel_tirello

Atividades de ROS 2 e Docker do Curso de Programacao e Robotica do LabRoM (projeto SONHO).

As demais atividades do curso (Linux, C++, Python, Git e Teoria de Robotica) ficam no repositorio
[robotics_course_samuel_tirello](https://github.com/sonho-usp/robotics_course_samuel_tirello).

## Estrutura

```
robotics_ros_samuel_tirello/
|-- docker/                    ambiente de desenvolvimento
|   |-- config/
|   |   |-- bashrc             carrega o ambiente ROS 2 ao abrir o terminal
|   |   `-- tools.sh           variaveis usadas pelos scripts
|   |-- scripts/
|   |   |-- build.sh           constroi a imagem
|   |   `-- run.sh             inicia o container
|   `-- robot.dockerfile       imagem de desenvolvimento (base ros:jazzy)
|-- ros_ws/                    workspace ROS 2
|   `-- src/                   codigo fonte dos pacotes
`-- README.md
```

As pastas `ros_ws/build/`, `ros_ws/install/` e `ros_ws/log/` sao geradas pelo `colcon build` e
estao no `.gitignore` — qualquer pessoa que clonar o repositorio as recria rodando `colcon build`.

> Nomenclatura: `docker/` e `ros_ws/` correspondem ao `Docker` e ao `ROS (ros_ws)` da estrutura
> pedida no Discord. Os nomes em minusculo seguem a Aula 2 (Docker e ROS), porque os scripts
> `tools.sh` e `run.sh` referenciam o diretorio pelo nome `ros_ws`.

## Fluxo de trabalho

Todos os comandos rodam a partir da raiz do repositorio.

1. Construir a imagem:

```bash
bash docker/scripts/build.sh
```

2. Iniciar o container:

```bash
bash docker/scripts/run.sh
```

3. Desenvolver dentro de `ros_ws/src/`. O diretorio `ros_ws/` e montado como volume, entao os
   arquivos ficam no computador e a compilacao acontece dentro do container.

4. Compilar, ja dentro do container:

```bash
colcon build
```

5. Executar um no:

```bash
ros2 run <pacote> <no>
```

## Atividades

Cada atividade nova e desenvolvida em uma branch propria (`ros/lista-N`, `docker/lista-N`) e
integrada a `main` depois de concluida.

| Atividade | Pasta |
|---|---|
| Aula 2 - Docker e ROS: montagem do ambiente | `docker/` |
