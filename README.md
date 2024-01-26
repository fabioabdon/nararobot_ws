# NARAROBOT 

## 1 Hardware
- Arduino Mega;
- IMU MPU6050;
- LIDAR A2;
- Ponte H BTS7960.

## 2 Software
- Linux 20.04;
- ROS Noetic.

## 3 Instalação

### 3.1 Clone os diretórios do NARAROBOT

```
git clone https://github.com/fabioabdon/nararobot_ws.git
```

### 3.2 Instale os repositórios necessários

```
cd lino_install
./install <base> <sensor>
```

### 3.3 Carregando o código para o Arduino

Instale a IDE do arduino através do site oficial, em seguida vá até o diretório "nararobot_ws/src/nararobot/arduino/nara2" e inicie o arquivo "nara2.ino".
Ao iniciar a ide, clique em "arquivos > preferencias" e modifique o local do sketchbook para a pasta "nararobot_ws/src/nararobot/arduino", assim será possível carregar todas as bibliotecas necessárias.

## 4 Criando novo mapa

### 4.1 Gerando o mapa

#### Inicie o driver básico:
```
roslaunch linorobot bringup.launch
```

#### Inicie pacotes de mapeamento:
```
roslaunch linorobot slam.launch
```

#### Inicie a teleoperação:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Navegue pelo ambiente para gerar o mapa. É possível visualizar o progresso pelo RViz.

### 4.2 Salvando o mapa
Quando terminar o mapeamento, salve o mapa. O nome map pode ser substituído por um outro nome de sua preferencia.

```
rosrun map_server map_saver -f ~/nararobot_ws/src/nararobot/maps/map
```

### 4.3 Alterando entre os mapas salvos (somente quando for utilizar um outro mapa salvo)
Navegue até o diretório "src/nararobot/launch" e abra o arquivo "navigate.launch", em seguida modifique a linha número 3 para o nome do mapa a ser utilizado.

## 5 Navegação Autônoma

### 5.1 Iniciando o AMCL

#### Inicie o driver básico:
```
roslaunch linorobot bringup.launch
```

#### Inicie pacotes de navegação:
```
roslaunch linorobot navigate.launch
```

### 5.2 Definir a pose inicial
Antes de iniciar a navegação autônoma, é preciso estimar a posição inicial do robô no mapa, para isto:

1) Clique em "2D Post Estimate";
2) Identifique no mapa a localização aproximada do robô e arraste em direção à direção do robô.

### 5.3 Definindo a posição de destino

Após a localização, clique em "2D Nav Goal".
Identifique a localização desejada no mapa e arraste para a direção que você deseja que seu robô esteja indo assim que atingir seu objetivo.

