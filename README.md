# Implantación de Estrategias de Control en Cirugía Laparoscópica para Interacción Adaptativa con Tejidos Blandos

[![ROS](https://img.shields.io/badge/ROS-Melodic-green.svg)](https://www.ros.org/)
[![C++](https://img.shields.io/badge/C++-11%20/%2014-blue.svg)](https://isocpp.org/)
[![UMA](https://img.shields.io/badge/Universidad_de_Málaga-UMA-blue.svg)](https://www.uma.es/)

Repositorio asociado al Trabajo Fin de Grado:

**Implantación de Estrategias de Control en Cirugía Laparoscópica para Interacción Adaptativa con Tejidos Blandos**

- **Autora:** María Santana Feliú  
- **Tutor:** Antonio Jesús Reina Terol  
- **Cotutor:** Álvaro Galán Cuenca  
- **Departamento:** Ingeniería de Sistemas y Automática  
- **Universidad:** Universidad de Málaga  
- **Convocatoria:** Junio 2026  

---

## 1. Descripción general

Este repositorio contiene el código desarrollado para implementar y validar una estrategia de estimación de fuerzas en una plataforma laparoscópica robotizada.

El problema de partida es que, durante una maniobra laparoscópica, la fuerza medida por el robot no corresponde únicamente a la interacción con el tejido interno. Esa fuerza total puede incluir distintas contribuciones, principalmente:

- fuerza asociada al tejido blando;
- contribución abdominal asociada al fulcro, la pared abdominal y el punto de entrada;
- perturbaciones no modeladas, como ruido o pequeños errores de calibración.

El objetivo del trabajo es utilizar modelos dinámicos identificados offline y trasladarlos a una implementación online integrada en ROS. El estimador calcula la fuerza de tejido a partir de la deformación efectiva y obtiene la contribución abdominal a partir de la fuerza total medida y la fuerza estimada de tejido.

> [!NOTE]
> El contenido de este repositorio corresponde a la parte software utilizada específicamente en este TFG.  
> No representa el entorno completo de desarrollo del laboratorio, sino los paquetes y archivos necesarios para reproducir la adquisición, el registro de ensayos y la estimación online descritos en la memoria.

---

## 2. Estructura del paquete

El paquete principal utilizado en este trabajo es `uma_fp_control`.

```text
uma_fp_control/
├── config/         # Archivos de configuración y matrices de calibración
├── launch/         # Archivos de lanzamiento ROS
├── src/            # Nodos principales y coordinación de ejecución
├── services/       # Servicios funcionales del sistema
├── dependencies/   # Módulos auxiliares y dependencias internas
└── mocks/          # Herramientas de prueba o emulación
```
---

## 3. Compilación

Desde el workspace de Catkin:

```bash
cd ~/catkin_ws
catkin_make --only-pkg-with-deps uma_fp_control
source devel/setup.bash
```

> [!NOTE]
> Si se modifican archivos de código fuente en C++, es necesario recompilar el paquete.  
> Los cambios en archivos `.launch` o archivos de configuración no requieren recompilación.

---

## 4. Comprobación de red

Antes de lanzar los nodos principales, se debe comprobar la conectividad con manipuladores y sensores.

### Manipuladores

```bash
ping 192.168.1.20   # Manipulador principal: auto
ping 192.168.1.50   # Manipulador de soporte: darel
```

### Sensores de fuerza/torque

```bash
ping 192.168.1.1    # Sensor del robot / fuerza total
ping 192.168.1.13   # Sensor de tejido
```

Estas comprobaciones permiten verificar que los dispositivos físicos están accesibles desde la red antes de iniciar la adquisición de datos o el estimador online.

---

## 5. Lanzamiento del entorno de trabajo

Antes de ejecutar la calibración, la teleoperación o el estimador online, debe inicializarse el entorno robótico de trabajo.

### 5.1. Inicialización de robots y planificación

```bash
roslaunch uma_ur_launch two_robot.launch
roslaunch ur3e_moveit_config move_group.launch
roslaunch uma_fp_control tfg.launch
```

Estos lanzamientos inicializan el entorno robótico, la planificación cinemática y los nodos base necesarios para la ejecución de los ensayos.

### 5.2. Publicación de datos RTDE

Para publicar los datos procedentes de la interfaz RTDE:

```bash
cd ~/catkin_ws/ur_rtde_publisher
python ur_rtde_publisher.py
```

Para comprobar la publicación:

```bash
rostopic echo /ur3e/rtde/force
```

---

## 6. Calibración espacial

> [!IMPORTANT]
> La calibración espacial no forma parte de la rutina diaria de ejecución.  
> Debe repetirse únicamente si se ha modificado la disposición física de los manipuladores o de los elementos de la plataforma.

> [!NOTE]
> Antes de lanzar la calibración, debe estar inicializado el entorno robótico descrito en el apartado anterior.

El proceso se lanza mediante:

```bash
roslaunch uma_fp_control calibration.launch
```

Este procedimiento obtiene las matrices de transformación del efector respecto a la base correspondiente. Estas matrices no se copian directamente como configuración final del sistema, sino que posteriormente se procesan en MATLAB para calcular las transformaciones necesarias entre los distintos sistemas de referencia.

A partir de ese procesamiento se actualizan los archivos de configuración correspondientes en `config/`.

---

## 7. Teleoperación de la herramienta

La teleoperación de la herramienta durante los ensayos se realiza mediante:

```bash
rosrun uma_fp_control teleopTFG
```

Este nodo lee comandos desde el teclado y publica incrementos de movimiento en el topic:

```text
/haptic_topic
```

Cada tecla genera un desplazamiento incremental de la herramienta en los ejes `X`, `Y` o `Z`, expresado como un mensaje `geometry_msgs/Point`.

### Movimientos básicos

```text
1 → X + 5 mm
4 → X - 5 mm

2 → Y + 5 mm
5 → Y - 5 mm

3 → Z + 5 mm
6 → Z - 5 mm
```

### Trayectorias predefinidas

> [!NOTE]
> Las trayectorias predefinidas están parametrizadas en el propio nodo `teleopTFG`.  
> Para modificar amplitudes, duraciones o pasos de movimiento, debe editarse el código fuente correspondiente (`/mocks/teleoperaciontfgKeyboard.cpp`) y recompilar el paquete.

Además de los movimientos básicos, el nodo incluye teclas para ejecutar trayectorias más complejas utilizadas durante los ensayos, como movimientos diagonales, saltos en `X`, una trayectoria tipo chirp y una trayectoria circular en el plano `XY`.

```text
7, 8, 9, o, p → movimientos diagonales
b, v          → saltos en X
c             → trayectoria tipo chirp en X
d             → trayectoria circular en XY
x             → salir de la teleoperación
```

> [!WARNING]
> Antes de ejecutar movimientos con `teleopTFG`, debe comprobarse que la plataforma está correctamente inicializada, que no existen obstáculos en la trayectoria y que los sensores han sido tarados cuando corresponda.

---

## 8. Registro de ensayos para identificación offline

El registro de ensayos para la identificación de modelos se realiza mediante:

```bash
roslaunch uma_fp_control rosbagExperimentos.launch name:=test_experimento
```

El argumento `name` es genérico y debe sustituirse por un nombre representativo del ensayo.

Por defecto, los archivos se guardan en:

```text
/home/labrob/experimentos/
```

> [!NOTE]
> Este registro se utiliza para la fase de identificación offline de modelos.  
> No forma parte del estimador online, sino de la fase previa de obtención de datos experimentales.

### Señales registradas

El archivo `rosbagExperimentos.launch` almacena las señales necesarias para caracterizar experimentalmente la interacción de la herramienta con el abdomen simulado y el tejido blando.

De forma resumida, registra los siguientes bloques de información:

- transformaciones y estado general del sistema;
- posición objetivo y posición del fulcro;
- pose, velocidad y fuerza medida en el manipulador principal;
- señales asociadas al sistema de abdomen simulado;
- fuerza real de tejido;
- datos crudos de fuerza procedentes de la interfaz RTDE.

Las señales más relevantes para la identificación de modelos son:

```text
/auto/pose_topic
/auto/velocity_topic
/auto/robotbase_force
/auto/tfg/tissue_force
/darel/abdomen_force_topic
/ur3e/rtde/force
```

Donde:

- `/auto/pose_topic`: pose del TCP, utilizada para calcular la deformación aplicada.
- `/auto/velocity_topic`: velocidad de la herramienta.
- `/auto/robotbase_force`: fuerza del robot expresada respecto a la base.
- `/auto/tfg/tissue_force`: fuerza real de tejido medida por el sensor correspondiente.
- `/darel/abdomen_force_topic`: fuerza real asociada al abdomen simulado.
- `/ur3e/rtde/force`: datos crudos de fuerza procedentes de la interfaz RTDE.

> [!TIP]
> El archivo `.launch` contiene el listado completo de topics registrados.  
> En esta sección se destacan únicamente las señales más relevantes para la identificación offline.

---

## 9. Estimador online

El estimador online se lanza mediante:

```bash
roslaunch uma_fp_control force_estimator.launch
```

El nodo parte de los modelos discretos de tejido identificados previamente. Durante una fase inicial de calibración, actualiza sus parámetros mediante RLS utilizando la referencia experimental disponible del tejido. Después, los parámetros pueden congelarse para operar con el modelo aprendido.

### Taraje de señales

Antes de iniciar la interacción, se debe realizar el taraje de las fuerzas:

```bash
rosservice call /auto/tare_forces
```

> [!WARNING]
> El robot debe estar en reposo y sin contacto externo durante el taraje para evitar introducir offsets en las señales.

### Congelación de parámetros

Una vez finalizada la fase de calibración adaptativa, se congelan los parámetros mediante:

```bash
rosservice call /auto/freeze_theta
```

A partir de ese momento, el estimador deja de actualizar los parámetros y utiliza el modelo aprendido para estimar la fuerza de tejido durante la maniobra.

### Actualización de modelos discretos

El estimador online utiliza como punto de partida los modelos discretos de tejido obtenidos en la fase de identificación offline.

> [!IMPORTANT]
> Si se identifica un nuevo modelo de tejido, o se modifican los coeficientes del modelo existente, es necesario actualizar esos parámetros en el código del estimador antes de ejecutar nuevos ensayos online.

En concreto, deben revisarse los coeficientes discretos utilizados por el nodo `force_estimator` para los modelos de tejido en los ejes `X/Y` y `Z`.

> [!NOTE]
> Si únicamente cambian los valores de los coeficientes, se actualizan dichos parámetros.  
> Si cambia la estructura del modelo, por ejemplo el orden del sistema o la forma de la ecuación discreta, también debe modificarse la lógica de cálculo correspondiente.


---

## 10. Registro de ensayos online

El registro de ensayos con el estimador online se realiza mediante:

```bash
roslaunch uma_fp_control rosbag_online.launch name:=test_online_piel1
```

Por defecto, los archivos se guardan en:

```text
/home/labrob/experimentos/online/
```

> [!IMPORTANT]
> Este registro corresponde a los ensayos con el estimador online activo.  
> No debe confundirse con `rosbagExperimentos.launch`, que se utiliza para la identificación offline de modelos.

Este registro almacena tanto las señales de entrada del sistema como las señales generadas por el nodo `force_estimator`. Su finalidad es comparar la fuerza real medida experimentalmente con la estimación online y con la respuesta del modelo offline.

### Señales registradas

De forma resumida, `rosbag_online.launch` registra los siguientes bloques de información:

- transformaciones, estado articular y mensajes del sistema;
- posición objetivo y posición del fulcro;
- señales de pose, velocidad y fuerza del manipulador principal;
- señales del abdomen simulado;
- fuerza total tarada;
- referencias reales de tejido y abdomen;
- estimaciones online de tejido y abdomen;
- estimaciones offline de tejido y abdomen;
- variables auxiliares como deformación y error de RCM.

Las señales más relevantes para evaluar el estimador son:

```text
/auto/robot_force_tared
/auto/tissue_force_real
/auto/abdomen_force_real

/auto/tissue_force_estimated
/auto/abdomen_force_estimated

/auto/tissue_force_offline
/auto/abdomen_force_offline

/auto/rcm_error
/auto/deformacion
```

Donde:

- `/auto/robot_force_tared`: fuerza total medida tras el taraje.
- `/auto/tissue_force_real`: referencia experimental de fuerza de tejido.
- `/auto/abdomen_force_real`: referencia experimental de fuerza abdominal.
- `/auto/tissue_force_estimated`: fuerza de tejido estimada online.
- `/auto/abdomen_force_estimated`: fuerza abdominal estimada online.
- `/auto/tissue_force_offline`: fuerza de tejido obtenida mediante el modelo offline.
- `/auto/abdomen_force_offline`: contribución abdominal obtenida a partir del modelo offline.
- `/auto/rcm_error`: error asociado a la estimación del fulcro o RCM.
- `/auto/deformacion`: deformación calculada durante la maniobra.

> [!TIP]
> Para una revisión completa de todos los topics registrados, debe consultarse directamente el archivo `rosbag_online.launch`.

---

## 11. Visualización de señales

Para visualizar las señales durante la ejecución se puede utilizar `rqt_plot` o `rqt_gui`.

Ejemplo:

```bash
rosrun rqt_gui
```

Señales recomendadas para comparar la estimación de tejido:

```text
/auto/tissue_force_real
/auto/tissue_force_offline
/auto/tissue_force_estimated
```

Señales recomendadas para comparar la contribución abdominal:

```text
/auto/abdomen_force_real
/auto/abdomen_force_offline
/auto/abdomen_force_estimated
```

> [!NOTE]
> La comparación entre señal real, modelo offline y estimación online permite evaluar si la adaptación inicial reduce el error respecto al modelo identificado previamente.

---

## 12. Exportación y análisis de datos

Para extraer un topic de un archivo `.bag` a formato CSV:

```bash
rostopic echo -b nombre_ensayo.bag -p /topic > datos_ensayo.csv
```

Ejemplo:

```bash
rostopic echo -b test_online_piel1.bag -p /auto/tissue_force_estimated > tissue_force_estimated.csv
```

Estos archivos pueden analizarse posteriormente en MATLAB o Python.

> [!TIP]
> La exportación a CSV es útil para realizar análisis posteriores, calcular errores, comparar señales o generar gráficas fuera del entorno de ROS.

### Análisis en entorno ROS 2 con PlotJuggler

Si se desea analizar posteriormente los ensayos en un entorno basado en ROS 2, los archivos `.bag` pueden convertirse antes de abrirlos en PlotJuggler.

Ejemplo de conversión de varios archivos `.bag`:

```bash
cd ~/Experimentos
for file in *.bag; do rosbag_convert "$file" --dst "${file%.bag}.ros2"; done
```

Una vez convertidos, puede abrirse PlotJuggler en ROS 2 mediante:

```bash
ros2 run plotjuggler plotjuggler
```

> [!NOTE]
> Este paso no es necesario para ejecutar el estimador online.  
> Se utiliza únicamente para inspeccionar o analizar los datos posteriormente en un entorno ROS 2.
---

## 13. Flujo resumido de uso

### Identificación offline

```text
1. Comprobar red de robots y sensores
2. Lanzar entorno robótico
3. Ejecutar teleoperación con teleopTFG
4. Registrar ensayos con rosbagExperimentos.launch
5. Analizar datos en MATLAB
6. Ajustar y comparar modelos
7. Seleccionar modelos de tejido
8. Discretizar los modelos seleccionados
```

> [!NOTE]
> Este flujo corresponde a la fase previa de identificación de modelos.  
> Su objetivo es obtener modelos dinámicos a partir de datos experimentales registrados en la plataforma.

### Estimación online

```text
1. Comprobar red de robots y sensores
2. Lanzar entorno robótico
3. Lanzar force_estimator.launch
4. Realizar taraje con /auto/tare_forces
5. Ejecutar fase de calibración adaptativa
6. Congelar parámetros con /auto/freeze_theta
7. Ejecutar maniobra de operación
8. Registrar datos con rosbag_online.launch
9. Comparar fuerza real, offline y online
```

> [!IMPORTANT]
> En la fase online, el sistema no identifica un modelo desde cero.  
> Parte de los modelos discretos obtenidos offline y adapta inicialmente sus parámetros mediante RLS.

---

## 14. Notas finales

- La identificación de modelos se realiza offline a partir de los datos registrados experimentalmente.
- La implementación online utiliza los modelos discretos de tejido como punto de partida.
- El abdomen queda caracterizado en la fase offline, pero no se implementa como modelo adaptativo independiente dentro del estimador.
- Si se identifican nuevos modelos discretos de tejido, los coeficientes utilizados por el estimador deben actualizarse antes de realizar nuevos ensayos online.
- La contribución abdominal se obtiene a partir de la fuerza total medida y de la fuerza de tejido estimada.
- Los ensayos online permiten comparar referencia experimental, modelo offline y estimación online.

> [!NOTE]
> Este repositorio documenta la parte software utilizada para la adquisición, estimación y validación experimental del sistema desarrollado en el TFG.
