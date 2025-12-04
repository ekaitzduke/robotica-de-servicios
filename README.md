# Robótica de servicios (Gestos y GUI)

Rama dedicada a la detección de gestos y GUI del turtlebot

## Utilidades

Los códigos se han desarrollado con la intención de que un contenedor docker (Lanzado en ubuntu 24.04 LTS nativo) pueda mantener la aplicación. Se aportan comandos interesantes para lanzar y gestionar este entorno:
<br/><br/>
### Antes de entrar al entorno

Permite al contenedor conectarse al servidor anfritión y poder lanzar GUIs:
```cmd
xhost +
```

Descarga los ficheros necesarios y preparalos (No importa el lugar):
```cmd
git clone https://github.com/ekaitzduke/robotica-de-servicios -b app-gesture && mv robotica-de-servicios serv && mv serv/Dockerfile ./Dockerfile
```

Construye la imagen docker (Se espera tener el código fuente accesible en la misma carpeta donde se construya, es decir, lanzar el comando después del anterior):<br/>
(**AVISO**: La imagen ocupará aproximadamente un poco más de 9 GB al terminar de construirse)
```cmd
docker build -t ros2_kobuki .
```

Lanzar el contenedor (Montará un volumen en la carpeta de Descargas, teniendo completa accesibilidad de los archivos en esta)<br/>
(Se espera que los drivers de nvidia estén correctamente configurados en su dispositivo):
```cmd
docker run --shm-size=1g --privileged --ulimit memlock=-1 --ulimit stack=67108864 --rm -it --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --name ros2_kobuki_container --cpuset-cpus=0-3 -v /home/alumno/Descargas/:/vgd -v /run/udev:/run/udev --gpus all -e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} -e NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-all} ros2_kobuki
```

Para acceder al entorno con otras terminales (**RECORDATORIO**: Al cerrar la terminal original todas estas se cerrarán también):
```cmd
docker exec -it ros2_kobuki_container /bin/bash
```

Al cerrar el contenedor (**exit**) se perderá cualquier cambio realizado en él que no sea en el volumen montado. Puede borrar la imagen residual usando (**AVISO**: No se borrarán los datos de la caché del propio docker, al menos no en ordenadores del laboratorio):
```cmd
docker image rmi ros2_kobuki:latest && docker image prune -f
```
**NOTE: Falta temas de conexión y comunicación**
<br/><br/>
### En el entorno
La imagen está preparada para cargar todos los installs para cada terminal que se lance con lo definido en bashrc (Y como la mayoría de la implementación está en python, cualquier cambio en estos no requiere recompilar y hacer otro source). Así que para empezar solo es necesario moverse al repositorio de ros donde están los paquetes (**ros2_Serv_ws**).<br/>
<br/>
Para lanzar la gui, carga primero el entorno virtual python preparado con pygame y mediapipe. 
```cmd
cd ros2_Serv_ws && source ./mp_env/bin/activate
```
Luego lanza el nodo directamente con python (ros2 no funciona ya que no reconoce los paquetes del entorno virtual):<br/>
(Recuerda que al cerrar el nodo, puedes desactivar el entorno virtual con el comando **deactivate**)
```cmd
python src/serv_proy/serv_proy/rosGUI.py
```
<br/><br/>
En otro terminal puedes lanzar el nodo de control con la máquina de estados de forma aislada:
```cmd
ros2 run serv_proy turte
```
Puedes probar a lanzar manualmente una acción de control con el siguiente comando (ejemplo con acción 6, pedir WIFI):
```cmd
ros2 topic pub --once /comando_gesto std_msgs/Int8 '{data: 6}'
```
**NOTE: Falta hacer que la simulación o el launch funcione**
<br/><br/>
## Problemas

Algunos ordenadores del laboratorio tienen problemas con el mediapipe corriendo en el contenedor (en entorno virtual nativo conda van bien), mostrando el siguiente mensaje **al cargar recognizer o hands** mediante opciones: **MESA: error: ZINK: vkCreateInstance failed (VK_ERROR_INCOMPATIBLE_DRIVER)** [Programa no se puede cerrar, necesita **crtl+z**]. <br/>
<br/>
Por el momento no se ha encontrado solución (algunos ordenadores van bien como el de la entrada al laboratorio en mesa del profesor). El modo debug (**-d**) está preparado para deshabilitar estas funcionalidades, lo cual deja al programa con la interfaz gráfica y poco más.

## ToDo:

- SOLUCIONAR GESTOS

    1) Hacer detección de pulgar más robusta

    2) Usar parámetros del detector (bajarle confianza) para que funcione en entornos con iluminación no uniforme



- CONEXIÓN CON ROBOT

    1) Probar GUI como un nodo de ros2 (ordenadores del laboratorio y aula de libre acceso tiene problemas con sus drivers. Por ahora se ha comprobado que solo funciona el de al lado del profesor)
 
    2) Arrancar robot y observar que comparte los topics con un ordenador externo
 
    3) Comprobar que se recibe la información de la cámara y se envían las órdenes de control correctamente 



- MEJORAS GUI

    1) Incorporar guardado de imágenes en la aplicación (Tiene que guardarse con cierta resolución, seguramente requiera duplicar imagen de lo que usa para mostrar en pantalla)

    2) Añadir un sistema de páginas en la cuadricula de pantallas (Que se vaya rellenando y añadiendo páginas en caso de querer incorporar más imágenes. Usar teclas +/- para pasar páginas?)
 
    3) Añadir pregunta directa en pantalla cuando haya una detección de gesto para confirmar (acknowledge)
 
    4) Añadir un botón para permitir cambiar distribución de imágenes en una cuadrícula (un swap entre 2 imágenes)
