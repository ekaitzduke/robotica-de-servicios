# Robótica de servicios (Gestos y GUI)

## ToDo:

- SOLUCIONAR GESTOS

    1) Hacer detección de pulgar más robusta

    2) Usar parámetros del detector (bajarle confianza) para que funcione en entornos con iluminación no uniforme



- CONEXIÓN CON ROBOT

    1) Averiguar si es conveniente que el robot mantenga propiamente la GUI como un nodo o que un ordenador externo envíe los datos directamente según acciones en esta al robot

    2) En caso de que el robot vaya a contener la GUI:
 
        1) Averiguar si robot tiene pygame, opencv y mediapipe incorporado.
      
        2) En caso negativo, ver si tiene anaconda/miniconda incorporado.
      
        3) Si todo esto falla, seguramente toque llorar en una esquina y olvidarse de que pueda mover la GUI por si solo (o volver a preguntar a profesor)
 
    3) En caso de que se vaya a comunicar los datos al robot (que solo tendrá máquina de estados y algo para enviar datos de la GUI captados externamente a un topic concreto)
 
        1) Averiguar como se pueden comunicar datos entre robot y ordenador (no abrir propiamente un escritorio remoto, sino un mensaje por la conexión o algo parecido)
      
        2) Construir nodo que se encargue de recoger estos datos y enviárselos a los topics de ROS2



- MEJORAS GUI

    1) Incorporar guardado de imágenes en la aplicación (Tiene que guardarse con cierta resolución, seguramente requiera duplicar imagen de lo que usa para mostrar en pantalla)

    2) Añadir un sistema de páginas en la cuadricula de pantallas (Que se vaya rellenando y añadiendo páginas en caso de querer incorporar más imágenes. Usar teclas +/- para pasar páginas?)
 
    3) Añadir pregunta directa en pantalla cuando haya una detección de gesto para confirmar (acknowledge)
 
    4) Añadir un botón para permitir cambiar distribución de imágenes en una cuadrícula (un swap entre 2 imágenes)
