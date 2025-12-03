# Robótica de servicios (Gestos y GUI)

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
