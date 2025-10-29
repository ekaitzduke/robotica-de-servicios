# Robótica de servicios

## Comandos para conectar al repositorio
    Conectarse al repositorio:
    git remote add origin https://github.com/ekaitzduke/robotica-de-servicios` 

    Descargarse el proyecto (solo la primera vez):
    git clone https://github.com/ekaitzduke/robotica-de-servicios


## ToDo:
    - MÁQUINA DE ESTADOS
        Estados:
            1) WANDER: El robot deambula  hasta detectar un gesto.
            2) APROACH: El robot se aproxima al cliente que ha levantado la mano (5 dedos extendidos).
            3) RECOGNISE: Reconocer el gesto (saber que gesto se va a hacer)
            4) EXECUTE: Ejecutar la acción

    - RECONOCIMIENTO DE GESTOS
        Clasificación de gestos:
            1) 5 dedos extendidos: Llamar al robot.
            2) Pulgar arriba: Pedido completado.
            3) En base al nº de dedos extendido: seleccionar pedido
            4) Pulgar abajo: Hoja de reclamaciones.
            5) Apuntar con un dedo a la palma contraria: Pedir cuenta.
            6) Wassaaa: Pedir Wifi.