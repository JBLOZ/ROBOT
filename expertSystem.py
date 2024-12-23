import math
import numpy as np
import datetime

class ExpertSystem:
    """
    Sistema experto para el control de un robot que navega a través de segmentos.
    Calcula velocidades de control, puntúa la trayectoria y registra logs de desempeño.
    """

    # Definición de constantes de clase
    VMAX = 3.0  # Velocidad lineal máxima (m/s)
    VMAX_TRIANGULO = 2.9
    WMAX = 1.0  # Velocidad angular máxima (rad/s)
    VACC = 1.0  # Aceleración lineal máxima (m/s²)
    WACC = 0.5  # Aceleración angular máxima (rad/s²)
    FACT_ANTICIPACION_GIRO = 1.4  # Parámetro que regula la anticipación al giro del robot
    TOLERACION_FIN_SEGMENTO = 0.5  # Tolerancia para considerar que se alcanzó el final del segmento (m)
    LOGS_TIEMPO_REAL = False  # Activar los logs en tiempo real solamente si se desea corregir algun error, 
                              # baja una media de 2 puntos el rendimiento
    MAXIMIZACION_DE_ESTE_EJERCICIO = True 
    """
    MAXIMIZACION_DE_ESTE_EJERCICIO:
    
    Si esta opcion se encuentra en False se trataran todos los segmentos de la 
    misma forma y todos los segmentos usaran el mismo factor de anticipacion al giro. 
    Puntuacion media de 41.

    Si esta opcion se encuentra en True  se  modificaran los factores de anticipacion para 
    cada segmento de forma en la que den mas puntos.
    Puntuacion media de 44.
    """        

    def __init__(self):
        """
        Inicializa los atributos del sistema experto.
        """
        self.objetivoAlcanzado = False  
        self.segmentoObjetivo = None  
        self.velocidad_lineal_previa = 0.0  # Velocidad lineal previa
        self.velocidad_angular_previa = 0.0  # Velocidad angular previa
        self.medioAlcanzado = False # Indica si el robot alcanza el punto medio del segmento triangulo
        self.segmento = 0 # Indica en que segmento se hubica, 0,5 ; 1,5 y 2,5 segmentos lineales -- 1;2;3 triangulos

    def setObjetivo(self, segmento):
        """
        Establece un nuevo objetivo para el robot.
        """
        self.objetivoAlcanzado = False
        self.segmentoObjetivo = segmento
        self.medioAlcanzado = False # Al volver a settear el objetivo el medio vuelve a False por si el siguinte triangulo tiene que usarlo
        self.segmento += 0.5 # Aumenta cada vez que se genera un objetivo para usar la MAXIMIZACION_DE_ESTE_EJERCICIO si es necesario
        
        if ExpertSystem.MAXIMIZACION_DE_ESTE_EJERCICIO:
            if self.segmento == 0.5:
                ExpertSystem.FACT_ANTICIPACION_GIRO = 1.4
            elif self.segmento == 2.5:
                ExpertSystem.FACT_ANTICIPACION_GIRO = 1.61
                
            elif self.segmento == 1 or self.segmento == 3:
                ExpertSystem.FACT_ANTICIPACION_GIRO = 1.7
            else:
                ExpertSystem.FACT_ANTICIPACION_GIRO = 1.2


    @staticmethod
    def straightToPointDistance(p1, p2, p3):
        """
        Calcula la distancia perpendicular desde un punto a una línea definida por dos puntos.

        Esta función sirve para calcular la distancia del robot con la linea y poder generar 
        los LOGS_TIEMPO_REAL.

        **Nota**: Esta función está copiada del código `P1Launcher`.

        Parámetros:
            p1: Coordenadas del primer punto que define el segmento
            p2: Coordenadas del segundo punto que define el segmento
            p3: Coordenadas del punto (robot) desde el cual se calcula la distancia perpendicular a la línea

        Retorna:
            float: La distancia perpendicular desde el punto `p3` a la línea definida por `p1` y `p2`
        """
        return np.linalg.norm(np.cross(p2 - p1, p1 - p3)) / np.linalg.norm(p2 - p1)

    @staticmethod
    def get_point_on_segment(k, start_point, end_point):
        """
        Calcula un punto específico en un segmento definido por dos puntos: inicio y fin

        Este punto se determina mediante una interpolación lineal basada en el parámetro `k`
        El valor de `k` debe estar en el rango [0, 1], donde:
            - `k = 0` corresponde al `start_point`
            - `k = 1` corresponde al `end_point`
            - Valores intermedios de `k` (por ejemplo, 0.5) devuelven un punto proporcionalmente ubicado entre `start_point` y `end_point`

        Parámetros:
            k (float): Un valor entre 0 y 1 que indica la posición relativa del punto en el segmento
            start_point: Coordenadas del punto de inicio del segmento
            end_point: Coordenadas del punto final del segmento

        Retorna:
            np.array: Las coordenadas del punto calculado en el segmento

        """
        return (1 - k) * np.array(start_point) + k * np.array(end_point)


    def actualizarEstado(self, poseRobot, puntoFin):
        """
        Actualiza el estado del robot basado en la distancia al punto final del segmento.
        """
        # Cálculo de la distancia al punto final del segmento
        distancia_punto_final = np.linalg.norm(puntoFin - np.array(poseRobot[0:2]))

        if distancia_punto_final <= ExpertSystem.TOLERACION_FIN_SEGMENTO:
            self.objetivoAlcanzado = True


    def calcularPuntoObjetivo(self, inicio, fin, poseRobot):
        """
        Calcula el punto objetivo adelantado basado en la posición actual del robot.

        Esta función determina un punto objetivo en un segmento definido por los puntos `inicio` y `fin`.
        El punto objetivo se calcula adelantando una distancia determinada basada en la velocidad actual del robot,
        lo que permite al robot anticipar su trayectoria y mejorar la navegación, especialmente en curvas.

        Parámetros:
            inicio: Coordenadas del punto de inicio del segmento
            fin: Coordenadas del punto final del segmento
            poseRobot: Coordenadas actuales del robot en el espacio

        Retorna:
            np.array: Coordenadas del punto objetivo calculado en el segmento

        Explicación de los Pasos:
            - **Velocidad Promedio y Distancia de Anticipación**:
                La velocidad promedio se calcula como la media entre la velocidad lineal previa y la velocidad máxima. 
                Esta velocidad promedio se multiplica por un factor de anticipación para determinar cuánto adelantarse en el segmento.

            - **Proyección de la Posición Actual**:
                La posición actual del robot se proyecta sobre el segmento para encontrar la posición relativa (`k_inicial`). 
                Esto ayuda a determinar dónde se encuentra el robot en relación con el segmento.

            - **Cálculo del Punto Adelantado**:
                A partir de `k_inicial`, se calcula `k_final` añadiendo la fracción correspondiente a la distancia de anticipación. 
                Esto garantiza que el punto objetivo esté adelantado en el segmento, mejorando la capacidad del robot para seguir la trayectoria de manera suave.

            """
        # Parámetros para anticipación
        velocidad_promedio = (self.velocidad_lineal_previa + self.VMAX) / 2
        distancia_anticipacion = velocidad_promedio * self.FACT_ANTICIPACION_GIRO

        # Cálculo de la distancia total del segmento
        longitud_segmento = np.linalg.norm(fin - inicio)

        # Cálculo de k_closest (proyección de la posición actual sobre el segmento)
        x, y = poseRobot[0], poseRobot[1]
        k_inicial = ((x - inicio[0]) * (fin[0] - inicio[0]) + (y - inicio[1]) * (fin[1] - inicio[1])) / (longitud_segmento ** 2)
        k_inicial = min(max(k_inicial, 0.0), 1.0)  # Asegurar que k_inicial esté entre 0 y 1

        # Cálculo de k_lookahead (posición adelantada en el segmento)
        k_final = k_inicial + (distancia_anticipacion / longitud_segmento)
        k_final = min(max(k_final, 0.0), 1.0)  # Asegurar que k_final esté entre 0 y 1

        # Obtener el punto objetivo adelantado
        target_point = self.get_point_on_segment(k_final, inicio, fin)

        

        return target_point



    def calcularControl(self, target_point, poseRobot):
        """
        Calcula las velocidades lineal y angular necesarias para dirigir el robot hacia el punto objetivo 
        basandose en las velocidades anteriores, la aceleracion lineal y angular del robot y (muy importante)
        la desaceleracion lineal y angular

        **Descripción Detallada**:

        1. **Extracción de la Pose Actual del Robot**:
            - `x`: Coordenada X de la posición actual del robot
            - `y`: Coordenada Y de la posición actual del robot
            - `theta_deg`: Orientación actual del robot en grados

        2. **Conversión de Ángulo a Radianes**:
            - Convierte `theta_deg` de grados a radianes para facilitar los cálculos trigonométricos

        3. **Cálculo de las Diferencias de Posición**:
            - `delta_x` (float): Diferencia en la coordenada X entre el punto objetivo y la posición actual
            - `delta_y` (float): Diferencia en la coordenada Y entre el punto objetivo y la posición actual

        4. **Cálculo del Ángulo hacia el Punto Objetivo**:
            - `angulo_a_target` (float): Ángulo en radianes desde la posición actual del robot hacia el punto objetivo

        5. **Cálculo del Error Angular**:
            - `error_angular` (float): Diferencia entre el ángulo deseado (`angulo_a_target`) y la orientación actual (`theta`)
            - **Normalización del Ángulo**: Ajusta `error_angular` para que esté en el rango [-π, π]

        6. **Aplicación de la Ganancia de Control**:
            - `k_w` (float): Ganancia proporcional para la velocidad angular, determina la sensibilidad del controlador al error angular

        7. **Cálculo de las Velocidades Deseadas**:
            - `velocidad_angular_deseada` (float): Velocidad angular calculada usando una función hiperbólica para suavizar la respuesta
            - `velocidad_lineal_deseada` (float): Velocidad lineal deseada, mantenida constante en `VMAX`; así el robot mantendrá la máxima puntuacion posible al solamente tener que girar

        8. **Cálculo de las Variaciones de Velocidad**:
            - `delta_v` (float): Cambio en la velocidad lineal, limitado por `VACC`.
            - `delta_w` (float): Cambio en la velocidad angular, limitado por `WACC`.

        9. **Actualización de las Velocidades**:
            - `velocidad_lineal` (float): Nueva velocidad lineal del robot, ajustada con `delta_v` y limitada por `VMAX`.
            - `velocidad_angular` (float): Nueva velocidad angular del robot, ajustada con `delta_w`.

        10. **Actualización de las Velocidades Previas**:
            - Guarda las velocidades actuales para usarlas en el siguiente ciclo de control.

        **Parámetros**:
            target_point: Coordenadas del punto objetivo hacia el cual el robot debe dirigirse

            poseRobot: Posición y orientación del robot

        **Retorna**:
            tupla: Velocidades lineal y angular calculadas
        """
        x, y, theta_deg = poseRobot[0], poseRobot[1], poseRobot[2]
        theta = math.radians(theta_deg)

        # Diferencias en posición
        delta_x = target_point[0] - x
        delta_y = target_point[1] - y

        # Ángulo hacia el punto objetivo
        angulo_a_target = math.atan2(delta_y, delta_x)
        error_angular = angulo_a_target - theta
        error_angular = (error_angular + math.pi) % (2 * math.pi) - math.pi  # Normalización del ángulo

        # Ganancias de control
        k_w = 2.0  # Ganancia proporcional para la velocidad angular

        # Velocidades deseadas
        velocidad_angular_deseada = self.WMAX * math.tanh(k_w * error_angular)
        if self.segmentoObjetivo.getType() == 2:
            velocidad_lineal_deseada = self.VMAX_TRIANGULO
        else:
            velocidad_lineal_deseada = self.VMAX  # Mantener la velocidad máxima siempre

        # Limitación de velocidades máximas para angular
        velocidad_angular_deseada = max(min(velocidad_angular_deseada, self.WMAX), -self.WMAX)

        # Cálculo de las variaciones de velocidad
        delta_v = velocidad_lineal_deseada - self.velocidad_lineal_previa
        delta_v = max(min(delta_v, self.VACC), -self.VACC)

        delta_w = velocidad_angular_deseada - self.velocidad_angular_previa
        delta_w = max(min(delta_w, self.WACC), -self.WACC)

        # Actualización de velocidades
        velocidad_lineal = self.velocidad_lineal_previa + delta_v
        velocidad_angular = self.velocidad_angular_previa + delta_w

        # Almacenamiento de velocidades anteriores para el siguiente ciclo
        self.velocidad_lineal_previa = velocidad_lineal
        self.velocidad_angular_previa = velocidad_angular

        return velocidad_lineal, velocidad_angular

    def tomarDecision(self, poseRobot):
        """
        En base a la posicion del robot y su objetivo realiza una busqueda del target point que
        se usara para calcular las velocidades y si es necesario llamar a generar los logs en tiempo
        real.
        """
        if self.segmentoObjetivo.getType() == 2:
            medio = self.segmentoObjetivo.getMedio()


        fin = np.array(self.segmentoObjetivo.getFin())  # Punto final del segmento

        inicio = np.array(self.segmentoObjetivo.getInicio())  # Punto inicial del segmento

        dist = self.straightToPointDistance(inicio, fin, np.array(poseRobot[0:2]))

        # Actualización del estado del robot basado en la distancia al punto final
        if self.segmentoObjetivo is None:
            return (0, 0)
        

        self.actualizarEstado(poseRobot, fin)

        
        if self.segmentoObjetivo.getType() == 2 and not self.medioAlcanzado:
            dist_a_medio = np.linalg.norm(medio - np.array(poseRobot[:2]))

            if dist_a_medio > 1 and (ExpertSystem.MAXIMIZACION_DE_ESTE_EJERCICIO or not ExpertSystem.MAXIMIZACION_DE_ESTE_EJERCICIO):
                target_point = self.calcularPuntoObjetivo(inicio, medio, poseRobot)

            else:
                self.medioAlcanzado = True
                target_point = self.calcularPuntoObjetivo(inicio, fin, poseRobot)
        else:
            target_point = self.calcularPuntoObjetivo(inicio, fin, poseRobot)


        velocidad_lineal, velocidad_angular = self.calcularControl(target_point, poseRobot)

        if self.LOGS_TIEMPO_REAL:
            self.imprimirPuntuacion(dist, velocidad_lineal, velocidad_angular)

        return velocidad_lineal, velocidad_angular


    def imprimirPuntuacion(self, dist, velocidad_lineal, velocidad_angular):
        """
        Imprime la distancia a la que esta el robot del segmento, anteriormente tambien imprimia la puntuacion a tiempo real
        """
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(
            f"[{timestamp}] Distancia al segmento: {dist:.4f} m, "
            f"Velocidad Lineal: {velocidad_lineal:.4f} m/s, "
            f"Velocidad Angular: {velocidad_angular:.4f} rad/s, "
        )

    def esObjetivoAlcanzado(self):
        """
        Devuelve si se ha alcanzado el objetivo.
        """
        return self.objetivoAlcanzado

    def hayParteOptativa(self):
        """
        Indica si hay una parte optativa implementada.
        """
        return True
