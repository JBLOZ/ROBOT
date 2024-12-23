import math
import numpy as np
import datetime

from fuzzy_expert.variable import FuzzyVariable
from fuzzy_expert.rule import FuzzyRule
from fuzzy_expert.inference import DecompositionalInference

class FuzzySystem:
    """
    Sistema experto difuso para el control de un robot que navega a través de segmentos.
    Utiliza lógica difusa para determinar la velocidad angular basada en el error angular.
    La velocidad lineal se mantiene constante en VMAX, con ajustes en segmentos triangulares.
    """

    # Definición de constantes de clase
    VMAX = 3.0  # Velocidad lineal máxima (m/s)
    VMAX_TRIANGULO = 3  # Velocidad lineal máxima en segmentos triangulares (m/s)
    WMAX = 1.0  # Velocidad angular máxima (rad/s)
    VACC = 1.0  # Aceleración lineal máxima (m/s²)
    WACC = 0.5  # Aceleración angular máxima (rad/s²)
    TOLERACION_FIN_SEGMENTO = 0.5  # Tolerancia para considerar que se alcanzó el final del segmento (m)
    TOLERANCIA_MEDIO = 3  # Tolerancia para considerar que se alcanzó el punto medio del triángulo (m)
    LOGS_TIEMPO_REAL = False # Activar los logs en tiempo real solamente si se desea corregir algun error, 
                            # baja una media de 2 puntos el rendimiento
    MAXIMIZACION_DE_ESTE_EJERCICIO = True 

    def __init__(self):
        """
        Inicializa el sistema experto difuso, definiendo las variables no difususas, las difusas y las reglas.
        También inicializa el sistema experto que usaremos, el DecompositionalInference
        """
        self.objetivoAlcanzado = False  
        self.segmentoObjetivo = None  
        self.velocidad_lineal_previa = 0.0  
        self.velocidad_angular_previa = 0.0  
        self.medioAlcanzado = False # Indica si el robot alcanza el punto medio del segmento triangulo
        self.segmento = 0 # Indica en que segmento se hubica, 0,5 ; 1,5 y 2,5 segmentos lineales -- 1;2;3 triangulos


        # Variables difusas
        self.variables_normales = self.definir_variables_normales()
        self.variables_triangulo = self.definir_variables_triangulo()

        # Reglas difusas
        self.rules = self.definir_reglas()

        # Configuracion del modelo
        self.modelo = DecompositionalInference(
            and_operator="min",
            or_operator="max",
            implication_operator="Rc",
            composition_operator="max-min",
            production_link="max",
            defuzzification_operator="cog",
        )

    
    def setObjetivo(self, segmento):
        """
        Establece un nuevo objetivo para el robot.
        """
        self.objetivoAlcanzado = False
        self.segmentoObjetivo = segmento
        self.medioAlcanzado = False  
        self.segmento += 0.5 # Aumenta cada vez que se genera un objetivo para usar la MAXIMIZACION_DE_ESTE_EJERCICIO si es necesario



    def definir_variables_normales(self):
        """
        Define las variables difusas de entrada y salida para segmentos lineales.
        
        Retorna:
            dict: Variables difusas que incluyen el error angular y la velocidad angular.
        
        Explicación:
            Estas variables difusas están diseñadas para segmentos lineales, donde el objetivo es mantener la estabilidad del robot
            y seguir una trayectoria recta. La entrada "error_angular" mide la desviación del robot respecto al ángulo deseado, y la salida
            "velocidad_angular" se utiliza para ajustar la orientación del robot de manera precisa.
        """
        variables = {
            # Entrada: error angular
            "error_angular": FuzzyVariable(
                universe_range=(-math.pi, math.pi),
                terms={
                    "NegativoGrande": ('trimf', -math.pi, -math.pi, -math.pi/2),
                    "NegativoPequeno": ('trimf', -math.pi, -math.pi/2, 0),
                    "Cero": ('trimf', -math.pi/10, 0, math.pi/10),
                    "PositivoPequeno": ('trimf', 0, math.pi/2, math.pi),
                    "PositivoGrande": ('trimf', math.pi/2, math.pi, math.pi),
                },
            ),
            # Salida: velocidad angular
            "velocidad_angular": FuzzyVariable(
                universe_range=(-self.WMAX, self.WMAX),
                terms={
                    "FuerteIzquierda": ('trimf', -self.WMAX, -self.WMAX, -self.WMAX/2),
                    "Izquierda": ('trimf', -self.WMAX, -self.WMAX/2, 0),
                    "Recto": ('trimf', -self.WMAX/10, 0, self.WMAX/10),
                    "Derecha": ('trimf', 0, self.WMAX/2, self.WMAX),
                    "FuerteDerecha": ('trimf', self.WMAX/2, self.WMAX, self.WMAX),
                },
            ),
        }
        return variables

    def definir_variables_triangulo(self):
        """
        Define las variables difusas de entrada y salida para segmentos triangulares.
        
        Retorna:
            dict: Diccionario con las variables difusas de entrada y salida, incluyendo el error angular y la velocidad angular.
        
        Explicación:
            Esta función ajusta las funciones de pertenencia de las variables difusas para permitir giros más bruscos en los segmentos triangulares. 
            En estos segmentos, el robot necesita reaccionar rápidamente a los cambios de dirección, por lo que los términos de las funciones de pertenencia 
            se adaptan para proporcionar un control más agresivo y preciso en comparación con los segmentos lineales. Esto implica definir un 
            "error angular" con divisiones más refinadas y una "velocidad angular" que permita giros más abruptos, mejorando así la maniobrabilidad del robot.
        """
        variables = {
            # Entrada: error angular
            "error_angular": FuzzyVariable(
                universe_range=(-math.pi, math.pi),
                terms={
                    "NegativoGrande": ('trimf', -math.pi, -math.pi, -math.pi/11),
                    "NegativoPequeno": ('trimf', -math.pi/4, -math.pi/8, 0),
                    "Cero": ('trimf', -math.pi/4 if self.segmento == 1 else -math.pi/5, 0, math.pi/4 if self.segmento == 1 else math.pi/5), # en el segmento triangulo (1) el mantenerse recto obtendrá mas peso en el sistema
                    "PositivoPequeno": ('trimf', 0, math.pi/8, math.pi/4),
                    "PositivoGrande": ('trimf', math.pi/11, math.pi, math.pi),
                },
            ),
            # Salida: velocidad angular
            "velocidad_angular": FuzzyVariable(
                universe_range=(-self.WMAX, self.WMAX),
                terms={
                    "FuerteIzquierda": ('trimf', -self.WMAX, -self.WMAX, -self.WMAX/2),
                    "Izquierda": ('trimf', -self.WMAX, -self.WMAX/2, 0),
                    "Recto": ('trimf', -self.WMAX/4 if self.segmento == 1 else -self.WMAX/5, 0, self.WMAX/4 if self.segmento == 1 else self.WMAX/5), # en el segmento triangulo (1) el mantenerse recto obtendrá mas peso en el sistema
                    "Derecha": ('trimf', 0, self.WMAX/2, self.WMAX),
                    "FuerteDerecha": ('trimf', self.WMAX/2, self.WMAX, self.WMAX),
                },
            ),
        }
        return variables

    def definir_reglas(self):
        """
        Define las reglas difusas para el sistema.
        
        Retorna:
            list: Lista de reglas difusas definidas para el sistema.
        
        Explicación:
            Las reglas difusas son la base del sistema de control, ya que definen cómo se deben traducir las condiciones 
            de entrada (como el error angular) en acciones de salida (como la velocidad angular). En este sistema, 
            cada regla especifica una relación entre el valor del error angular y la respuesta correspondiente de la 
            velocidad angular para corregir la trayectoria del robot. Por ejemplo, si el error angular es "Cero", 
            la velocidad angular debe ser "Recto" para mantener la orientación actual. Estas reglas están diseñadas 
            para minimizar el error y garantizar que el robot siga una trayectoria óptima.
        """
        rules = [
            # Si el error angular es Cero, entonces angular es Recto
            FuzzyRule(
                premise=[
                    ("error_angular", "Cero"),
                ],
                consequence=[
                    ("velocidad_angular", "Recto"),
                ],
            ),
            # Si el error angular es PositivoPequeno, entonces angular es Derecha
            FuzzyRule(
                premise=[
                    ("error_angular", "PositivoPequeno"),
                ],
                consequence=[
                    ("velocidad_angular", "Derecha"),
                ],
            ),
            # Si el error angular es PositivoGrande, entonces angular es FuerteDerecha
            FuzzyRule(
                premise=[
                    ("error_angular", "PositivoGrande"),
                ],
                consequence=[
                    ("velocidad_angular", "FuerteDerecha"),
                ],
            ),
            # Si el error angular es NegativoPequeno, entonces angular es Izquierda
            FuzzyRule(
                premise=[
                    ("error_angular", "NegativoPequeno"),
                ],
                consequence=[
                    ("velocidad_angular", "Izquierda"),
                ],
            ),
            # Si el error angular es NegativoGrande, entonces angular es FuerteIzquierda
            FuzzyRule(
                premise=[
                    ("error_angular", "NegativoGrande"),
                ],
                consequence=[
                    ("velocidad_angular", "FuerteIzquierda"),
                ],
            ),
        ]
        return rules


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
        distancia_punto_final = np.linalg.norm(puntoFin - np.array(poseRobot[0:2]))
        if distancia_punto_final <= FuzzySystem.TOLERACION_FIN_SEGMENTO:
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
        # Aumentar la distancia de anticipación para empezar a girar antes
        distancia_anticipacion = self.VMAX * ((2.2 if (self.segmento == 2 or self.segmento == 2.5 or self.segmento == 1) else 1.4) if self.MAXIMIZACION_DE_ESTE_EJERCICIO else 2) # si esta activado se trataran el segmento 2 y el 2.5 con un target mas extendido
        longitud_segmento = np.linalg.norm(fin - inicio)

        x, y = poseRobot[0], poseRobot[1]

        k_inicial = ((x - inicio[0]) * (fin[0] - inicio[0]) + (y - inicio[1]) * (fin[1] - inicio[1])) / (longitud_segmento ** 2)
        k_inicial = min(max(k_inicial, 0.0), 1.0)

        k_final = k_inicial + (distancia_anticipacion / longitud_segmento)
        k_final = min(max(k_final, 0.0), 1.0)

        target_point = self.get_point_on_segment(k_final, inicio, fin)
        
        return target_point

    def calcularPuntoMedioTrianguloExtendido(self, inicio, fin, medio):
        """
        Calcula el punto medio extendido del triángulo, desplazando el punto medio original
        'x' unidades en la dirección perpendicular al segmento inicio-fin y 'y' unidades en la 
        direccion paralela al segmento inicio-fin
        """
        # Vector director del segmento inicio-fin
        vector_segmento = fin - inicio
        norm_segmento = np.linalg.norm(vector_segmento)

        vector_segmento_unitario = vector_segmento / norm_segmento

        # Vector perpendicular al segmento (rotación de 90 grados)
        vector_perpendicular = np.array([-vector_segmento_unitario[1], vector_segmento_unitario[0]])

        extension_perpendicular = (2.3 if self.segmento == 1 else 1.9 if self.segmento == 2 else 0.8)

        # Extender el punto medio x unidades en la dirección del vector perpendicular
        extension_perpendicular = (2.3 if self.segmento == 1 else 1.9 if self.segmento == 2 else 0.8)
        punto_medio_extendido_perpendicular = medio + extension_perpendicular * vector_perpendicular

        # Vector dirección desde el medio hacia el inicio
        direccion_hacia_inicio = inicio - medio
        norm_direccion = np.linalg.norm(direccion_hacia_inicio)

        direccion_hacia_inicio_unitario = direccion_hacia_inicio / norm_direccion

    
        # Extender el punto medio x unidades en la dirección del inicio
        extension_paralela = (2.4 if self.segmento == 1 else 3 if self.segmento == 2 else 1.5)
        punto_medio_final = punto_medio_extendido_perpendicular +  extension_paralela * direccion_hacia_inicio_unitario

        return punto_medio_final

    def calcularControl(self, W):
        """
        Aplica las restricciones de aceleración y velocidad máxima a las velocidades calculadas ademas
        de usar la velocidad angular previa para progresivanmente ir modificando la velocidad angular.
        """

        # Limitar aceleración angular
        delta_w = W - self.velocidad_angular_previa
        delta_w = max(min(delta_w, self.WACC), -self.WACC)
        velocidad_angular = self.velocidad_angular_previa + delta_w

        # Asegurar que la velocidad angular no exceda los límites
        velocidad_angular = max(min(velocidad_angular, self.WMAX), -self.WMAX)

        # Actualizar velocidades previas
        self.velocidad_angular_previa = velocidad_angular

        return velocidad_angular

    def tomarDecision(self, poseRobot):
        """
        Toma una decisión sobre las velocidades de control basadas en la posición actual del robot y su objetivo.
        
        Parámetros:
            poseRobot: Contiene la posición actual del robot en términos de coordenadas (x, y) y orientación (theta).
        
        Retorna:
            tuple: Contiene la velocidad lineal `V` y la velocidad angular ajustada `velocidad_angular`.
        
        Explicación:
            Este método determina la trayectoria del robot en función de su posición actual y su objetivo.
            Primero, calcula el punto objetivo al cual el robot debe dirigirse, ya sea en un segmento lineal o en un triángulo,
            dependiendo del tipo del segmento objetivo. Luego, se calcula el error angular entre la orientación actual del robot
            y el ángulo necesario para alcanzar el punto objetivo. Utilizando el sistema de inferencia difusa, se determina la 
            velocidad angular para corregir la orientación, teniendo en cuenta las restricciones de aceleración y velocidad máxima.
        """

        fin = np.array(self.segmentoObjetivo.getFin())  
        inicio = np.array(self.segmentoObjetivo.getInicio())  

        # Cálculo de distancia al segmento
        dist = FuzzySystem.straightToPointDistance(inicio, fin, np.array(poseRobot[0:2]))
        
        # Actualizar estado del robot respecto al punto final
        self.actualizarEstado(poseRobot, fin)

        # Implementar lógica para segmentos de tipo 2 (triángulos)
        if self.segmentoObjetivo.getType() == 2 and not self.medioAlcanzado:
            medio = np.array(self.segmentoObjetivo.getMedio())  # Obtener el punto medio del triángulo

            # Calcular el punto medio extendido
            medio_extendido = self.calcularPuntoMedioTrianguloExtendido(inicio, fin, medio)
            dist_a_medio = np.linalg.norm(medio_extendido - np.array(poseRobot[:2]))

            if dist_a_medio > self.TOLERANCIA_MEDIO:
                # Dirigirse al punto medio extendido
                target_point = self.calcularPuntoObjetivo(inicio, medio_extendido, poseRobot)
            else:
                # Cambiar objetivo al punto final
                self.medioAlcanzado = True
                target_point = self.calcularPuntoObjetivo(medio_extendido, fin, poseRobot)
        else:
            # Para otros segmentos o si ya se alcanzó el punto medio, dirigirse al final
            target_point = self.calcularPuntoObjetivo(inicio, fin, poseRobot)

        # Calcular el error angular
        x, y, theta_deg = poseRobot[0], poseRobot[1], poseRobot[2]
        theta = math.radians(theta_deg)
        delta_x = target_point[0] - x
        delta_y = target_point[1] - y
        angulo_a_target = math.atan2(delta_y, delta_x)
        error_angular = angulo_a_target - theta
        error_angular = (error_angular + math.pi) % (2 * math.pi) - math.pi

        # Preparar entradas difusas
        inputs = {
            "error_angular": error_angular,
        }

        # Determinar si estamos en un segmento triangular
        if self.segmentoObjetivo.getType() == 2:
            variables = self.variables_triangulo
            V = self.VMAX_TRIANGULO


        else:
            variables = self.variables_normales
            V = self.VMAX

        # Ejecutar inferencia difusa
        resultado, cf = self.modelo(
            variables=variables,
            rules=self.rules,
            error_angular=inputs["error_angular"],
        )

        # Obtener valor de salida difuso
        W_fuzzy = resultado.get("velocidad_angular", 0)


        # Aplicar restricciones de aceleración y velocidad
        velocidad_angular = self.calcularControl(W_fuzzy)

        if self.LOGS_TIEMPO_REAL:
            self.imprimirPuntuacion(dist, V, velocidad_angular)


        return V, velocidad_angular


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
