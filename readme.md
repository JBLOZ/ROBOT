# Práctica 1: Sistemas Expertos

## Descripción

Esta práctica tiene como objetivo implementar un sistema experto para controlar un robot móvil en un plano horizontal. El robot debe recorrer un segmento de manera rápida y precisa utilizando dos modelos diferentes:

1. **Modelo convencional:** Basado en reglas lógicas simples.
2. **Modelo fuzzy:** Basado en lógica difusa para mejorar la precisión y adaptabilidad.

El robot debe cumplir con las siguientes tareas:

- Moverse desde un punto aleatorio del mapa hasta un punto final.
- Seguir segmentos lineales alineándose lo más cerca posible de la línea.
- Rodear áreas triangulares para maximizar la puntuación.

## Índice

1. [Objetivo de la Práctica](#objetivo-de-la-práctica)
2. [Consideraciones](#consideraciones)
3. [Lógica Común](#lógica-común)
4. [Cálculo de la Aceleración Angular](#cálculo-de-la-aceleración-angular)
   - [Cálculo del Error Angular](#cálculo-del-error-angular)
   - [Cálculo W en el Modelo Experto Simple](#cálculo-w-en-el-modelo-experto-simple)
   - [Cálculo W en el Modelo Experto Fuzzy](#cálculo-w-en-el-modelo-experto-fuzzy)
5. [Resultados y Conclusiones](#resultados-y-conclusiones)

## Objetivo de la Práctica

El objetivo fue crear un sistema experto para controlar un robot móvil en un plano horizontal, haciendo que recorra un segmento de manera rápida y precisa, utilizando funciones para regular su velocidad lineal y angular. Se implementaron dos versiones: una con reglas normales y otra con lógica difusa para mejorar la precisión y la capacidad de respuesta del robot.

El robot debía moverse desde un punto aleatorio del mapa hasta un punto final. Si el robot se encontraba en un segmento lineal, debía seguir la línea lo más cerca posible. Si estaba en un segmento de tipo triángulo, debía rodear el área triangular para sumar más puntos.

## Consideraciones

Para realizar la práctica se tuvieron en cuenta varias consideraciones comunes para ambas versiones, además de consideraciones específicas para cada una:

- **Alineación inicial:** Al inicio de cada objetivo lineal, el robot debía dirigirse al punto más cercano a la línea para alinearse rápidamente con el segmento.
- **Velocidad constante:** El robot siempre mantendría la velocidad lineal máxima, y el programa se encargaría de ajustar solo la velocidad angular. En los segmentos triangulares, se redujo la velocidad máxima en 0.1 unidades en la versión normal para darle tiempo suficiente al robot para girar y llegar al punto final del triángulo. En la versión fuzzy, se realizaron ajustes adicionales para maximizar la puntuación.
- **Conexión precisa entre segmentos:** Para conseguir una conexión precisa entre los segmentos triangulares y los lineales, se implementaron tres técnicas:
  1. Anticipación del Giro personalizada (aplicada a ambas versiones, explicada más adelante).
  2. Cambios manuales y precisos en las entradas y salidas del sistema fuzzy para ajustar la velocidad angular y asegurar que el robot girara más rápido o mantuviera la dirección recta con distintos niveles de error angular.
  3. Uso de un target intermedio personalizado para cada triángulo, facilitando una transición recta hacia cada segmento lineal.

## Lógica Común

Para que el robot se alineara y siguiera los segmentos lineales, se implementó la siguiente lógica en ambas versiones:

- **Cálculo del Target Point:** Se creó una función que calcula un punto en la recta, basado en un parámetro k entre 0 y 1. Esta función devuelve un punto proporcionalmente ubicado entre el punto de inicio y el punto final del segmento. Esto facilita la integración de la lógica para los puntos objetivo relativos que usa el código.

- **Seguimiento del Target Point:** En cada iteración, el robot realiza una búsqueda del target point deseado. Si el robot está lejos de la línea, el target point será el punto más cercano a la línea hasta que se acerque. Una vez cerca de la línea, el valor de k se incrementa gradualmente, guiando al robot a lo largo de la línea. Cuando el robot está alineado, sigue un punto objetivo que se mueve dentro del segmento para que siga la línea justo por el medio.

- **Anticipación del Giro:** Implementé un FACT_ANTICIPACION_GIRO, un factor que ajusta el valor de k para que el robot anticipe los giros, aumentando la distancia del target point. Esto asegura que el giro no ocurra demasiado tarde, evitando un deslizamiento causado por la desaceleración angular.

- **Segmentos Triangulares:** Para los segmentos de tipo triángulo, usé una lógica diferente. Utilicé las mismas funciones de seguimiento de línea, pero agregué un punto objetivo fijo intermedio entre el inicio y el fin. En la versión normal, este punto objetivo es el punto medio del triángulo, mientras que en la versión fuzzy se desplaza 'x' unidades en la dirección perpendicular al segmento inicio-fin y 'y' unidades en la dirección paralela al segmento inicio-fin hacia el punto inicial.

- **Tolerancias con el target:** Para los segmentos lineales, la tolerancia del target final siempre se mantuvo en 0.5, tal como indicaba el enunciado de la práctica. Sin embargo, para los segmentos triangulares, modifiqué la tolerancia del target fijo intermedio a 3 unidades para permitir una mayor flexibilidad.

## Cálculo de la Aceleración Angular

### Cálculo del Error Angular

El error angular es la diferencia entre el ángulo al que el robot debería dirigirse y el ángulo actual del robot. Se calcula utilizando las coordenadas del objetivo relativo (target_point) y la posición del robot (poseRobot). Primero se determina el ángulo hacia el punto objetivo (angulo_a_target) usando la función atan2 para los componentes delta_x y delta_y entre el objetivo y la posición del robot. Luego, se resta la orientación actual del robot (theta), la cual está en radianes. La fórmula es:

```
error_angular = angulo_a_target - theta
```

El valor resultante se normaliza para que esté en el rango [-π, π] con la siguiente expresión:

```
error_angular = (error_angular + π) % (2 * π) - π
```

### Cálculo W en el Modelo Experto Simple

En el modelo lógico simple, W (la velocidad angular) se calcula utilizando una ganancia proporcional (k_w) y el error angular. La fórmula utilizada es la siguiente:

```
velocidad_angular_deseada = WMAX * tanh(k_w * error_angular)
```

El valor de k_w determina la sensibilidad del robot al error angular. Para limitar la aceleración, se compara la velocidad_angular_deseada con la velocidad_angular_previa, utilizando:

```
delta_w = min(max(velocidad_angular_deseada - velocidad_angular_previa, -WACC), WACC)
```

### Cálculo W en el Modelo Experto Fuzzy

En el modelo fuzzy, W se calcula utilizando un sistema de inferencia difusa que tiene en cuenta el error angular. Variables principales:

- Entrada: error_angular.
- Salida: velocidad_angular.

El resultado se defusifica utilizando el método del centroide.

## Resultados y Conclusiones

Tras probar ambos códigos en numerosas ocasiones, se concluyó que el modelo fuzzy es más adaptativo y efectivo para problemas complejos, mientras que el modelo simple es más preciso en segmentos lineales.

### Ejemplos:

- **Captura 1: Modelo no fuzzy (43 puntos)**
![Captura 1](Captura%20de%20pantalla%202024-11-17%20232645.png)

- **Captura 2: Modelo fuzzy (67 puntos)**
![Captura 2](Captura%20de%20pantalla%202024-11-19%20235007.png)

- **Captura 3: Modelo fuzzy - configuración alternativa**
![Captura 3](Captura%20de%20pantalla%202024-11-17%20233856.png)

- **Captura 4: Modelo no fuzzy - otra prueba**
![Captura 4](Captura%20de%20pantalla%202024-11-19%20225607.png)
