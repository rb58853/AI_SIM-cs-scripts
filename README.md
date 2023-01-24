# NavMeshAgent

**Integrantes:**
* Raúl Beltrán Gómez (C-312) ([@rb58853](https://github.com/rb58853))

**Objetivo:** Decidir y simular los movimientos óptimos de agentes de navegación que interactúan entre ellos y donde influyen condiciones climatológicas, estacionales y del terreno.

## Malla de navegación (NavMesh)
Estructura de datos abstracta utilizada en aplicaciones de inteligencia artificial para ayudar a los agentes a encontrar caminos a través de espacios complicados. Consiste en una colección de polígonos convexos bidimensionales que definen qué áreas de un entorno pueden atravesar los agentes.
* Los polígonos adyacentes se conectan entre sí en un grafo.
* Cada vértice de los polígonos posee un factor de elevación para la representación del terreno en tres dimensiones.
* A los polígonos se le pueden asociar lagos, ríos, biomas como pantanos o selvas, y los elementos del terreno relacionados con estos como árboles o rocas.
* La malla se mantiene estática durante la ejecución del programa aunque algunos elementos pueden ser modificados, por ejemplo un río puede congelarse y volverse transitable.
* Nos apoyaremos en librerías ya implementadas en Unity para la generación del terreno con malla de navegación.

## Simulación

* **Agentes de navegación:** Simulación basada en agentes inteligentes que son capaces de realizar acciones autónomas flexibles:
    * Reactivos: Perciben una vecindad del terreno y sus cambios y deciden cómo moverse en consecuencia, en dependencia además de las características específicas de cada tipo de agente.
    * Pro-activos: Se mueven con comportamiento dirigido a llegar a la posición especificada de forma óptima.
    * Sociables: Interactúan con agentes aliados mediante técnicas grupales de locomoción.

* **Condiciones ambientales:** Simulación basada en eventos discretos de situaciones que influyen en los movimientos que los agentes pueden realizar y en los costos de los mismos:          
    * Eventos climatológicos: Con el paso del tiempo y los cambios de estaciones se van sucediendo condiciones climatológicas que conllevan a cambios en el terreno, como caída de hojas en otoño o congelamiento de ríos en invierno.
    * Eventos arbitrarios: Pueden ocurrir eventos de forma aleatoria y no necesariamente relacionados con la estación, como lluvia o la caída de un árbol

## Inteligencia artificial
La toma de decisiones de los agentes se resuelve naturalmente mediante algoritmos de búsqueda de caminos de costo mínimo sobre grafos, pero como los juegos en tiempo real suelen tener entornos demasiado grandes y terrenos dinámicos, las estrategias básicas de búsqueda resultan inadecuadas. Por tanto planteamos como objetivo: Implementar y evaluar el comportamiento en distintos escenarios y con distintas heurísticas del algoritmo A*, así como algoritmos de búsqueda parcial que restringen la frontera de búsqueda en etapas alternadas de planificación y ejecución (se pierde precisión pero responde mejor a las limitaciones de recursos de tiempo y memoria) como Real-Time A*. También mejorar el desempeño de los agentes con:
* Técnicas de percepción, ordenamiento espacial y detección de colisiones
* Algoritmos para comportamientos grupales de locomoción y comunicación
* Uso de metaheurísticas para la optimización de los algoritmos anteriores, y como base del movimiento grupal


[Ejecutable en Telegram] https://t.me/+Eh4DRMgqdtYzMDlh
