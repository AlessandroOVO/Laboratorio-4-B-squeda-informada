import heapq #biblioteca que permite el uso de colas de prioridad

# Representación del laberinto
# 0 = camino, 1 = pared
# maze = [
#     [1, 0, 1, 1, 1],
#     [1, 0, 0, 0, 1],
#     [1, 1, 1, 0, 1],
#     [1, 0, 0, 0, 0],
#     [1, 1, 1, 1, 1]
# ]

# Posición de inicio y salida
# start = (0, 1)  # Coordenadas (fila, columna) de inicio
# end = (3, 4)    # Coordenadas (fila, columna) de salida

maze = [
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
    [1, 1, 1, 1, 0, 1, 0, 1, 0, 1],
    [1, 0, 0, 0, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 1, 0, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]

start = (0, 1)
end = (7, 8)


# Función heurística (distancia de Manhattan)
def heuristic(a, b):#calcula una estimación del costo para llegar desde el nodo actual hasta el nodo objetivo
    return abs(a[0] - b[0]) + abs(a[1] - b[1])#Utiliza la distancia de Manhattan, que es la suma de las diferencias absolutas entre las coordenadas de los dos puntos (a,b)

# Algoritmo A* para resolver el laberinto
def astar(maze, start, end):
    rows, cols = len(maze), len(maze[0])
    open_set = [] #lista que representa la cola de prioridad de los nodos por explorar
    heapq.heappush(open_set, (0, start))#inserta el nodo inicial en la cola con una prioridad de 0
    came_from = {} #diccionario que rastrea de dónde prpviene cada nodo
    g_score = {start: 0} #almacena el costo del camino más corto encontrado hasta el momento para llegar a cada nodo
    f_score = {start: heuristic(start, end)}#almacena la suma del costo del camino actual
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Derecha, abajo, izquierda, arriba
        for d in neighbors:
            neighbor = (current[0] + d[0], current[1] + d[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and maze[neighbor[0]][neighbor[1]] == 0:
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # No se encontró camino

# Ejecutar el algoritmo A*
if __name__ == "__main__":
    import time
    start_time = time.time()
    path = astar(maze, start, end)
    end_time = time.time()

    # Mostrar resultados
    if path:
        print("Camino encontrado:", path)
    else:
        print("No se encontró un camino.")
    
    print("Tiempo de ejecución: {:.8f} segundos".format(end_time - start_time))
