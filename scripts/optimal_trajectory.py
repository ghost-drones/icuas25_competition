import numpy as np
from numpy import ndarray
from typing import List, Dict, Tuple, Union
import math
from icuas25_msgs.msg import Waypoints,WaypointInfo
from copy import deepcopy

# #Exemplo: dicionário com posições (apenas para teste)
# points: List[int] = [0, 1, 2, 3, 4, 5, 6, 48, 89, 96, 115, 120, 51, 160, 10]
# distances: Dict[int, ndarray] = {}
# for point in points:
#     distances[point] = np.random.rand(3) * 100

Graph = Dict[int, Dict[int, float]]  # Grafo: nó -> {nó vizinho: distância}
Trajectory = List[Union[int, List[int]]] #Estrutura de dados do input, uma trajetória

def _get_position_xyz_array_by_id(id: int, points_ids:Waypoints) -> ndarray:
    """
        Retorna posição em x,y,z de um ponto a partir de seu id.
        Args:
            id1: id do ponto a ser descoberto, int
            points_ids: Conjunto de todos os Pontos atrelados aos seus ids, Waypoints
        Return:
            point: vetor x,y,z de posição do ponto atraldo ao id, ndarray
    """
    # global distances ((para testes)
    # return distances[id]
    
    if id != 0:
        for waypoint in points_ids.waypoints:
            if isinstance(waypoint,WaypointInfo):
                if  waypoint.id == id:
                    position = waypoint.pose.position
                    return np.array([position.x,position.y,position.z])
    return np.array([0.0,0.0,0.0])

def _calculate_distance_between_id(id1: int, id2: int, points_ids:Waypoints) -> float:
    """
        Calcula distância euclidiana entre dois pontos.
        Args:
            id1: id do primeiro ponto, int
            id2: id do segundo ponto, int
            points_ids: Conjunto de todos os Pontos atrelados aos seus ids, Waypoints
        Return:
            distance: distancia normalizada entre os dois pontos, float
    """
    pos1 = _get_position_xyz_array_by_id(id1,points_ids)
    pos2 = _get_position_xyz_array_by_id(id2,points_ids)
    return np.linalg.norm(pos1 - pos2)

def _input_extractor_for_path_list(input:Trajectory) ->  List[List[int]]:
    """
        Extrai todas as paths a um cluster de exploração, será usada para construir um grafo.
        Args:
            input: Trajetória que um drone deve seguir, Trajectory (List[Union[int, List[int]]])
        Return:
            paths: Conjunto de paths que levam a todos os clusters, List[List[int]]
    """
    paths:List[List[int]] = []
    
    path:List[int] = []
    sub_path:List[int] = []
    
    for elemet in input:
        if isinstance(elemet,list):
            continue
        else:
            path.append(elemet)

    for element in path:
        if element == 0:
            sub_path.append(element)
            sub_path.insert(0,0)
            half_way = len(sub_path) // 2
            paths.append(deepcopy(sub_path[:half_way]))
            sub_path.clear()
        else:
            sub_path.append(element)
    return paths

def _build_graph(input_paths: List[List[int]],points_ids:Waypoints) -> Graph:
    """
        Constrói um grafo em estrutura de árvore, Cada pai só pode ter filhos de ordem imediatamente inferior, cada filho só pode ter um pai.
        Todos os nós irmãos possuem arestas. As arestas representam a distancia entre os pontos e os nós, os pontos.
        A não existencia de vertices entre todos os nós garante o alinhamento, base do problema.
               
        Args:
            input_paths: Conjunto de paths que levam a todos os clusters, List[List[int]]
            points_ids: Conjunto de todos os Pontos atrelados aos seus ids, Waypoints
        Return:
            graph: Grafo constrúido, Graph
    """
    graph: Graph = {}
    # Dicionário para armazenar, para cada pai, seus filhos
    parent_children: Dict[int, List[int]] = {}

    # Etapa 1: Conectar pai e filho (apenas conexões entre nós consecutivos)
    for path in input_paths:
        if len(path) < 2:
            continue  # Nada a conectar se tiver só um nó
        parent =  path[-2]
        child = path[-1]
        
        if parent not in graph:
            graph[parent] = {}
        if child not in graph:
            graph[child] = {}

        # Conecta pai e filho, se ainda não estiver conectado
        if child not in graph[parent]:
            dist = _calculate_distance_between_id(parent, child,points_ids)
            graph[parent][child] = dist
            graph[child][parent] = dist

        # Registra o filho para o pai (para depois conectar irmãos)
        if parent in parent_children:
            if child not in parent_children[parent]:
                parent_children[parent].append(child)
        else:
            parent_children[parent] = [child]

    # Etapa 2: Conectar irmãos (nós que compartilham o mesmo pai)
    for parent, children in parent_children.items():
        for i in range(len(children)):
            for j in range(i + 1, len(children)):
                a = children[i]
                b = children[j]
                # Se ainda não houver conexão entre irmãos, cria
                if b not in graph[a]:
                    dist = _calculate_distance_between_id(a, b, points_ids)
                    graph[a][b] = dist
                    graph[b][a] = dist

    return graph

def _floyd_warshall(graph: Graph) -> Tuple[Dict[int, Dict[int, float]], Dict[int, Dict[int, int]]]:
    nodes = list(graph.keys())
    # Inicializa as matrizes de distância e de próximo nó
    dist = {u: {v: math.inf for v in nodes} for u in nodes}
    nxt = {u: {v: None for v in nodes} for u in nodes}
    
    for u in nodes:
        dist[u][u] = 0
        nxt[u][u] = u
        for v, w in graph[u].items():
            dist[u][v] = w
            nxt[u][v] = v
            
    for k in nodes:
        for i in nodes:
            for j in nodes:
                if dist[i][k] + dist[k][j] < dist[i][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    nxt[i][j] = nxt[i][k]
    
    return dist, nxt

def _reconstruct_path(u: int, v: int, nxt: Dict[int, Dict[int, int]]) -> List[int]:
    """Reconstrói o caminho de u a v usando a matriz de próximos nós."""
    if nxt[u][v] is None:
        return []  # caminho inexistente
    path = [u]
    while u != v:
        u = nxt[u][v]
        path.append(u)
    return path

def _held_karp_tsp(nodes: List[int], cost: Dict[int, Dict[int, float]]) -> Tuple[float, List[int]]:
    """
    Retorna o custo mínimo e a ordem dos nós (como índices na lista 'nodes') 
    para o TSP, partindo e terminando em nodes[0] (nó 0).
    """
    n = len(nodes)
    dp = {}      # dp[(mask, j)] = custo mínimo para visitar o conjunto 'mask' (bits) terminando no nó j
    parent = {}  # Para reconstrução do caminho

    start = 0  # Considera que o nó 0 é o início
    dp[(1 << start, start)] = 0

    # Itera sobre todos os subconjuntos de nós
    for mask in range(1 << n):
        for j in range(n):
            if (mask, j) in dp:
                for k in range(n):
                    if mask & (1 << k) == 0:  # se o nó k ainda não foi visitado
                        next_mask = mask | (1 << k)
                        new_cost = dp[(mask, j)] + cost[nodes[j]][nodes[k]]
                        if (next_mask, k) not in dp or new_cost < dp[(next_mask, k)]:
                            dp[(next_mask, k)] = new_cost
                            parent[(next_mask, k)] = j

    full_mask = (1 << n) - 1
    best_cost = math.inf
    last = None
    for j in range(n):
        if (full_mask, j) in dp:
            cur_cost = dp[(full_mask, j)] + cost[nodes[j]][nodes[start]]
            if cur_cost < best_cost:
                best_cost = cur_cost
                last = j

    # Reconstrução do caminho (em índices)
    mask = full_mask
    route_indices = [last]
    while mask != (1 << start):
        j = route_indices[-1]
        i = parent[(mask, j)]
        route_indices.append(i)
        mask = mask ^ (1 << j)
    route_indices.reverse()
    route_indices.append(start)  # Completa o ciclo voltando ao início

    # Converte índices para os rótulos reais dos nós
    route = [nodes[i] for i in route_indices]
    return best_cost, route

def _best_path_generator(graph: Graph) -> List[int]:
    """
        Recebe um grafo e usa um algoritmo de floyd warshall e de held karp tsp para calcular o melhor caminho entre os clusters.
               
        Args:
            graph: Grafo que será utilizado, Graph
        Return:
            full_route: melhor rota calculada, List[int]
    """
    
    nodes = list(graph.keys())
    
    # Etapa 1: Computa os caminhos mínimos (custo e rota) entre todos os nós
    complete_cost, nxt = _floyd_warshall(graph)
    
    # Etapa 2: Resolve o TSP (usando os custos da matriz completa)
    best_cost, tsp_route = _held_karp_tsp(nodes, complete_cost)
    # tsp_route é uma lista de nós (ex: [0, 48, 115, 120, 96, 51, 160, 89, 10, 0])
    
    # Etapa 3: Expande o caminho TSP para incluir os nós intermediários (caso o caminho direto não exista)
    full_route = []
    for i in range(len(tsp_route) - 1):
        segmento = _reconstruct_path(tsp_route[i], tsp_route[i+1], nxt)
        if i > 0:
            # Evita duplicar o nó de conexão
            segmento = segmento[1:]
        full_route.extend(segmento)
    
    return full_route

def _construct_cluster_path(trajectory_id:Trajectory) -> Dict[int,Trajectory]:
    cluster_path:Dict[int,Trajectory] = {}
    sub_path:Trajectory = []
    sub_paths:List[Trajectory] = []
    count = 1
    for element in trajectory_id:
        if isinstance(element,list):
            sub_path.append(element)
            count+=1
        else:    
            sub_path.append(element)
        if count == 2 or element == 0:
            sub_path.insert(0,0)
            sub_paths.append(deepcopy(sub_path))
            sub_path.clear()
            count = 0
    
    sub_paths[0] = [sub_paths[0][1]]
    sub_paths.remove([0,0])
    for sub in sub_paths:
        int_save = 0
        for s in sub:
            if isinstance(s,int):
                int_save = s
            else:
                break
        cluster_path[int_save] = sub
    return cluster_path

def _remove_duplicate_zeros(lst: Trajectory) -> Trajectory:
    result:Trajectory = []
    prev_was_zero = False  # Flag para acompanhar zeros consecutivos
    
    for item in lst:
        if item == 0:
            if not prev_was_zero:  # Adiciona o primeiro zero encontrado
                result.append(item)
            prev_was_zero = True
        else:
            result.append(item)
            prev_was_zero = False  # Reseta a flag ao encontrar um número diferente de zero
            
    return result

def _reconstruct_trajectory_id(best_path:List[int],trajectory_id:Trajectory) -> Trajectory:
    """
        Usa a best_path como máscara para reorganizar o trajectory_id e retorna um novo com o melhor caminho.

        Args:
            best_path: melhor caminho encontrado, List[int]
            trajectory_id: trajetória original, Trajectory
        Return:
            output_list: nova trajetória, Trajectory
    """
    output_list:Trajectory = []
    aux_List:List[int] = []
    cluster_path:Dict[int,List] = _construct_cluster_path(trajectory_id) #dicionario que relaciona cluster com sua path
    
    for element_i in best_path:
        if element_i in cluster_path.keys() and not element_i in aux_List:
            aux_List.append(element_i)
            for element_j in cluster_path[element_i]:
                output_list.append(element_j)
    
    return _remove_duplicate_zeros(output_list)

def compute_optimal_trajectory_mapping(trajectory_ids:Dict[int,Trajectory],points_ids:Waypoints) -> Trajectory:
    """
        Função principal e a Unica externa, ela une todas as funções locais e gera o objetivo final.
        Ela recebe um trajectory_ids e retorna reordenado com a melhor ordem.

        Args:
            graph: Grafo que será utilizado, Graph
            points_ids: Conjunto de todos os Pontos atrelados aos seus ids, Waypoints
        Return:
            full_route: melhor rota calculada, List[int]
    """
    
    trajectory_id = trajectory_ids[4]
    input_paths = _input_extractor_for_path_list(trajectory_id)
    graph = _build_graph(input_paths,points_ids)
    best_path = _best_path_generator(graph)
    new_trajectory_id:Dict[int,List] = {}
    for drone_id in trajectory_ids:
        new_trajectory_id[drone_id] = _reconstruct_trajectory_id(best_path,trajectory_ids[drone_id])
    return new_trajectory_id

# trajectory_id:Trajectory = [[88,133, 134], 0, 48, [173, 170, 167, 18, 17, 16, 1, 2, 3, 4, 5, 20, 19, 21], 48, 0, 89, [13, 12, 11, 129, 130, 132, 15, 0], 89, 0, 96, [146, 148, 149, 149, 148, 140], 96, 0, 96, 51, [51], 51, 96, 0, 89, 10, [24, 23, 22, 7, 6, 8, 9, 10], 10, 89, 0, 48, 115, [68, 156, 154, 153, 151], 115, 48, 0, 1, [50], 1,  0, 1, 2, [52], 2 ,1, 0]
# trajectory_ids:Dict[int,List] = {}
# trajectory_ids[1] =  trajectory_id
# retorno = compute_optimal_trajectory_mapping(trajectory_ids)
# print(retorno[1])
