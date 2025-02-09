import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
import numpy as np

class OctomapToGraph(Node):
    def __init__(self):
        super().__init__('OctomapToGraph')
        
        # Create a publisher for the Octomap messages
        self.sub_octomap = self.create_subscription(
            Octomap,
            '/octomap',
            self.graph_generator,
            10)
        
        # Lista para armazenar os nós do grafo
        self.graph_nodes = []

        # Parâmetros do drone e take
        self.altura_do_drone = 0.05  # m
        self.base_do_drone = 0.07   # m
        self.profundidade_do_drone = 0.10  # m
        self.volume_drone = self.altura_do_drone * self.base_do_drone * self.profundidade_do_drone
        
        self.distancia_drone_predio = 3.0  # m
        self.altura_take_drone = 5.0       # m
        self.base_take_drone = 5.0         # m

        self.altura_marcador = 0.1  # m
        self.base_marcador = 0.1    # m
        
    def graph_generator(self, data: Octomap):
        """
        Processa o Octomap para gerar os nós do grafo.
        """
        try:
            voxels = self.converter_octomap(data)
        except Exception as e:
            self.get_logger().error(f"Erro na conversão do Octomap: {e}")
            return

        # Percorrer os voxels para identificar as superfícies dos prédios
        for voxel in voxels:
            if self.voxel_ocupado(voxel):
                # Verifica se é um voxel de superfície (ou seja, se tem vizinhos livres)
                if self.voxel_e_superficie(voxel, voxels):
                    # Calcula a normal da superfície
                    normal = self.calcula_normal(voxel, voxels)
                    
                    # Calcula a posição do nó deslocada a 3m na direção contrária à normal
                    posicao_no = np.array(voxel['centro']) - normal * self.distancia_drone_predio
                    
                    # # Ajusta a altura do drone para a altura do take
                    # posicao_no[2] = self.altura_take_drone
                    
                    # Armazena o nó (aqui representado como um dicionário)
                    self.graph_nodes.append({
                        'posicao': posicao_no.tolist(),
                        'normal': normal.tolist()
                    })

        self.get_logger().info(f"Nós do grafo gerados: {len(self.graph_nodes)}")
    
    def converter_octomap(self, data: Octomap):
        """
        Converter os dados do Octomap em uma lista de voxels.
        Cada voxel pode ser representado como um dicionário com informações como:
        - 'centro': [x, y, z]
        - 'tamanho': dimensão do voxel
        - 'ocupado': booleano
        """
        
        # Aqui vamos supor que retornamos uma lista de voxels.
        voxels = {}
        # ... código de conversão ...
        return voxels
    
    def voxel_ocupado(self, voxel):
        """
        Verifica se o voxel está ocupado.
        """
        return voxel.get('ocupado', False)
    
    def voxel_e_superficie(self, voxel, voxels):
        """
        Determina se o voxel faz parte da superfície do prédio.
        Uma estratégia é verificar se pelo menos um vizinho (em 6 direções) não está ocupado.
        """
        return True  # Para fins de exemplo

    def calcula_normal(self, voxel, voxels):
        """
        Calcula a normal da superfície no voxel dado.
        Pode ser feita através de uma técnica de regressão ou olhando para os vizinhos livres.
        Retorna um vetor unitário (np.array) com a normal.
        """
        return np.array([0, 1, 0])  # Exemplo: normal apontando na direção y positiva

def main(args=None):
    rclpy.init(args=args)
    octomap = OctomapToGraph()
    
    try:
        rclpy.spin(octomap)
    except KeyboardInterrupt:
        octomap.get_logger().info('Shutting down the octomap graph generator...')
    finally:
        octomap.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()