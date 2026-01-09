import matplotlib.pyplot as plt
import numpy as np
import csv

def plot_cones_and_image(cone_gc_path, cone_measure_path):

    csv_file = open('error_data.csv', mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['error', 'dist'])
    
    # Carrega os dados do arquivo, ignorando as colunas de y e confiabilidade  
    cones_gc = np.genfromtxt(cone_gc_path,
                               delimiter = ',',
                               skip_header = 1,
                               dtype=float,
                               invalid_raise=False,
                               ndmin=2)
    
    cones_measure = np.genfromtxt(cone_measure_path,
                               delimiter = ',',
                               skip_header = 1,
                               dtype=float,
                               invalid_raise=False,
                               ndmin=2)

    # Assume que o arquivo contém colunas de cor, x, y, z
    x_gc, z_gc = cones_gc[:, 0], cones_gc[:, 1]

    x_m, z_m = cones_measure[:, 0], cones_measure[:, 1]

    error_values = []    

    for cone in cones_measure:
        erro = np.min([np.linalg.norm([cone[0] - gc_point[0], cone[1] - gc_point[1]]) for gc_point in cones_gc])
        dist = np.linalg.norm([cone[0], cone[1]])

        error_values.append([erro, dist])
        csv_writer.writerow([erro, dist])

    # Cria um gráfico 2D
    plt.figure()

    # Plotagem de cada cone com cores correspondentes dentro dos limites
    plt.scatter(x_gc, z_gc, c='r', label="Posição real", marker='o')
    plt.scatter(x_m, z_m, c='b', label="Posição detectada", marker='x')

    # Configuração de visualização
    plt.xlabel('X')
    plt.ylabel('Z')
    plt.title('Comparação entre Cones Reais e Detectados')

    # Mostra o gráfico
    plt.legend()
    plt.grid(True, which='both')
    plt.show()

if __name__ == "__main__":

    cone_gc_path = '/home/pedro_garbin/ws/src/as_amp/as_utils/rosbag_converter/Track_gorund_truth.csv'
    cone_measure_path = '/home/pedro_garbin/ws/src/as_amp/as_utils/rosbag_converter/track_data.csv'  # Substitua pelo caminho real da sua imagem
  
    plot_cones_and_image(cone_gc_path, cone_measure_path)