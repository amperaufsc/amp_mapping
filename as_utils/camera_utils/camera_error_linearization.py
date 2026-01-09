import numpy as np
import matplotlib.pyplot as plt

def plot_cones_and_image(error_data):

    cones_gc = np.genfromtxt(error_data,
                               delimiter = ',',
                               skip_header = 1,
                               dtype=float,
                               invalid_raise=False,
                               ndmin=2)
    
    error, dist = cones_gc[:, 0], cones_gc[:, 1]

    # Realiza a regressão linear
    coefficients = np.polyfit(dist, error, 1)

    # Os coeficientes retornados são [m, b]
    m = coefficients[0]
    b = coefficients[1]

    # Imprime a equação da reta
    print(f"A equação da reta é: y = {m:.4f}x + {b:.4f}")
    fitted_line = np.poly1d(coefficients)

    # Gera valores para x usando a reta ajustada
    x_fit = np.linspace(min(dist), max(dist), 100)
    y_fit = fitted_line(x_fit)

    # Plota os pontos e a reta ajustada
    plt.figure(figsize=(8, 5))
    plt.scatter(dist, error, label="Cones detectados", color="blue", alpha=0.7)
    plt.plot(x_fit, y_fit, label=f"Reta ajustada: y = {m:.4f}x + {b:.4f}", color="red", linestyle="--")
    
    # Configuração do gráfico
    plt.xlabel("Distância (m)")
    plt.ylabel("Erro (m)")
    plt.title("Erro da Detecção vs. Distância")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.7)
    
    plt.show()

if __name__ == "__main__":
    error_data = '/home/pedro_garbin/ws/src/as_amp/as_utils/camera_utils/error_data.csv'
    plot_cones_and_image(error_data)