import numpy as np

class CoordinateTransforms:

    '''
     * @brief Constrói um objeto de transformações de coordenadas a partir de uma referência geodésica (lat, lon, alt).
     *
     * Converte latitude/longitude para radianos, pré-computa seno/cosseno e monta a matriz de rotação
     * usada para converter vetores em ECEF (Δx, Δy, Δz) para ENU (east, north, up) na referência.
     *
     * @param lat Latitude da referência (graus)
     * @param lon Longitude da referência (graus)
     * @param alt Altitude da referência (metros) 
    '''
    def __init__(self, lat, lon, alt):
        self.lat_rad = np.radians(lat)
        self.lon_rad = np.radians(lon)
        self.alt = alt

        self.sin_lat = np.sin(self.lat)
        self.cos_lat = np.cos(self.lat)
        self.sin_lon = np.sin(self.lon)
        self.cos_lon = np.cos(self.lon)

        self.rotation_matrix = np.array([[-self.sin_lon, self.cos_lon, 0],
                                    [-self.cos_lon*self.sin_lat, -self.sin_lon*self.sin_lat, self.cos_lat],
                                    [self.cos_lon*self.cos_lat, self.sin_lon*self.cos_lat, self.sin_lat]])

    '''
     * @brief Calcula o vetor diferença (Δx, Δy, Δz) entre um alvo ECEF e a referência (lat, lon, alt) convertida para ECEF.
     *
     * Usa parâmetros do elipsóide WGS84 (semi-eixo maior A e excentricidade ao quadrado E2) para converter
     * a referência geodésica (lat, lon, alt) em coordenadas ECEF e então subtrair do alvo.
     *
     * @param target_x Coordenada X do alvo em ECEF (metros)
     * @param target_y Coordenada Y do alvo em ECEF (metros)
     * @param target_z Coordenada Z do alvo em ECEF (metros)
     * @return Tupla (dx, dy, dz) em metros
    '''   
    def ecefDifference(self, target_x, target_y, target_z):
        A = 6378137.0
        E2 = 0.00669437999014
        
        n = A / np.sqrt(1 - E2 * self.sin_lat**2)
        ref_x = (n + self.alt) * self.cos_lat * self.cos_lon
        ref_y = (n + self.alt) * self.cos_lat * self.cos_lon
        ref_z = (n * (1 - E2) + self.alt) * self.sin_lat
        
        dx = target_x - ref_x
        dy = target_y - ref_y
        dz = target_z - ref_z

        return dx, dy, dz

    '''
     * @brief Converte coordenadas ECEF de um alvo para coordenadas locais ENU (east, north, up) na referência.
     *
     * Primeiro calcula o vetor diferença em ECEF (Δx, Δy, Δz) via ecefDifference().
     * Em seguida aplica a matriz de rotação pré-computada (ECEF -> ENU) para obter (E, N, U).
     *
     * @param target_x Coordenada X do alvo em ECEF (metros)
     * @param target_y Coordenada Y do alvo em ECEF (metros)
     * @param target_z Coordenada Z do alvo em ECEF (metros)
     * @return Tupla (east, north, up) em metros
     '''
    def ecefToEnu(self, target_x, target_y, target_z):
        dx, dy, dz = self.ecefDifference(target_x, target_y, target_z)

        ecef_matrix = np.array([[dx],
                                [dy],
                                [dz]])

        enu_matrix = self.rotation_matrix @ ecef_matrix

        east = enu_matrix[0][0]
        north = enu_matrix[1][0]
        up = enu_matrix[2][0]

        return east, north, up