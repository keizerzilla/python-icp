import numpy as np
import open3d as o3d
from matplotlib import cm
from sklearn.neighbors import NearestNeighbors


def load_cloud(file_path):
    """
    Carrega nuvem de pontos e salva resultado em matriz numpy.
    
    Parâmetros
    ----------
    file_path: Caminho completo para o arquivo com a nuvem.
    
    Retornos
    --------
    xyz: Matriz (n x 3) com os pontos da nuvem.
    
    """
    
    pcd = o3d.io.read_point_cloud(file_path)
    xyz = np.asarray(pcd.points)
    
    return xyz


def draw_clouds(clouds):
    """
    Desenha em janela 3D uma lista de nuvens de pontos. Cada uma receberá uma
    cor diferente a partir de uma lista predefinida de cores.
    
    Parâmetros
    ----------
    clouds: uma lista de matrizes (n x 3)
    
    Retornos
    --------
    Nenhum.
    
    """
    
    draw = []
    color_list = np.array(cm.get_cmap("tab10").colors)[:len(clouds), :]
    for xyz, color in zip(clouds, color_list):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.paint_uniform_color(color)
        draw.append(pcd)
    
    o3d.visualization.draw_geometries(draw, width=800, height=600, window_name="plot_slice")


def best_fit_transform(A, B):
    """
    Calcula a melhor transformação entre as matrizes A e B que minimiza o erro
    quadrático entre as distância média dos pontos.
    
    Parâmetros
    ----------
    A: Matriz (n x 3) de pontos correspondentes A.
    B: Matriz (n x 3) de pontos correspondentes B.
    
    Retornos
    -------
    R: Matriz de rotação (3 x 3).
    t: Vetor de translação (3 x 1).
    
    """
    
    m = A.shape[1]
    
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    
    H = np.matmul(AA.T, B)
    U, S, V = np.linalg.svd(H)
    R = np.matmul(V.T, U.T)
    
    if np.linalg.det(R) < 0:
       V[m-1,:] *= -1
       R = np.matmul(V.T, U.T)
    
    t = centroid_B - np.matmul(R, centroid_A)
    
    return R.T, t


def matching_subclouds(src, dst):
    """
    Calcula distâncias e correspondências ponto a ponto entre duas matrizes
    usando distância euclidiana e retorna subnuvens de pontos correspondentes,
    além do vetor de distâncias para cálculo do erro.
    
    Parâmetros
    ----------
    src: Matriz (n x m) de pontos da nuvem de origem.
    dst: Matriz (n x m) de pontos da nuvem de destino.
    
    Retornos
    -------
    src_match: Sub-nuvem de origem de tamanho (min_n x 3).
    dst_match: Sub-nuvem de destino de tamanho (min_n x 3).
    dist: Vetor com (min_n) distâncias entre os pontos correspondentes.
    
    """
    
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    d, i = neigh.kneighbors(src)
    
    min_size = min([src.shape[0], dst.shape[0]])
    
    dist = d.ravel()[:min_size]
    ind = i.ravel()
    
    dst_match = dst[ind]
    src_match = src[:min_size, :]
    dst_match = dst_match[:min_size, :]
    
    return src_match, dst_match, dist


def rmse(distances):
    """
    Retorna o erro quadrático médio de um vetor de distâncias.
    
    Parâmetros
    ----------
    distances: Vetor de distâncias.
    
    Retornos
    --------
    rmse: Erro quadrático médio.
    """
    
    rmse = np.sqrt(np.mean(distances**2))
    
    return rmse


def icp(A, B, max_iterations=20, min_error=0.01, verbose=True):
    """
    Algoritmo Iterative Closest Point: encontra a melhor transformação de
    rotação R e de translação t que minimiza o erro quadrático médio entre duas
    nuvens de pontos (origem A e destino B).
    
    Parâmetros
    ----------
    A: Nuvem de pontos 3D de origem com n pontos, ou seja, matriz de dimensão (n x 3).
    B: Nuvem de pontos 3D de destino com m pontos, ou seja, matriz de dimensão (m x 3).
    max_iterations: Critério de parada 1: máximo número de iterações.
    min_error: Critério de parada 2: erro quadrático médido mínimo de uma iteração.
    verbose: Se True, mostra mensagem de acompanhamento na saída padrão.
    
    Retornos
    -------
    R: Matriz de rotação (3 x 3) que melhor corrige rotação entre A com B.
    t: Vetor de translação (3 x 1) que melhor corrige translação entre A e B.
    i: Número de iterações necessárias até final do algoritmo.
    error: Erro quadrá médio da última iteração do algoritmo.
    
    """
    
    src = np.copy(A)
    dst = np.copy(B)
    
    for i in range(1, max_iterations+1):
        src_match, dst_match, dist = matching_subclouds(src, dst)
        R, t = best_fit_transform(src_match, dst_match)
        src = np.matmul(src, R) + t
        
        error = rmse(dist)
        if error < min_error:
            break
        
        if verbose:
            print(f"Iter {i:02d} -> Error: {error:.6f}")
    
    R, t = best_fit_transform(A, src)
    
    return R, t, i, error

