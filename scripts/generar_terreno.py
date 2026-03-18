import numpy as np
import cv2
import os

def generar_obj_terreno_rugoso(filename="lunar_terrain.obj", grid_size=200):
    size_px = grid_size
    terreno = np.full((size_px, size_px), 127, dtype=np.uint8)

    # 1. Macro-relieve (Crater, Surco, Pendiente Principal)
    cv2.circle(terreno, (int(size_px*0.3), int(size_px*0.3)), int(size_px*0.15), 50, -1) 
    cv2.line(terreno, (int(size_px*0.1), int(size_px*0.8)), (int(size_px*0.8), int(size_px*0.7)), 60, int(size_px*0.04))
    cv2.ellipse(terreno, (int(size_px*0.75), int(size_px*0.25)), (int(size_px*0.1), int(size_px*0.2)), 45, 0, 360, 200, -1)

    terreno_base = cv2.GaussianBlur(terreno, (31, 31), 0)

    # 2. Micro-relieve (Montículos de arena y pedregosidad)
    # Generamos ruido en baja resolución y lo escalamos para crear "bultos" suaves
    ruido_baja_res = np.random.normal(0, 30, (int(size_px/10), int(size_px/10))).astype(np.float32)
    monticulos = cv2.resize(ruido_baja_res, (size_px, size_px), interpolation=cv2.INTER_CUBIC)
    
    # Combinamos el terreno base con los montículos
    # Limitamos los valores para que sigan siendo válidos (0-255)
    terreno_final = np.clip(terreno_base.astype(np.float32) + monticulos, 0, 255).astype(np.uint8)
    
    # Suavizamos un poco el resultado final para que el rover no se atasque
    terreno_final = cv2.GaussianBlur(terreno_final, (5, 5), 0)

    # Convertir a metros (Max elevación ~0.5m)
    Z = (terreno_final / 255.0) * 0.5
    dx = 10.0 / (size_px - 1)
    dy = 10.0 / (size_px - 1)

    print("Calculando normales para la iluminación de las dunas...")
    dzdy, dzdx = np.gradient(Z, dy, dx)
    mag = np.sqrt(dzdx**2 + dzdy**2 + 1.0)
    nx = -dzdx / mag
    ny = -dzdy / mag
    nz = 1.0 / mag

    print("Escribiendo malla 3D pedregosa, por favor espera...")
    with open(filename, 'w') as f:
        f.write("# Terreno Lunar Pedregoso M4F00CV\n")
        
        for y in range(size_px):
            for x in range(size_px):
                vx = (x / (size_px - 1)) * 10.0 - 5.0
                vy = (y / (size_px - 1)) * 10.0 - 5.0
                vz = Z[y, x]
                f.write(f"v {vx:.4f} {vy:.4f} {vz:.4f}\n")
                f.write(f"vn {nx[y, x]:.4f} {ny[y, x]:.4f} {nz[y, x]:.4f}\n")
        
        for y in range(size_px - 1):
            for x in range(size_px - 1):
                v1 = y * size_px + x + 1
                v2 = v1 + 1
                v3 = (y + 1) * size_px + x + 1
                v4 = v3 + 1
                f.write(f"f {v1}//{v1} {v2}//{v2} {v3}//{v3}\n")
                f.write(f"f {v2}//{v2} {v4}//{v4} {v3}//{v3}\n")
                
    print(f"Malla 3D generada con éxito en: {os.path.abspath(filename)}")

if __name__ == '__main__':
    ruta_salida = os.path.join(os.path.dirname(__file__), '../worlds/lunar_terrain.obj')
    generar_obj_terreno_rugoso(ruta_salida)
