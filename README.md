# Proyecto para visor geográfico en pantalla esférica

La idea del proyecto es poder crear las escenas 3D correspondientes a los proyectores recreando sus posiciones relativas a la pantalla. Luego se mandan las imágenes a cada proyector por separado y se ve cómo se hace para empalmarlas en la realidad.

# Configuración

El programa actual fue testeado en Windows 10, Windows 11 y Ubuntu 22.04 LTS. Para correrlo se requiere:

- Python 3.10.8  
- Open3D 0.17.0  
- NumPy 1.24.3  
- SciPy 1.10.1  
- rasterio 1.3.7  
- opencv-python 4.8.0.74  
- pyautogui 0.9.54

Se debe conocer previamente la resolución del monitor de la PC donde se corre el programa y la resolución de los proyectores. Esta ultima debe ser la misma para todos los proyectores. Una vez conectados, se debe organizar todos los proyectores a la derecha de la pantalla principal (el monitor), ubicándolos sobre una misma fila.

Es recomendable cuando se utiliza Windows ocultar la barra de tareas para mejorar la visualización.

Si se utiliza textura, la imagen debe encontrarse en formato PNG. En caso de que se coloreen los vértices de la malla, la imagen debe encontrarse en formato TIFF. Se permiten dos alternativas para generar la malla de la esfera:

 - Malla triangular generada automaticamente
 - Malla generada a partir de intervalos de latitud y longitud constantes

En el primer caso es posible subdividir de manera iterativa los triángulos. En el segundo puede reducirse el intervalo de latitud y longitud mediante el cual se generan los vértices.

Todas las configuraciones se realizan desde un archivo de configuración.

## Ejemplo de archivo de configuración

> [configs]
> 
>ruta_imagen = BlueMarbleNG_2004-12-01_rgb_3600x1800.png
>
>\# resolución del monitor principal
>resolucion_x_m = 1366
>resolucion_y_m = 768
>
>\# resolución de los proyectores: todos tienen que tener la misma resolucion
>resolucion_x = 640
>resolucion_y = 480
>
>\# triángulos (t) o rectángulos de intervalos de latitud/longitud \(r\)
>tipo_malla = t
>
>\# si la malla es de triángulos, es la cantidad de veces a subdividirlos
>\# si la malla es de rectángulos, es el delta de lat/lon para construirlos (en grados)
>subdivisión = 3
>
>\# yes/no
>textura = yes

# Ejecución del programa

Una vez completo el archivo de configuración y configurados los proyectores es posible ejecutar el programa mediante el siguiente comando:

    python3 proyectores.py <archivo de configuración>


# Mapa del teclado

La interacción con el programa y los proyectores se da mediante el teclado, sin la necesidad de utilizar el mouse. Las funciones de las teclas son:

 - **Q**: activa el proyector 1
 - **W**: activa el proyector 2
 - **E**: activa el proyector 3
 - **R**: activa el proyector 4
 - **O**: apaga/prende la pantalla actual
 - **P**: activa/desactiva pequeños movimientos
 - **↑**: mueve la cámara hacia arriba
 - **↓**: mueve la cámara hacia abajo
 - **←**: mueve la cámara hacia la derecha
 - **→**: mueve la cámara hacia la izquierda
 - **T**: gira la cámara alrededor de la visual en sentido horario
 - **G**: gira la cámara alrededor de la visual en sentido antihorario
 - **H**: gira la esfera sobre su eje en sentido horario
 - **L**: gira la esfera sobre su eje en sentido antihorario
 - **J**: gira la cámara hacia arriba
 - **K**: gira la cámara hacia abajo
 - **V**: acerca la cámara
 - **B**: aleja la cámara
 - **C**: activa/desactiva la escena de calibración
 - **Z**: mueve el cubo hacia abajo en la escena de calibración
 - **X**: mueve el cubo hacia arriba en la escena de calibración
 - **ESC**: finaliza el programa
