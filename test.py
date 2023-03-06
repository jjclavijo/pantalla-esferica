import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui


def crear_esfera():
    """
    Crear una esfera pintada
    """

    # Crear una esfera de radio 1
    esfera = o3d.geometry.TriangleMesh.create_sphere(1)
    # Internet dice que esto sirve para que se vea bien.
    esfera.compute_vertex_normals()

    # Setear los colores de la esfera. Acá podría cargarse
    # consultando por posición los datos de un mapa.
    esfera.vertex_colors = o3d.utility.Vector3dVector(
        np.array(
            [
                np.linspace(0, 30, 762) % 1,
                (np.linspace(0, 30, 762) % 1).reshape(6, -1).T.flatten(),
                np.linspace(1, 0, 762),
            ]
        ).T
    )
    return esfera

# ---------------------
# GLOBALS
# ---------------------

# Aunque no nos guste usar globals, es la primera forma que encontré de
# tener disponibles los datos de las cuatro cámaras y compartir la geometría
# representada.
# Lo lógico sería armar una clase o diccionario contenedor y ir pasandolo entre
# llamadas a funciones. Así también se puede hacer pickle de eso para guardar
# proyectos ya creados y calibrados.

geometrias = []
windows = []
look_at_list = []
widgets = []
scenes = []

# ----------------------
# Hasta acá los Globals
# ----------------------

def agregar_camara(posicion,nombre=''):
    """
    Crear una ventana con una camara en la posición deseada, apuntando hacia el centro.
    Se crea la camara con la escena y el widget correspondiente y se carga la
    geometría.

    Posicion: array de dimensión 3.
    """

    # Traer los globals
    global windows
    global widgets
    global scenes
    global look_at_list

    if not nombre:
        nombre = f'{len(windows)+1}'

    # Crear una ventana
    w = gui.Application.instance.create_window(f"Camara {nombre}", 640, 480)
    # Agregarle el widget contenedor de la escena
    scene_widget = gui.SceneWidget()
    # crear la escena (indicando la ventana en que se va a renderizar)
    scene = o3d.visualization.rendering.Open3DScene(w.renderer)

    # Agregar las geometrías
    for g in geometrias:
        scene.add_geometry(*g)

    # Asigna la escena al widget
    scene_widget.scene = scene

    # Configurar la cámara
    scene_widget.setup_camera(60, scene.bounding_box, (0, 0, 0)) #???
    scene_widget.look_at((0,0,0),posicion,(0,0,1))

    # Agregar el widget a la ventana.
    w.add_child(scene_widget)


    # Agregar los objetos a las listas globales.
    windows.append(w)
    widgets.append(scene_widget)
    scenes.append(scene)
    look_at_list.append(posicion)

    return 0


def rotm(degree):
    """
    un helper para crear una matriz de rotación alrededor del eje Z
    """
    return np.array(
        [
            [np.cos(degree), -np.sin(degree), 0],
            [np.sin(degree), np.cos(degree), 0],
            [0, 0, 1],
        ]
    )

def get_on_key(index):
    """
    Los mapas de teclado se setean por cada widget.
    Como son todos iguales creamos una función generadora de mapas, que crea la
    función adecuada que recibe un evento de teclado y hace lo que tiene que
    hacer.
    """
    def on_key(e):
        """
        Un mapa de teclas para ejecutar las acciones necesarias para el
        proyecto. A hoy lo que hace es rotar la camara alrededor del centro de
        la esfera con H o L, y cambiar la distancia de la cámara con J y K,
        """

        # Cargar los globals, a los que vamos a acceder por índice.
        global look_at_list
        global widgets

        if e.key == gui.KeyName.L:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Rotar en un sentido.
                R = rotm(1 * np.pi / 180)
                look_at_list[index] = look_at_list[index] @ R

                # Actualizar la cámara.
                widgets[index].look_at((0, 0, 0), look_at_list[index],  (0, 0, 1))
            return gui.Widget.EventCallbackResult.HANDLED

        if e.key == gui.KeyName.H:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Rotar en el otro sentido.
                R = rotm(-1 * np.pi / 180)
                look_at_list[index] = look_at_list[index] @ R

                # Actualizar la cámara.
                widgets[index].look_at( (0, 0, 0),look_at_list[index], (0, 0, 1))
            return gui.Widget.EventCallbackResult.HANDLED

        if e.key == gui.KeyName.J:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Alejar la cámara
                lookat = np.array(look_at_list[index])
                look_at_list[index] = lookat + 0.5* lookat/np.sum(lookat**2)**0.5
                widgets[index].look_at( (0, 0, 0),look_at_list[index], (0, 0, 1))
            return gui.Widget.EventCallbackResult.HANDLED

        if e.key == gui.KeyName.K:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Acercar la cámara
                lookat = np.array(look_at_list[index])
                look_at_list[index] = lookat - 0.5 * lookat/np.sum(lookat**2)**0.5
                widgets[index].look_at( (0, 0, 0),look_at_list[index], (0, 0, 1))
            return gui.Widget.EventCallbackResult.HANDLED

        return gui.Widget.EventCallbackResult.IGNORED
    return on_key

if __name__ == "__main__":
    gui.Application.instance.initialize()

    # Creamos la esfera.
    geometrias.append( ('esfera',crear_esfera(),o3d.visualization.rendering.MaterialRecord()))

    # En el ejemplo creamos cuatro cámaras.
    posiciones_de_camara = [ [ 10,  0,0] ,
                             [  0, 10,0] ,
                             [-10,  0,0] ,
                             [  0,-10,0] ]

    for ix,pos in enumerate(posiciones_de_camara):
        agregar_camara(pos)
        widgets[ix].set_on_key(get_on_key(ix))

    gui.Application.instance.run()
